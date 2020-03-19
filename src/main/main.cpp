/*
 * This file is part of Cleanflight and Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/atomic.h"
#include "common/maths.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/ranging_vl53l0x.h"
#include "rx/rx.h"

#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/serial_msp.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"
#include "flight/altitudehold.h"
#include "flight/posEstimate.h"
#include "flight/opticflow.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "drivers/opticflow_paw3903.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "build_config.h"

#include "command/localisationCommand.h"

#include "API/PlutoPilot.h"
#include "API/API-Utils.h"
#include "API/Utils.h"
#include "API/Peripheral.h"
#include "API/Localisation.h"
#include "API/Motor.h"


extern uint32_t previousTime;
extern uint8_t motorControlEnable;

//uint32_t userCurrentTime = 0;
//uint32_t userLoopTime = 0;

#ifdef SOFTSERIAL_LOOPBACK
serialPort_t *loopbackPort;
#endif

#ifdef __cplusplus
extern "C" {
#endif

void printfSupportInit(void);
void timerInit(void);
void telemetryInit(void);
void serialInit(serialConfig_t *initialSerialConfig, bool softserialEnabled);
void mspInit(serialConfig_t *serialConfig);
void cliInit(serialConfig_t *serialConfig);
void failsafeInit(rxConfig_t *intialRxConfig, uint16_t deadband3d_throttle);
pwmOutputConfiguration_t *pwmInit(drv_pwm_config_t *init);
void rxInit(rxConfig_t *rxConfig, modeActivationCondition_t *modeActivationConditions);
void gpsInit(serialConfig_t *serialConfig, gpsConfig_t *initialGpsConfig);
void imuInit(void);
void displayInit(rxConfig_t *intialRxConfig);
void ledStripInit(ledConfig_t *ledConfigsToUse, hsvColor_t *colorsToUse);
void loop(void);
void spektrumBind(rxConfig_t *rxConfig);
const sonarHardware_t *sonarGetHardwareConfiguration(batteryConfig_t *batteryConfig);
void sonarInit(const sonarHardware_t *sonarHardware);

uint8_t failureFlag;

#ifdef STM32F303xC
// from system_stm32f30x.c
uint32_t SetSysClock(void);
#endif
#ifdef STM32F10X
// from system_stm32f10x.c
uint32_t SetSysClock(bool overclock);
#endif

#ifdef __cplusplus
}
#endif


typedef enum {
SYSTEM_STATE_INITIALISING = 0,
SYSTEM_STATE_CONFIG_LOADED = (1 << 0),
SYSTEM_STATE_SENSORS_READY = (1 << 1),
SYSTEM_STATE_MOTORS_READY = (1 << 2),
SYSTEM_STATE_READY = (1 << 7)
} systemState_e;

static uint8_t systemState = SYSTEM_STATE_INITIALISING;
uint32_t clockcheck;
void init(void)
{
uint8_t i;
drv_pwm_config_t pwm_params;

printfSupportInit();

initEEPROM();

ensureEEPROMContainsValidData();
readEEPROM();

systemState |= SYSTEM_STATE_CONFIG_LOADED;

#ifdef STM32F303
// start fpu
SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));
#endif

#ifdef STM32F303xC
clockcheck = SetSysClock();  //DD
#endif
#ifdef STM32F10X
// Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers
// Configure the Flash Latency cycles and enable prefetch buffer
clockcheck = SetSysClock(masterConfig.emf_avoidance);
#endif

#ifdef USE_HARDWARE_REVISION_DETECTION
detectHardwareRevision();
#endif

systemInit();

// Latch active features to be used for feature() in the remainder of init().
latchActiveFeatures();

ledInit();

#ifdef SPEKTRUM_BIND
if (feature(FEATURE_RX_SERIAL)) {
    switch (masterConfig.rxConfig.serialrx_provider) {
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        // Spektrum satellite binding if enabled on startup.
        // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
        // The rest of Spektrum initialization will happen later - via spektrumInit()
        spektrumBind(&masterConfig.rxConfig);
        break;
    }
}
#endif

//timerDataConfiguration();
//
//delay(300);
//
//timerInit();  // timer must be initialized before any channel is allocated

serialInit(&masterConfig.serialConfig, feature(FEATURE_SOFTSERIAL));

/*
#ifdef USE_SERVOS
mixerInit((mixerMode_e)masterConfig.mixerMode, masterConfig.customMotorMixer, masterConfig.customServoMixer);
#else
mixerInit((mixerMode_e) masterConfig.mixerMode, masterConfig.customMotorMixer);
#endif
*/

mixerInit((mixerMode_e) masterConfig.mixerMode, masterConfig.customMotorMixer);

memset(&pwm_params, 0, sizeof(pwm_params));

#ifdef SONARz
const sonarHardware_t *sonarHardware = NULL;

if (feature(FEATURE_SONAR)) {
    sonarHardware = sonarGetHardwareConfiguration(&masterConfig.batteryConfig);
    sonarGPIOConfig_t sonarGPIOConfig = {.gpio = SONAR_GPIO, .triggerPin = sonarHardware->echo_pin, .echoPin = sonarHardware->trigger_pin,};
    pwm_params.sonarGPIOConfig = &sonarGPIOConfig;
}
#endif

//// when using airplane/wing mixer, servo/motor outputs are remapped
//if (masterConfig.mixerMode == MIXER_AIRPLANE
//        || masterConfig.mixerMode == MIXER_FLYING_WING
//        || masterConfig.mixerMode == MIXER_CUSTOM_AIRPLANE)
//    pwm_params.airplane = true;
//else
//    pwm_params.airplane = false;
//#if defined(USE_USART2) && defined(STM32F10X)
//pwm_params.useUART2 = doesConfigurationUsePort(SERIAL_PORT_USART2);
//#endif
//#ifdef STM32F303xC
//pwm_params.useUART3 = doesConfigurationUsePort(SERIAL_PORT_USART3);
//#endif
//pwm_params.useVbat = feature(FEATURE_VBAT);
//pwm_params.useSoftSerial = feature(FEATURE_SOFTSERIAL);
//pwm_params.useParallelPWM = feature(FEATURE_RX_PARALLEL_PWM);
//pwm_params.useRSSIADC = feature(FEATURE_RSSI_ADC);
//pwm_params.useCurrentMeterADC = feature(FEATURE_CURRENT_METER)
//        && masterConfig.batteryConfig.currentMeterType
//                == CURRENT_SENSOR_VIRTUAL; //REMEMBER REMEMBER
//pwm_params.useLEDStrip = feature(FEATURE_LED_STRIP);
//pwm_params.usePPM = feature(FEATURE_RX_PPM);
//pwm_params.useSerialRx = feature(FEATURE_RX_SERIAL);
//#ifdef SONAR
//pwm_params.useSonar = feature(FEATURE_SONAR);
//#endif
//
//#ifdef USE_SERVOS
//pwm_params.useServos = false;
//pwm_params.useChannelForwarding = feature(FEATURE_CHANNEL_FORWARDING);
//pwm_params.servoCenterPulse = masterConfig.escAndServoConfig.servoCenterPulse;
//pwm_params.servoPwmRate = masterConfig.servo_pwm_rate;
//#endif
//
//pwm_params.useOneshot = feature(FEATURE_ONESHOT125);
//pwm_params.motorPwmRate = masterConfig.motor_pwm_rate;
//pwm_params.idlePulse = masterConfig.escAndServoConfig.mincommand;
//if (feature(FEATURE_3D))
//    pwm_params.idlePulse = masterConfig.flight3DConfig.neutral3d;
//if (pwm_params.motorPwmRate > 500)
//    pwm_params.idlePulse = 0; // brushed motors
//
//pwmRxInit(masterConfig.inputFilteringMode);
//
//pwmOutputConfiguration_t *pwmOutputConfiguration = pwmInit(&pwm_params);
//
//mixerUsePWMOutputConfiguration(pwmOutputConfiguration);

if (!feature(FEATURE_ONESHOT125))
    motorControlEnable = true;

systemState |= SYSTEM_STATE_MOTORS_READY;

#ifdef BEEPER
beeperConfig_t beeperConfig = { .gpioPeripheral = BEEP_PERIPHERAL, .gpioPin =
        BEEP_PIN, .gpioPort = BEEP_GPIO,
#ifdef BEEPER_INVERTED
        .gpioMode = Mode_Out_PP,
        .isInverted = true
#else
        .gpioMode = Mode_Out_OD, .isInverted = false
#endif
        };
#ifdef NAZE
if (hardwareRevision >= NAZE32_REV5) {
    // naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
    beeperConfig.gpioMode = Mode_Out_PP;
    beeperConfig.isInverted = true;
}
#endif

beeperInit(&beeperConfig);
#endif

#ifdef INVERTER
initInverter();
#endif

#ifdef USE_SPI
spiInit (SPI1);
spiInit (SPI2);
#endif

#ifdef USE_HARDWARE_REVISION_DETECTION
updateHardwareRevision();
#endif

#if defined(NAZE)
if (hardwareRevision == NAZE32_SP) {
    serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
} else {
    serialRemovePort(SERIAL_PORT_USART3);
}
#endif

#if defined(SPRACINGF3) && defined(SONAR) && defined(USE_SOFTSERIAL2)
if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
    serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
}
#endif

#ifdef USE_I2C
#if defined(NAZE)
if (hardwareRevision != NAZE32_SP) {
    i2cInit(I2C_DEVICE);
} else {
    if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) {
        i2cInit(I2C_DEVICE);
    }
}
#elif defined(CC3D)
if (!doesConfigurationUsePort(SERIAL_PORT_USART3)) {
    i2cInit(I2C_DEVICE);
}
#else
i2cInit(I2C_DEVICE);
#endif
#endif

#ifdef USE_ADC
drv_adc_config_t adc_params;

adc_params.enableVBat = feature(FEATURE_VBAT);
adc_params.enableRSSI = feature(FEATURE_RSSI_ADC);
adc_params.enableCurrentMeter = feature(FEATURE_CURRENT_METER);
adc_params.enableExternal1 = false;
#ifdef OLIMEXINO
adc_params.enableExternal1 = true;
#endif
#ifdef NAZE
// optional ADC5 input on rev.5 hardware
adc_params.enableExternal1 = (hardwareRevision >= NAZE32_REV5);
#endif

adcInit(&adc_params);
#endif

initBoardAlignment(&masterConfig.boardAlignment);

#ifdef DISPLAY
if (feature(FEATURE_DISPLAY)) {
    displayInit(&masterConfig.rxConfig);
}
#endif

//#ifdef TEST_ENABLE
failureFlag = 0;
if (!sensorsAutodetectmpu(&masterConfig.sensorAlignmentConfig,
        masterConfig.gyro_lpf, masterConfig.acc_hardware,
        masterConfig.mag_hardware)) {
    // if gyro was not detected due to whatever reason, we give up now.
    //LEDx_ON;   //PA4
    failureFlag |= (1 << FAILURE_MISSING_ACC);
}
#ifdef BARO
if (!sensorsAutodetectbaro(masterConfig.baro_hardware)) {
    // if baro was not detected due to whatever reason, we give up now.
    //LEDy_ON;   //PA5

    // LED1_TOGGLE;
    // failureMode(2);

    failureFlag |= (1 << FAILURE_BARO);
}
#endif
if (clockcheck == 1) {
    //failure if running on internal clock
    //LEDz_ON;   //PA6
    failureFlag |= (1 << FAILURE_EXTCLCK);
}

//failureFlag = 0;
//
//failureFlag |=  (1 << FAILURE_MISSING_ACC);
//failureFlag |= (1 << FAILURE_EXTCLCK);

if (failureFlag != 0) {
    failureMode(failureFlag);
}
//#endif
systemState |= SYSTEM_STATE_SENSORS_READY;

// LED sequence for start
LED_R_ON;
LED_M_OFF;
for (i = 0; i < 15; i++) {
    LED_R_TOGGLE;
    LED_M_TOGGLE;
    LED_L_TOGGLE;
    delay(25);
    BEEP_ON;
    delay(25);
    BEEP_OFF;
}
LED_M_OFF;
LED_R_OFF;
LED_L_OFF;



#ifdef MAG
if (sensors(SENSOR_MAG))
    compassInit();
#endif

imuInit();

mspInit(&masterConfig.serialConfig);



#ifdef USE_CLI
cliInit(&masterConfig.serialConfig);
#endif

failsafeInit(&masterConfig.rxConfig,
        masterConfig.flight3DConfig.deadband3d_throttle);

rxInit(&masterConfig.rxConfig, currentProfile->modeActivationConditions);



#ifdef GPS
if (feature(FEATURE_GPS)) {
    gpsInit(&masterConfig.serialConfig, &masterConfig.gpsConfig);
    navigationInit(&currentProfile->gpsProfile, &currentProfile->pidProfile);
}
#endif

#ifdef SONAR
if (feature(FEATURE_SONAR)) {
    sonarInit (sonarHardware);
}
#endif

#ifdef LED_STRIP
ledStripInit(masterConfig.ledConfigs, masterConfig.colors);

if (feature(FEATURE_LED_STRIP)) {
    ledStripEnable();
}
#endif

#ifdef TELEMETRY
if (feature(FEATURE_TELEMETRY)) {
    telemetryInit();
}
#endif

#ifdef USE_FLASHFS
#ifdef NAZE
if (hardwareRevision == NAZE32_REV5) {
    m25p16_init();
}
#elif defined(USE_FLASH_M25P16)
m25p16_init();
#endif

flashfsInit();
#endif

#ifdef BLACKBOX
initBlackbox();
#endif

previousTime = micros();

if (masterConfig.mixerMode == MIXER_GIMBAL){

    accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);

}



updateGains();
gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
#ifdef BARO
baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
baroCalibrate();
baroInit();
#endif



// start all timers
// TODO - not implemented yet
timerStart();

ENABLE_STATE(SMALL_ANGLE);
DISABLE_ARMING_FLAG(PREVENT_ARMING);

#ifdef SOFTSERIAL_LOOPBACK
// FIXME this is a hack, perhaps add a FUNCTION_LOOPBACK to support it properly
loopbackPort = (serialPort_t*)&(softSerialPorts[0]);

if (!loopbackPort->vTable) {
    loopbackPort = openSoftSerial(0, NULL, 19200, SERIAL_NOT_INVERTED);
}
serialPrint(loopbackPort, "LOOPBACK\r\n");
#endif

// Now that everything has powered up the voltage and cell count be determined.

if (feature(FEATURE_VBAT | FEATURE_CURRENT_METER))
    batteryInit(&masterConfig.batteryConfig);

#ifdef DISPLAY
if (feature(FEATURE_DISPLAY)) {
#ifdef USE_OLED_GPS_DEBUG_PAGE_ONLY
    displayShowFixedPage(PAGE_GPS);
#else
    displayResetPageCycling();
    displayEnablePageCycling();
#endif
}
#endif

#ifdef CJMCU

#endif

// Latch active features AGAIN since some may be modified by init().
latchActiveFeatures();
motorControlEnable = true;

systemState |= SYSTEM_STATE_READY;

#ifdef LASER_TOF

ranging_init();

#endif


//spi.Init();
//spi.Settings(MODE0, 562, LSBFIRST);

#ifdef OPTIC_FLOW

initOpticFlow();

#endif

updatePosGains();


resetUser();



plutoInit();


timerDataConfiguration();


delay(300);



timerInit();  // timer must be initialized before any channel is allocated




// when using airplane/wing mixer, servo/motor outputs are remapped
if (masterConfig.mixerMode == MIXER_AIRPLANE
        || masterConfig.mixerMode == MIXER_FLYING_WING
        || masterConfig.mixerMode == MIXER_CUSTOM_AIRPLANE)
    pwm_params.airplane = true;
else
    pwm_params.airplane = false;
#if defined(USE_USART2) && defined(STM32F10X)
pwm_params.useUART2 = doesConfigurationUsePort(SERIAL_PORT_USART2);
#endif
#ifdef STM32F303xC
pwm_params.useUART3 = doesConfigurationUsePort(SERIAL_PORT_USART3);
#endif
pwm_params.useVbat = feature(FEATURE_VBAT);
pwm_params.useSoftSerial = feature(FEATURE_SOFTSERIAL);
pwm_params.useParallelPWM = feature(FEATURE_RX_PARALLEL_PWM);
pwm_params.useRSSIADC = feature(FEATURE_RSSI_ADC);
pwm_params.useCurrentMeterADC = feature(FEATURE_CURRENT_METER)
        && masterConfig.batteryConfig.currentMeterType
                == CURRENT_SENSOR_VIRTUAL; //REMEMBER REMEMBER
pwm_params.useLEDStrip = feature(FEATURE_LED_STRIP);
pwm_params.usePPM = feature(FEATURE_RX_PPM);
pwm_params.useSerialRx = feature(FEATURE_RX_SERIAL);
#ifdef SONAR
pwm_params.useSonar = feature(FEATURE_SONAR);
#endif

#ifdef USE_SERVOS
pwm_params.useServos = false;
pwm_params.useChannelForwarding = feature(FEATURE_CHANNEL_FORWARDING);
pwm_params.servoCenterPulse = masterConfig.escAndServoConfig.servoCenterPulse;
pwm_params.servoPwmRate = masterConfig.servo_pwm_rate;
#endif

pwm_params.useOneshot = feature(FEATURE_ONESHOT125);
pwm_params.motorPwmRate = masterConfig.motor_pwm_rate;
pwm_params.idlePulse = masterConfig.escAndServoConfig.mincommand;
if (feature(FEATURE_3D))
    pwm_params.idlePulse = masterConfig.flight3DConfig.neutral3d;
if (pwm_params.motorPwmRate > 500)
    pwm_params.idlePulse = 0; // brushed motors

pwmRxInit(masterConfig.inputFilteringMode);



pwmOutputConfiguration_t *pwmOutputConfiguration = pwmInit(&pwm_params);




mixerUsePWMOutputConfiguration(pwmOutputConfiguration);





#if defined(PRIMUSX) || defined(PRIMUSX2)
unibusAdcInit();
reverseMotorGPIOInit();
xRangingInit();
if(localisationType==UWB){
UART.init(UART2, BAUD_RATE_115200);
GPIO.init(Pin8,INPUT_PD);
}
//if(useRangingSensor)
//    ranging_init();

#endif




}

#ifdef SOFTSERIAL_LOOPBACK
void processLoopback(void) {
if (loopbackPort) {
    uint8_t bytesWaiting;
    while ((bytesWaiting = serialRxBytesWaiting(loopbackPort))) {
        uint8_t b = serialRead(loopbackPort);
        serialWrite(loopbackPort, b);
    };
}
}
#else
#define processLoopback()
#endif

int main(void)
{

init();

//accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
//GPIO.init(Pin3, INPUT_PD);

//GPIO.write(Pin3, STATE_LOW);

//PWM.init(Pin13, 50);


while (1) {

    loop();




    processLoopback();

}


}

void HardFault_Handler(void)
{
// fall out of the sky
uint8_t requiredState = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
if ((systemState & requiredState) == requiredState) {
    stopMotors();
}
while (1);
}
