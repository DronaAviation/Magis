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
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/ranging_vl53l0x.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/escservo.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
//#include "config/config.h"
#include "config/runtime_config.h"

#include "command/command.h"
#include "flight/acrobats.h"
#include "altitudehold.h"

// Distance below max_altitude where limitAltitude ( ) stops accepting climb, cm.
// It has no velocity term, so this is the stopping distance: keep it at least
// the overshoot at the fastest climb flown, or the ceiling gets breached.
#define ALT_CEILING_MARGIN_CM 25

int16_t max_altitude = -1;
int16_t althold_throttle = 0;
int32_t errorVelocityI = 0;
int32_t altHoldThrottleAdjustment = 0;
int32_t AltHold;
int32_t vario = 0;                      // variometer in cm/s
int32_t setVelocity = 0;                // rate requested by the stick, or the landing descent, cm/s ( getSetVelocity ( ) )
int32_t calculatedError = 10;
int32_t VelocityZ;
int32_t baroAlt_offset_print = 0;
int32_t PositionZ;
uint32_t baro_last_update;
bool AltRstRequired = 1;

static barometerConfig_t *barometerConfig;
static pidProfile_t *pidProfile;
static rcControlsConfig_t *rcControlsConfig;
static escAndServoConfig_t *escAndServoConfig;
barometerConfig_t *barometerConfig_tmp;

int QUEUE_MAX_LENGTH = 15;

static float buff[15];
static int16_t head = 0;
static int16_t rear = -1;
static int16_t itemCount = 0;
#ifdef LASER_ALT
float _time_constant_z=1.5f;
#else
float _time_constant_z = 2.0f;
#endif
float accZ_tmp;
static float accZ_old = 0.0f;

int16_t first_reads = 0;
int16_t first_velocity_reads = 0;
int16_t ctr = 0;
static int32_t last_hist_position = 0;

void setAltitude(float new_altitude);

float _k1_z;                      // gain for vertical position correction
float _k2_z;                      // gain for vertical velocity correction
float _k3_z;                // gain for vertical accelerometer offset correction

// general variables
float _position_base_z; // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float _position_correction_z; // sum of corrections to _position_base from delayed 1st order samples in cm
float _accel_correction_hbf_z;
float _velocity_z; // latest velocity estimate (integrated from accelerometer values) in cm/s
float _position_error_z; // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
float _position_z; // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float accel_ef_z;

//temp vr

float _time_constant_z1 = 2.0f;

float _k1_z1;                      // gain for vertical position correction
float _k2_z1;                      // gain for vertical velocity correction
float _k3_z1;                // gain for vertical accelerometer offset correction

int32_t VelocityZ1 = 0;
int32_t EstAlt1 = 0;

uint32_t baro_last_update1;

int16_t first_reads1 = 0;
static int32_t last_hist_position1 = 0;

float _position_base_z1; // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float _position_correction_z1; // sum of corrections to _position_base from delayed 1st order samples in cm
float _accel_correction_hbf_z1;
float _velocity_z1; // latest velocity estimate (integrated from accelerometer values) in cm/s
float _position_error_z1; // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
float _position_z1; // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)

float ToF_Height = 0.0f;
float Baro_Height = 0.0f;
float fused = 0.0f;
float filtered = 0.0f;

int32_t altholdDebug = 0;
int32_t altholdDebug1 = 0;
int32_t altholdDebug2 = 0;
int32_t altholdDebug3 = 0;
int32_t altholdDebug4 = 0;
int32_t altholdDebug5 = 0;
int32_t altholdDebug6 = 0;
int32_t altholdDebug7 = 0;
int32_t altholdDebug8 = 0;
int32_t altholdDebug9 = 0;
int32_t velControlDebug[3] = { 0 };

void configureAltitudeHold(pidProfile_t *initialPidProfile, barometerConfig_t *intialBarometerConfig, rcControlsConfig_t *initialRcControlsConfig, escAndServoConfig_t *initialEscAndServoConfig)
{
    pidProfile = initialPidProfile;
    barometerConfig = intialBarometerConfig;
    rcControlsConfig = initialRcControlsConfig;
    barometerConfig_tmp = barometerConfig;
    escAndServoConfig = initialEscAndServoConfig;
}

#if defined(BARO) || defined(SONAR) || defined(LASER_ALT)

int16_t initialThrottleHold_test;
int16_t debug_e1;

static int16_t initialThrottleHold;
static int32_t EstAlt = 0;                // in cm

// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)
#define UPDATE_FREQUENCY (1000 * 10) //100Hz

#define DEGREES_80_IN_DECIDEGREES 800

/*
 * Hover-trim offload. With initialThrottleHold pinned at 1500, the whole hover
 * trim lives in errorVelocityI, clamped at +/- 300 counts, so hover throttle
 * could not go above 1800 and the craft sank as the battery sagged.
 *
 * While settled ( armed, stick centred, setpoint fixed, |VelocityZ| small ) the
 * trim moves into initialThrottleHold one count at a time. Commanded throttle
 * is unchanged: the baseline gains what the integrator loses, and
 * altHoldThrottleAdjustment is corrected here too, since it only updates at
 * 100 Hz. Never offload mid-climb - a transient would stick in the baseline.
 */
#define ALT_TRIM_OFFLOAD_MS  20         // ms, at most one count this often
#define ALT_TRIM_OFFLOAD_VEL 10         // cm/s below which the craft counts as settled

/*
 * Setpoint shaping ( ArduPilot / DJI style ). The throttle stick never takes the
 * position loop out of the chain: it sets a climb rate that moves AltHold, and
 * the position loop keeps tracking the moving target with that rate fed
 * forward. The rate is ramped, so climbs and descents start and stop without a
 * step, and centring the stick lets the target coast to a stop instead of
 * snapping it to wherever EstAlt happens to be ( which overshot ).
 *
 * A new AltHold written from outside ( take-off, setAltitude ( ),
 * setRelativeAltitude ( ), MSP ) is not jumped to. It becomes a goal, and the
 * target travels there on a profile: ramp up, cruise at the command rate, and
 * brake at ALT_GOAL_DECEL_CMSS just in time to stop on it. Stepping AltHold
 * instead left the P = 1 position loop to close the gap, and its demand shrinks
 * with the gap, so the last 40 cm of a 120 cm take-off crawled. Moving the
 * stick cancels the goal.
 *
 * setVelocity holds the request ( stick rate, landing descent, 0 when centred ),
 * as getSetVelocity ( ) returns it. The final velocity setpoint is clamped and
 * slewed as well; it lives in altVelTarget and the controller's setVel.
 */
#define ALT_MAX_CLIMB_CMS       40      // cm/s, full stick up
#define ALT_MAX_DESCENT_CMS     30      // cm/s, full stick down
#define ALT_CMD_MAX_CLIMB_CMS   60      // cm/s, cruise up to a commanded goal ( take-off )
#define ALT_CMD_MAX_DESCENT_CMS 30      // cm/s, cruise down to a commanded goal
#define ALT_GOAL_DECEL_CMSS     80      // cm/s^2, braking into a goal ( below the ramp, so the ramp can follow it )
#define ALT_STICK_FULL_TRAVEL   500     // RC counts from centre ( 1500 ) to end
#define ALT_STICK_ACCEL_CMSS    100     // cm/s^2, ramp of the profile rate
#define ALT_VEL_ACCEL_CMSS      150     // cm/s^2, slew of the final velocity setpoint
#define ALT_TARGET_LEASH_CM     50      // cm, target stops advancing this far from EstAlt

/*
 * Landing keeps its own descent profile, not the stick limits. annexCode ( )
 * ( mw.cpp ) writes landThrottle into rcData[THROTTLE] while isLanding, flown as
 * ( landThrottle - 1500 ) / 4: -75 cm/s at the default 1200, and -48 to -112 cm/s
 * ( clamped here to -100 ) when user code picks a speed through Command.land ( )
 * ( API/User.cpp: landThrottle = 1305 - landSpeed ). Near the floor the baro drifts
 * low in the craft's own downwash; at the stick's 10-20 cm/s that drift alone
 * met the descent demand, so the craft hovered a few cm up with EstAlt still
 * falling and the touchdown test ( descent stopped ) never fired.
 */
#define ALT_LAND_MAX_DESCENT_CMS 100    // cm/s, descent clamp while isLanding

static float altRate = 0.0f;            // ramped profile rate moving the target, cm/s
static float altVelTarget = 0.0f;       // slewed velocity setpoint, cm/s
static float altTarget = 0.0f;          // AltHold with the fraction kept, cm
static float altGoal = 0.0f;            // commanded altitude the target is travelling to, cm
static bool altGoalActive = false;
static bool altHoldGroundIdle = false;  // armed on the throttle stick, motors held at idle

// Drop all setpoint shaping state and pin the target to the current estimate.
static void resetAltSetpoint(void)
{
    altRate = 0.0f;
    altVelTarget = 0.0f;
    altGoalActive = false;
    AltHold = EstAlt;
    altTarget = (float) EstAlt;
}

/*
 * Flip ( acrobats.cpp ) keeps ALT_HOLD on: its DEACTIVATE_RC_MODE(BOXBARO) is
 * undone by updateActivatedModes ( ) on the next RX frame while the app holds the
 * BARO AUX channel, before updateAltHoldState ( ) reads it. It drives
 * rcData[THROTTLE] itself ( 2000 in ASCEND and HOLD ) and ASCEND waits for
 * VelocityZ >= 100 cm/s before it starts the rotation. Through setpoint shaping
 * full stick is 40 cm/s, so ASCEND timed out after 2.2 s and never flipped
 * ( MagisV2 flip-althold-regression ). While a flip runs, the controller flies
 * the raw rate path it had before setpoint shaping instead.
 */
#define ALT_FLIP_MAX_CLIMB_CMS   120    // cm/s, ( 2000 - 1500 ) / 4 = 125, clamped as before shaping
#define ALT_FLIP_MAX_DESCENT_CMS 100    // cm/s
#define ALT_FLIP_EXIT_IGNORE_MS  100    // ms, throttle ignored after a flip ( RX refresh <= 20 ms )
#define ALT_FLIP_RETURN_MIN_CM   20     // cm, pre-flip AltHold below this is treated as on the ground: no return
#define ALT_FLIP_EXIT_I_HOLD_MS  500    // ms, velocity integrator held after a flip while its climb is braked

static bool flipActive(void)
{
#ifdef ENABLE_ACROBAT
    return flipState != 0;
#else
    return false;
#endif
}

// Pre-shaping controller: outside the deadband the rate is fed straight to the
// velocity loop with no cap, ramp or slew; inside it the position loop holds
// AltHold. The shaping state follows; the hand-back on exit is in
// calculateAltHoldThrottleAdjustment ( ). Returns cm/s.
static int32_t flipVelocitySetpoint(void)
{
    int32_t setVel;    // cm/s

    if (setVelocity != 0) {
        AltHold = EstAlt;
        setVel = setVelocity;
    } else {
        const int32_t error = constrain(AltHold - EstAlt, -500, 500);    // cm
        calculatedError = error;
        altholdDebug8 = error;
        setVel = constrain((pidProfile->P8[PIDALT] * error / 128), -300, +300);
    }

    altTarget = (float) AltHold;
    altRate = 0.0f;
    altGoalActive = false;
    altVelTarget = (float) setVel;
    return setVel;
}

static void offloadHoverTrim(void)
{
    static uint32_t lastOffload = 0;    // ms

    if (!ARMING_FLAG(ARMED) || altHoldGroundIdle || setVelocity != 0 || altRate != 0.0f || altGoalActive)
        return;

    if (ABS(VelocityZ) > ALT_TRIM_OFFLOAD_VEL)
        return;

    uint32_t now = millis();
    if ((now - lastOffload) < ALT_TRIM_OFFLOAD_MS)
        return;

    lastOffload = now;

    int32_t trim = errorVelocityI / 8192;   // throttle counts
    if (trim == 0)
        return;

    int32_t step = (trim > 0) ? 1 : -1;
    int32_t baseline = (int32_t) initialThrottleHold + step;

    // Never walk the baseline outside what the ESCs can be commanded anyway.
    if (baseline < escAndServoConfig->minthrottle || baseline > escAndServoConfig->maxthrottle)
        return;

    initialThrottleHold = (int16_t) baseline;
    errorVelocityI -= step * 8192;
    altHoldThrottleAdjustment -= step;
}

/** Drone should maintian its altitude which it attins at the throttle of '1500'*/
static void applyMultirotorAltHold(void)
{
    static uint8_t isAltHoldChanged = 0;
    static int16_t throttle_history = 0;
    static int16_t sensitivity_inv = 6;
    // multirotor alt hold
    if (rcControlsConfig->alt_hold_fast_change) {
        // rapid alt changes
        if (ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband) {

            isAltHoldChanged = 1;
            //rcCommand[THROTTLE] += (rcData[THROTTLE] > initialThrottleHold) ? -rcControlsConfig->alt_hold_deadband : rcControlsConfig->alt_hold_deadband; //drona
            rcCommand[THROTTLE] = throttle_history + constrain((rcData[THROTTLE] - initialThrottleHold) / sensitivity_inv, -50, 80); //Magis
//            if (rcData[THROTTLE] < 1100){
//                rcCommand[THROTTLE] = 1150;
//            }

        } else {
            if (isAltHoldChanged) {
                // Not just AltHold: the target and rate must go too, or the recentred
                // height reads as an external write and is flown back to as a goal from
                // wherever the target was left ( the craft sank about a third of the
                // climb ). Fast mode is off by default ( alt_hold_fast_change 0 ).
                resetAltSetpoint();
//                AltHold = 120;
                isAltHoldChanged = 0;
                // Do NOT reset errorVelocityI here: it holds the hover trim, so
                // clearing it drops the throttle and the craft falls. Reset only on
                // BARO entry / disarm.
                if (ARMING_FLAG(ARMED)) {
                    //altHoldThrottleAdjustment = throttle_history;
                }
            }

            rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
            throttle_history = rcCommand[THROTTLE]; //Magis

        }
    } else {
        // slow alt changes: the stick sets the climb rate that moves AltHold ( see
        // setpoint shaping ). The rate starts at zero on the deadband edge, so
        // leaving the deadband is not a step, and reaches the maximum at full stick.
        const int32_t deflection = rcData[THROTTLE] - 1500;    // RC counts from centre
        const int32_t deadband = rcControlsConfig->alt_hold_deadband;

        // flip ( ) writes 2000 into rcData[THROTTLE] on its last tick as well, and
        // it stays there until the next RX frame. Read as stick it would cancel the
        // return-to-height goal set on flip exit, so ignore the throttle briefly.
        static bool flipSeen = false;
        static uint32_t flipEndedAtMs = 0;    // ms, millis ( ) at the flip's end, 0 = none yet
        if (flipActive()) {
            flipSeen = true;
        } else if (flipSeen) {
            flipSeen = false;
            flipEndedAtMs = millis();
        }
        const bool flipExitGrace = (flipEndedAtMs != 0) && ((millis() - flipEndedAtMs) < ALT_FLIP_EXIT_IGNORE_MS);

        if (isLanding) {
            // rcData[THROTTLE] holds landThrottle here ( annexCode ( ) in mw.cpp ):
            // the landing descent never goes through the stick limits.
            setVelocity = constrain(deflection / 4, -ALT_LAND_MAX_DESCENT_CMS, 0);
        } else if (flipExitGrace) {
            setVelocity = 0;
        } else if (flipActive()) {
            // rcData[THROTTLE] is written by flip ( ), not the pilot: raw rate as
            // before setpoint shaping ( see flipVelocitySetpoint ( ) ).
            setVelocity = (ABS(deflection) > deadband) ? constrain(deflection / 4, -ALT_FLIP_MAX_DESCENT_CMS, ALT_FLIP_MAX_CLIMB_CMS) : 0;
        } else if (ABS(deflection) > deadband) {
            const int32_t maxRate = (deflection > 0) ? ALT_MAX_CLIMB_CMS : ALT_MAX_DESCENT_CMS;
            const int32_t rate = constrain((ABS(deflection) - deadband) * maxRate / (ALT_STICK_FULL_TRAVEL - deadband), 0, maxRate);
            setVelocity = (deflection > 0) ? rate : -rate;
        } else {
            setVelocity = 0;
            offloadHoverTrim();
        }

        rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);

        altholdDebug1=altHoldThrottleAdjustment;

        altholdDebug2=initialThrottleHold;

//#ifdef LASER_TOF
//        if(rcData[THROTTLE]<1150 && isOutofRange())
//#else
        //    if (rcData[THROTTLE] < 1150)
//#endif
//        if (rcData[THROTTLE] < 1150)
//        {
//            rcCommand[THROTTLE] = 1350;
//        }

//        if(!isTakeOffHeightSet){
//
//        	  rcCommand[THROTTLE] = 1000;
//        }

    }

    // Both modes: armed on the throttle stick with the stick down means the craft is
    // still on its skids, so hold the motors at idle and tell the controller to stay
    // in reset. Outside the if / else as in MagisV2 c569c0f: inside the slow branch
    // this never ran with alt_hold_fast_change = 1, so that mode neither held the
    // controller in reset on the ground nor cleared isThrottleStickArmed, which left
    // the airborne I-term freeze switched off for the whole flight.
    if (isThrottleStickArmed && rcData[THROTTLE] <= 1500) {
        // rcData[THROTTLE] = 1000;
        rcCommand[THROTTLE] = 1000;
        altHoldGroundIdle = true;
    } else {
        isThrottleStickArmed = false;
        altHoldGroundIdle = false;
    }

    althold_throttle = rcCommand[THROTTLE];

    altholdDebug = AltHold;
    altholdDebug3 = altGoalActive ? 1 : 0;    // goal profile running
    altholdDebug4 = rcCommand[THROTTLE];
}

static void applyFixedWingAltHold(airplaneConfig_t *airplaneConfig)
{
    // handle fixedwing-related althold. UNTESTED! and probably wrong
    // most likely need to check changes on pitch channel and 'reset' althold similar to
    // how throttle does it on multirotor

    rcCommand[PITCH] += altHoldThrottleAdjustment * airplaneConfig->fixedwing_althold_dir;
}

void applyAltHold(airplaneConfig_t *airplaneConfig)
{
    if (STATE(FIXED_WING)) {
        applyFixedWingAltHold(airplaneConfig);
    } else {
        applyMultirotorAltHold();
    }
}

void updateAltHoldState(void)
{
    // Baro alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXBARO)) {
        DISABLE_FLIGHT_MODE(BARO_MODE);
        // applyAltHold ( ) no longer runs, so nothing else clears the stick inputs.
        setVelocity = 0;
        altHoldGroundIdle = false;
        return;
    }

    if (!FLIGHT_MODE(BARO_MODE)) {

        ENABLE_FLIGHT_MODE(BARO_MODE);
        resetAltSetpoint();
        //initialThrottleHold = rcData[THROTTLE];
        initialThrottleHold = 1500;
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
    initialThrottleHold_test = initialThrottleHold;

    //debug_d0 = pidProfile->D8[PIDALT];
    debug_e1 = rcCommand[THROTTLE];
}

void updateSonarAltHoldState(void)
{
    // Sonar alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXSONAR)) {
        DISABLE_FLIGHT_MODE(SONAR_MODE);
        return;
    }

    if (!FLIGHT_MODE(SONAR_MODE)) {
        ENABLE_FLIGHT_MODE(SONAR_MODE);
        resetAltSetpoint();
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

bool isThrustFacingDownwards(rollAndPitchInclination_t *inclination)
{
    return ABS(inclination->values.rollDeciDegrees) < DEGREES_80_IN_DECIDEGREES && ABS(inclination->values.pitchDeciDegrees) < DEGREES_80_IN_DECIDEGREES;
}

/*
 * This (poorly named) function merely returns whichever is higher, roll inclination or pitch inclination.
 * //TODO: Fix this up. We could either actually return the angle between 'down' and the normal of the craft
 * (my best interpretation of scalar 'tiltAngle') or rename the function.
 */
int16_t calculateTiltAngle(rollAndPitchInclination_t *inclination)
{
    return MAX(ABS(inclination->values.rollDeciDegrees), ABS(inclination->values.pitchDeciDegrees));
}

/*
 * Altitude P-controller with setpoint shaping ( see the shaping block above ).
 * Moves the target, then returns the velocity setpoint for the velocity loop:
 * profile rate fed forward plus P_ALT x the position error, clamped per source
 * and slewed. ctrlDt is the control-loop period, s. Returns cm/s.
 */
static int32_t shapedVelocitySetpoint(float ctrlDt)
{
    int32_t error;

    // AltHold written elsewhere ( take-off, setAltitude ( ), setRelativeAltitude ( ),
    // MSP ) becomes the goal; the target stays where it is and travels there below.
    if (AltHold != lrintf(altTarget)) {
        altGoal = (float) AltHold;
        altGoalActive = true;
    }
    if (setVelocity != 0) {
        altGoalActive = false;    // the stick ( or landing ) takes over
    }

    // Rate the target should move at: the stick's, or the goal profile's - the
    // fastest rate that can still brake to a stop on the goal, capped at cruise.
    float profileRate = (float) setVelocity;    // cm/s
    if (altGoalActive) {
        const float remaining = altGoal - altTarget;    // cm
        const float stopRate = sqrtf(2.0f * ALT_GOAL_DECEL_CMSS * fabsf(remaining));
        if (remaining > 0.0f) {
            profileRate = fminf(stopRate, (float) ALT_CMD_MAX_CLIMB_CMS);
        } else {
            profileRate = -fminf(stopRate, (float) ALT_CMD_MAX_DESCENT_CMS);
        }
    }

    const float rateStep = ALT_STICK_ACCEL_CMSS * ctrlDt;    // cm/s per tick
    altRate += constrainf(profileRate - altRate, -rateStep, rateStep);

    // Move the target. The leash stops it running away from a craft that cannot
    // keep up ( still on the ground, weak battery ); the rate is kept, so it resumes
    // as soon as the craft catches up.
    const float nextTarget = altTarget + altRate * ctrlDt;
    if (!((altRate > 0.0f && nextTarget > (float) (EstAlt + ALT_TARGET_LEASH_CM)) || (altRate < 0.0f && nextTarget < (float) (EstAlt - ALT_TARGET_LEASH_CM)))) {
        altTarget = nextTarget;
    }

    // Arrived: land exactly on the goal once the profile has braked.
    if (altGoalActive && fabsf(altGoal - altTarget) < 1.0f && fabsf(altRate) <= 2.0f * rateStep) {
        altTarget = altGoal;
        altRate = 0.0f;
        altGoalActive = false;
    }
    AltHold = lrintf(altTarget);

    error = constrain(AltHold - EstAlt, -500, 500);    // cm
    // No deadband on the position error: applyDeadband ( ) subtracts, so a 5 cm
    // band took 5 cm off every error ( a 10 cm sag read as 5 ) and caused a limit
    // cycle. EstAlt is already smooth ( ~1 cm between samples ), so none is needed.
    calculatedError = error;
    altholdDebug8 = error;

    // Profile rate fed forward plus the position loop, clamped and slewed.
    const float maxClimb = altGoalActive ? (float) ALT_CMD_MAX_CLIMB_CMS : (float) ALT_MAX_CLIMB_CMS;
    const float maxDescent = isLanding ? (float) ALT_LAND_MAX_DESCENT_CMS : (altGoalActive ? (float) ALT_CMD_MAX_DESCENT_CMS : (float) ALT_MAX_DESCENT_CMS);
    float velDemand = altRate + (float) pidProfile->P8[PIDALT] * (float) error / 128.0f;
    velDemand = constrainf(velDemand, -maxDescent, maxClimb);
    const float velStep = ALT_VEL_ACCEL_CMSS * ctrlDt;    // cm/s per tick
    altVelTarget += constrainf(velDemand - altVelTarget, -velStep, velStep);

    return lrintf(altVelTarget);
}

int32_t calculateAltHoldThrottleAdjustment(int32_t velocity_z, float accZ_tmp, float accZ_old, float ctrlDt)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel;

    // Flip hand-back ( MagisV2 flip-althold-regression ). HOLD asks for 120 cm/s
    // while the craft falls out of the rotation, so the velocity integrator winds
    // up, and the flip's last setpoint ( 120 ) was left for the slew to walk down:
    // together they drove a flyaway into the ceiling. On exit, restart from a zero
    // setpoint with the hover trim from before the flip, and fly back to the
    // altitude ALT_HOLD was holding when the flip was sent: HOLD's fixed 1.5 s of
    // full throttle ends the flip 40-120 cm high. Writing AltHold makes it a goal,
    // flown on the goal profile like take-off; the stick cancels it. Checked ahead
    // of the tilt test so a flip that ends tilted is still handed back.
    static bool altFlipWasActive = false;
    static int32_t altPreFlipVelocityI = 0;
    static int32_t altPreFlipAltHold = 0;     // cm
    static bool altPreFlipReturn = false;
    static int32_t altGroundAlt = 0;          // cm, EstAlt last seen on the ground ( see the reset below )
    static uint32_t altFlipExitAtMs = 0;      // ms, millis ( ) at flip exit, 0 = no hold
    const bool flipping = flipActive();
    if (flipping && !altFlipWasActive) {
        // Without BARO_MODE the integrator is chasing an AltHold nothing actuates;
        // save what BARO entry would reset it to instead, and hold where the flip
        // ends ( a user-code flip can start with ALT_HOLD off, command.cpp only
        // checks MAG_MODE ).
        altPreFlipVelocityI = FLIGHT_MODE(BARO_MODE) ? errorVelocityI : 0;
        // Only return to a height the craft was flying at: a flip sent while armed on
        // the floor would fly back down to it with the motors running, and the
        // ground-effect baro bias can then hold it just above the surface with no
        // touchdown detection. Measured against altGroundAlt rather than a bare 20 cm:
        // the barometer datum freezes at the arm instant ( sensors/barometer.cpp ), so
        // altGroundAlt is normally about 0 and this matches MagisV2's test, but after a
        // disarm and re-arm in the air the datum is the first arm's and a bare threshold
        // would measure from there. ( altHoldGroundIdle is ANDed in as well, but it is
        // usually already false by the time the 100 Hz tick sees the flip, so the
        // altGroundAlt difference is what actually guards this. )
        // While a goal is being flown AltHold is the moving target, so take the goal
        // itself: otherwise flips sent during the return descent ratchet the height up.
        const int32_t preFlipTarget = altGoalActive ? lrintf(altGoal) : AltHold;    // cm
        altPreFlipReturn = FLIGHT_MODE(BARO_MODE) && !altHoldGroundIdle && (preFlipTarget - altGroundAlt) >= ALT_FLIP_RETURN_MIN_CM;
        altPreFlipAltHold = preFlipTarget;
    } else if (!flipping && altFlipWasActive) {
        resetAltSetpoint();
        errorVelocityI = altPreFlipVelocityI;
        altFlipExitAtMs = millis();
        if (altPreFlipReturn) {
            AltHold = altPreFlipAltHold;
        }
    }
    altFlipWasActive = flipping;

    if (!isThrustFacingDownwards(&inclination)) {
        return result;
    }

    // Disarmed, or armed on the throttle stick with the motors held at idle: the
    // craft is on the ground, so hold the whole controller in reset. Otherwise the
    // integrator winds down against a craft that cannot move and the take-off is
    // late and sluggish.
    if (!ARMING_FLAG(ARMED) || altHoldGroundIdle) {
        resetAltSetpoint();
        errorVelocityI = 0;
        altFlipExitAtMs = 0;    // no post-flip integrator hold after a re-arm
        altGroundAlt = EstAlt;  // cm, the surface the flip return test measures against
        return result;
    }

    ctrlDt = constrainf(ctrlDt, 0.0f, 0.05f);    // s

    setVel = flipActive() ? flipVelocitySetpoint() : shapedVelocitySetpoint(ctrlDt);

  //  altholdDebug1 = setVel;

    // Velocity PID-Controller
    // P
    error = setVel - velocity_z;

    altholdDebug9 = error;

    // result = constrain((pidProfile->P8[PIDVEL] * error / 32), -100, +100);
    result = constrain((pidProfile->P8[PIDVEL] * error / 32), -300, +300);

    velControlDebug[0] = result;

    // The flip ends climbing at ~120 cm/s; braking that from a near-zero setpoint
    // drove the integrator to -20 counts in 0.4 s, and once the climb stopped that
    // trim dropped the craft 15-46 cm below the target. Hold the restored pre-flip
    // trim while the P term ( at its clamp ) brakes the climb.
    const bool holdFlipExitI = (altFlipExitAtMs != 0) && ((millis() - altFlipExitAtMs) < ALT_FLIP_EXIT_I_HOLD_MS);

    // I
    if (ARMING_FLAG(ARMED)) {
        if (!holdFlipExitI) {
            errorVelocityI += (pidProfile->I8[PIDVEL] * error);
        }
    } else {
        errorVelocityI = 0;
    }

    //  errorVelocityI = constrain(errorVelocityI, -(8192 * 150), (8192 * 150));
    errorVelocityI = constrain(errorVelocityI, -(8192 * 300), (8192 * 300));

    result += errorVelocityI / 8192;     // I in range +/-200

    velControlDebug[1] = errorVelocityI / 8192;

    // D
    result -= constrain(pidProfile->D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

    velControlDebug[2] = constrain(pidProfile->D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

    return result;
}

int16_t accalttemp;
float Temp;

void calculateEstimatedAltitude(uint32_t currentTime)
{
    static uint32_t previousTime;
    uint32_t dTime;
    int32_t baroVel;
    float dt;
    float vel_acc;
    int32_t vel_tmp;
    float accZ_tmp;
    int32_t sonarAlt = -1;
    static float accZ_old = 0.0f;
    static float vel = 0.0f;
    static float accAlt = 0.0f;
    static int32_t lastBaroAlt;

    static int32_t baroAlt_offset = 0;
    float sonarTransition;

#ifdef SONAR
    int16_t tiltAngle;
#endif

    dTime = currentTime - previousTime;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ)
        return;

    previousTime = currentTime;

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        performBaroCalibrationCycle();
        vel = 0;
        accAlt = 0;
    }

    //BaroAlt = baroCalculateAltitude();
#else
    BaroAlt = 0;
#endif

#ifdef SONAR
    tiltAngle = calculateTiltAngle(&inclination);
    sonarAlt = sonarRead();
    sonarAlt = sonarCalculateAltitude(sonarAlt, tiltAngle);
#endif

    if (sonarAlt > 0 && sonarAlt < 200) {
        baroAlt_offset = BaroAlt - sonarAlt;
        BaroAlt = sonarAlt;
    } else {
        BaroAlt -= baroAlt_offset;
        if (sonarAlt > 0 && sonarAlt <= 300) {
            sonarTransition = (300 - sonarAlt) / 100.0f;
            BaroAlt = sonarAlt * sonarTransition + BaroAlt * (1.0f - sonarTransition);
        }
    }

    dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec
    if (accSumCount) {
        accZ_tmp = (float) accSum[2] / (float) accSumCount;
    } else {
        accZ_tmp = 0;
    }
    vel_acc = accZ_tmp * accVelScale * (float) accTimeSum;

    // Integrator - Altitude in cm
    accAlt += (vel_acc * 0.5f) * dt + vel * dt;
    accalttemp = lrintf(100 * accAlt); //Checking how acc measures height                                                                // integrate velocity to get distance (x= a/2 * t^2)
    accAlt = accAlt * barometerConfig->baro_cf_alt + (float) BaroAlt * (1.0f - barometerConfig->baro_cf_alt); // complementary filter for altitude estimation (baro & acc)
    vel += vel_acc;

#ifdef DEBUG_ALT_HOLD
    debug[1] = accSum[2] / accSumCount; // acceleration
    debug[2] = vel;// velocity
    debug[3] = accAlt;// height
#endif

    imuResetAccelerationSum(1);

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        return;
    }
#endif

    if (sonarAlt > 0 && sonarAlt < 200) {
        // the sonar has the best range
        EstAlt = BaroAlt;
    } else {
        EstAlt = accAlt;
    }

    baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = BaroAlt;

    baroVel = constrain(baroVel, -1500, 1500); // constrain baro velocity +/- 1500cm/s
    baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * barometerConfig->baro_cf_vel + baroVel * (1.0f - barometerConfig->baro_cf_vel);
    vel_tmp = lrintf(vel);

    // set vario
    vario = applyDeadband(vel_tmp, 5);
    if (1) //!(ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband))
    {
        altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(vel_tmp, accZ_tmp, accZ_old, (float) dTime * 1e-6f);
    }        //dronadrona_1200am
    Temp = pidProfile->I8[PIDALT];
    barometerConfig->baro_cf_alt = 1 - Temp / 1000;
    accZ_old = accZ_tmp;
}

/* queue implementation */

void addHistPositionBaseEstZ(float position)
{

    if (itemCount < QUEUE_MAX_LENGTH) {

        rear++;

        if (rear >= QUEUE_MAX_LENGTH) {

            rear = 0;

        }

        buff[rear] = position;

        itemCount++;

    }

    else {

        if (++rear == QUEUE_MAX_LENGTH) {

            rear = 0;

            buff[rear] = position;

            head++;

        }

        else {
            buff[rear] = position;

            head++;

            if (head == QUEUE_MAX_LENGTH) {
                head = 0;
            }

        }

    }

}

float getFrontHistPositionBaseEstZ()
{

    float return_value = buff[head];

    head++;

    if (head == QUEUE_MAX_LENGTH) {
        head = 0;

    }

    itemCount--;

    return return_value;

}

bool isPositionBaseQueueIsFull()
{

    return itemCount == QUEUE_MAX_LENGTH;

}

/* using ArduPilots Third Order Compilmentary filter */

void apmCalculateEstimatedAltitude(uint32_t currentTime)
{

    static uint32_t previousTime;
    //int32_t vel_tmp;
    float dt = (currentTime - previousTime) / 1000000.0f;
    uint32_t dTime;
    dTime = currentTime - previousTime;

    if (dTime < UPDATE_FREQUENCY)
        return;

    previousTime = currentTime;

    /* Sanity Check */
    if (dTime > 2 * UPDATE_FREQUENCY) {       //Too long. Reset things.
        imuResetAccelerationSum(1);
    }

    if (AltRstRequired && !ARMING_FLAG(ARMED)) //Velocity out of bounds reset variables
        AltRst();

#if defined(BARO) && !(defined(LASER_ALT))
    checkBaro(); // check if new baro readings have arrived and use them to correct vertical accelerometer offsets.
#else
    checkReading();
#endif

    if (accSumCount) {
        accel_ef_z = (float) accSum[2] / (float) accSumCount;
    } else {
        accel_ef_z = 0;
    }

    accZ_tmp = accel_ef_z;
    accel_ef_z = accel_ef_z * accVelScale;
    accel_ef_z = constrainf(accel_ef_z, -800, 800);

    altholdDebug6 = accZ_tmp;

    imuResetAccelerationSum(1); //Check position of this

    if (first_velocity_reads <= 5) { // discarding first five readings TODO why?
        first_velocity_reads++;
        return;
    }

    _accel_correction_hbf_z += _position_error_z * _k3_z * dt;
    _velocity_z += _position_error_z * _k2_z * dt;

    _position_correction_z += _position_error_z * _k1_z * dt;

    // calculate velocity increase adding new acceleration from accelerometers
    float velocity_increase_z;
    velocity_increase_z = (accel_ef_z + _accel_correction_hbf_z) * dt;

    // calculate new estimate of position
    _position_base_z += (_velocity_z + velocity_increase_z * 0.5) * dt;

    // update the corrected position estimate
    _position_z = _position_base_z + _position_correction_z;

    // calculate new velocity

    _velocity_z += velocity_increase_z;

    VelocityZ = lrintf(_velocity_z);

    EstAlt = lrintf(_position_z);

//    _accel_correction_hbf_z1 += _position_error_z1 * _k3_z1 * dt;
//    _velocity_z1 += _position_error_z1 * _k2_z1 * dt;
//
//    _position_correction_z1 += _position_error_z1 * _k1_z1 * dt;
//
//    // calculate velocity increase adding new acceleration from accelerometers
//    float velocity_increase_z1;
//    velocity_increase_z1 = (accel_ef_z + _accel_correction_hbf_z1) * dt;
//
//    // calculate new estimate of position
//    _position_base_z1 += (_velocity_z1 + velocity_increase_z1 * 0.5) * dt;
//
//    // update the corrected position estimate
//    _position_z1 = _position_base_z1+ _position_correction_z1;
//
//
//    _velocity_z1 += velocity_increase_z1;
//
//    VelocityZ1 = lrintf(_velocity_z1);
//
//    EstAlt1 = lrintf(_position_z1);

    // store 3rd order estimate (i.e. estimated vertical position) for future use

    addHistPositionBaseEstZ(_position_base_z);

    //   addHistPositionBaseEstZ(_position_base_z1);

    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(VelocityZ, accZ_tmp, accZ_old, dt);

    altholdDebug7 = altHoldThrottleAdjustment;

    accZ_old = accZ_tmp;
    // set vario
    vario = applyDeadband(VelocityZ, 5);
    if (abs(VelocityZ) > 200)
        AltRstRequired = 1;

}

#ifdef LASER_ALT
void checkReading()
{
    uint32_t baro_update_time;
    float dt;
    float tilt = 0;
    //float tofTransition;
    static int32_t baro_offset = 0;

    //Baro update reading
    baro_update_time = getBaroLastUpdate();
    if( baro_update_time != baro_last_update )
    {
        dt = (float)(baro_update_time - baro_last_update) * 0.001f; // in seconds
        Baro_Height = baroCalculateAltitude();
        filtered = (0.75f * filtered) + ((1-0.75f)* Baro_Height);

        baro_last_update = baro_update_time;
    }

    //Laser sensor update reading
    if(isTofDataNew() && (!isOutofRange())) {

        ToF_Height = (float)NewSensorRange/10.0f;
        isTofDataNewflag = false;

        tilt = degreesToRadians(calculateTiltAngle(&inclination)/10);
        if(tilt < 25)
        ToF_Height *= cos_approx(tilt);
    }

    //Fusion
    if ((ToF_Height > 0 && ToF_Height < 200) && (!isOutofRange())) {
        //   	baro_offset = ToF_Height-filtered;
        baro_offset = filtered-ToF_Height;
        //baroAlt_offset_print = baro_offset;
        correctedWithTof(ToF_Height);
    } /* else
     //{ Baro_Height -= baro_offset;
     if (ToF_Height >= 120  && ToF_Height <= 200) {
     //tofTransition = (200 - ToF_Height) / 100.0f;
     tofTransition = 0.5f;
     fused = ToF_Height * tofTransition + Baro_Height * (1.0f - tofTransition);

     correctedWithBaro( fused, dt);
     } */
    else {
        correctedWithBaro(Baro_Height-baro_offset, dt);
    }
}
#endif

void checkBaro()
{

    uint32_t baro_update_time;

    // calculate time since last baro reading (in ms)

    baro_update_time = getBaroLastUpdate();

    if (baro_update_time != baro_last_update) {

        const float dt = (float) (baro_update_time - baro_last_update) * 0.001f; // in seconds
        // call correction method
        correctedWithBaro(baroCalculateAltitude(), dt);

        baro_last_update = baro_update_time;
    }

//    if (baro_update_time != baro_last_update1) {
//
//
//        const float dt = (float) (baro_update_time - baro_last_update1) * 0.001f; // in seconds
//        // call correction method
//        correctedWithBaro(apmBaroCalculateAltitude(), dt);
//
//
//        baro_last_update1 = baro_update_time;
//    }

}

void correctedWithBaro(float baroAlt, float dt)
{

    altholdDebug5 = baroAlt;

    if (dt > 0.5f) {
        return;
    }

    float hist_position_base_z = 0;

    if (first_reads <= 10) {
        setAltitude(baroAlt);
        first_reads++;

    }

//    if (first_reads1 <= 10) {
//
//        _position_base_z1 = baroAlt;
//        _position_correction_z1 = 0;
//        _position_z1 = baroAlt; // _position = _position_base + _position_correction
//        last_hist_position1 = 0; // reset z history to avoid fake z velocity at next baro calibration (next rearm)
//        imuResetAccelerationSum(1);
//
//        first_reads1++;
//
//    }

//    if (isPositionBaseQueueIsFull()) {
//        hist_position_base_z = getFrontHistPositionBaseEstZ();
//
//    } else {
//        hist_position_base_z = _position_base_z1;
//    }

    if (isPositionBaseQueueIsFull()) {
        hist_position_base_z = getFrontHistPositionBaseEstZ();

    } else {
        hist_position_base_z = _position_base_z;
    }

    // calculate error in position from baro with our estimate
    _position_error_z = baroAlt - (hist_position_base_z + _position_correction_z);

//    _position_error_z1 = baroAlt
//            - (hist_position_base_z + _position_correction_z1);

//    updateGains1();

//     #ifdef LASER_TOF1
//     if(_time_constant_z != 5.0f)
//     {
//     _time_constant_z = 5;
//     updateGains();
//     }
//     #endif

    // Deci-degrees: 300 = 30 deg ( the old 30 meant 3 deg, so the slow ~15 s filter
    // ran all the time and the estimate lagged high in descents ). 30 deg is above
    // the 20 deg max_angle_inclination, so this only trips in aggressive flight.
    if (ABS(inclination_generalised.values.rollDeciDegrees) > 300 || ABS(inclination_generalised.values.pitchDeciDegrees) > 300) {

        _time_constant_z = 5;
        updateGains();

    }

    else {

        _time_constant_z = 2;
        updateGains();

    }

}

#ifdef LASER_ALT
void correctedWithTof(float ToF_Height) {
    if( first_reads == 0 )
    {
        setAltitude(ToF_Height);
        first_reads++;
    }
    _position_error_z = ToF_Height - EstAlt;
    if(_time_constant_z!=1.5f) {
        _time_constant_z = 1.5;
        updateGains();
    }
}
#endif

void updateGains()
{
    if (_time_constant_z == 0.0f) {
        _k1_z = _k2_z = _k3_z = 0.0f;
    } else {
        _k1_z = 3.0f / _time_constant_z;
        _k2_z = 3.0f / (_time_constant_z * _time_constant_z);
        _k3_z = 1.0f / (_time_constant_z * _time_constant_z * _time_constant_z);
    }

}

void updateGains1()
{
    if (_time_constant_z1 == 0.0f) {
        _k1_z1 = _k2_z1 = _k3_z1 = 0.0f;
    } else {
        _k1_z1 = 3.0f / _time_constant_z1;
        _k2_z1 = 3.0f / (_time_constant_z1 * _time_constant_z1);
        _k3_z1 = 1.0f / (_time_constant_z1 * _time_constant_z1 * _time_constant_z1);
    }

}

void setAltitude(float new_altitude)

{
    _position_base_z = new_altitude;
    _position_correction_z = 0;
    _position_z = new_altitude; // _position = _position_base + _position_correction
    last_hist_position = 0; // reset z history to avoid fake z velocity at next baro calibration (next rearm)
    imuResetAccelerationSum(1);
}

int32_t altitudeHoldGetEstimatedAltitude(void)
{

    return EstAlt;

}

int32_t getSetVelocity(void)
{
    return setVelocity;
}

int32_t getSetAltitude(void)
{

    return AltHold;
}

void AltRst(void)
{

    _velocity_z = 0;
    imuResetAccelerationSum(1);
    AltRstRequired = 0;
    initialThrottleHold = 1600;
    errorVelocityI = 0;
    altHoldThrottleAdjustment = 0;
}

float getTimeConstant()
{
    return _time_constant_z;
}

#endif

void setAltitude(int32_t altitude)
{

    AltHold = altitude;

}

void setRelativeAltitude(int32_t altitude)
{

    AltHold = EstAlt + altitude;

}

int32_t getEstAltitude()
{

    return EstAlt;

}

int32_t getEstVelocity()
{

    return VelocityZ;

}

int32_t getEstAltitude1()
{

    return EstAlt1;

}

int32_t getEstVelocity1()
{

    return VelocityZ1;

}

bool limitAltitude()
{

    if (max_altitude != -1 && IS_RC_MODE_ACTIVE(BOXBARO)) {

        if (EstAlt >= (max_altitude - ALT_CEILING_MARGIN_CM)) {

            return true;

        }

    }

    return false;
}
