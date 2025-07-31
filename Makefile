###############################################################################
#  SPDX-License-Identifier: GPL-3.0-or-later                                  #
#  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
#  -------------------------------------------------------------------------  #
#  Copyright (c) 2025 Drona Aviation                                          #
#  All rights reserved.                                                       #
#  -------------------------------------------------------------------------  #
#  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
#  Project: MagisV2-MechAsh-Dev                                               #
#  File: \Makefile                                                            #
#  Created Date: Mon, 28th Apr 2025                                           #
#  Brief:                                                                     #
#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
#  Last Modified: Tue, 29th Apr 2025                                          #
#  Modified By: AJ                                                            #
#  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
#  HISTORY:                                                                   #
#  Date      	By	Comments                                                    #
#  ----------	---	---------------------------------------------------------   #
###############################################################################
#
# Makefile for building the cleanflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#
###############################################################################
# User-configurable options
FORKNAME	=	MAGIS
TARGET	?=	PRIMUSX
BUILD_TYPE	?= BIN
LIB_MAJOR_VERSION	=	0
LIB_MINOR_VERSION	=	3
FW_Version	=	1.0.0
API_Version	=	1.0.1
# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE	?=
# Debugger optons, must be empty or GDB
DEBUG	?=
# Serial port/Device for flashing
SERIAL_DEVICE	?=	$(firstword $(wildcard /dev/ttyUSB*) no-port-found)


# Compile-time options
OPTIONS	?= 	'__FORKNAME__="$(FORKNAME)"' \
		   			'__TARGET__="$(TARGET)"' \
			 			'__FW_VER__="$(FW_Version)"' \
		   			'__API_VER__="$(API_Version)"' \
        		'__BUILD_DATE__="$(shell date +%Y-%m-%d)"' \
        		'__BUILD_TIME__="$(shell date +%H:%M:%S)"' \

# Configure default flash sizes for the targets
ifeq ($(FLASH_SIZE),)
	ifeq ($(TARGET),PRIMUSV3R)
		FLASH_SIZE = 128
	else ifeq ($(TARGET),PRIMUSX)
		FLASH_SIZE = 256
	else
		$(error FLASH_SIZE not configured for target)
	endif
endif

VALID_TARGETS	=	PRIMUSV3R PRIMUSX

###############################################################################
# Directorie

ROOT	:=	$(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR	=	$(ROOT)/src/main
OBJECT_DIR	=	$(ROOT)/obj/main
BIN_DIR	=	$(ROOT)/obj
CMSIS_DIR	=	$(ROOT)/lib/main/CMSIS
INCLUDE_DIRS	=	$(SRC_DIR)
LINKER_DIR	=	$(ROOT)/src/main/target

# Search path for sources
VPATH	:=	$(SRC_DIR) \
					$(SRC_DIR)/startup


USBFS_DIR	=	$(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC	=	$(notdir $(wildcard $(USBFS_DIR)/src/*.c))

# Ranging sensor VL53L0X libraries
RANGING_DIR	=	$(ROOT)/lib/main/VL53L0X_API
RANGING_SRC	=	$(notdir $(wildcard $(RANGING_DIR)/core/src/*.c \
																	$(RANGING_DIR)/platform/src/*.c\
																	$(RANGING_DIR)/core/src/*.cpp \
																	$(RANGING_DIR)/platform/src/*.cpp))

INCLUDE_DIRS	:=	$(INCLUDE_DIRS) \
              		$(RANGING_DIR)/core/inc \
              		$(RANGING_DIR)/platform/inc   

VPATH := 	$(VPATH) \
					$(RANGING_DIR)/core/src \
					$(RANGING_DIR)/platform/src


CSOURCES	:=	$(shell find $(SRC_DIR) -name '*.c')

# MCU and Peripheral settings for PRIMUSX
ifeq ($(TARGET),PRIMUSX)

STDPERIPH_DIR = $(ROOT)/lib/main/STM32F30x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	=	stm32f30x_crc.c \
    				stm32f30x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

VPATH := 	$(VPATH) \
					$(CMSIS_DIR)/CM1/CoreSupport \
					$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x

CMSIS_SRC = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
               									$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
           			$(STDPERIPH_DIR)/inc \
           			$(CMSIS_DIR)/CM1/CoreSupport \
           			$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x \
           			$(USBFS_DIR)/inc \
           			$(ROOT)/src/main/vcp

VPATH :=	$(VPATH) \
					$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC	:=	$(DEVICE_STDPERIPH_SRC) \
          								$(USBPERIPH_SRC)

LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS = -DSTM32F303xC -DSTM32F303
TARGET_FLAGS = -D$(TARGET)

else

STDPERIPH_DIR = $(ROOT)/lib/main/STM32F10x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	=	stm32f10x_crc.c \
						stm32f10x_cec.c \
						stm32f10x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

VPATH := 	$(VPATH) \
					$(CMSIS_DIR)/CM3/CoreSupport \
					$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x

CMSIS_SRC = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
               									$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
           			$(STDPERIPH_DIR)/inc \
           			$(CMSIS_DIR)/CM3/CoreSupport \
           			$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \


LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f103_$(FLASH_SIZE)k.ld

ARCH_FLAGS = -mthumb -mcpu=cortex-m3 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS = -DSTM32F10X_MD -DSTM32F10X
TARGET_FLAGS = -D$(TARGET) -pedantic



endif

ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

TARGET_DIR = $(ROOT)/src/main/target/$(TARGET)
TARGET_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))



INCLUDE_DIRS	:=	$(INCLUDE_DIRS) \
		    					$(TARGET_DIR)

VPATH	:=	$(VPATH) \
					$(TARGET_DIR)

MAIN_COMMON = common/maths.cpp \
		   				common/printf.cpp \
		   				common/typeconversion.cpp \
		   				common/encoding.cpp \

MAIN_FLIGHT = flight/altitudehold.cpp \
		   				flight/failsafe.cpp \
		   				flight/pid.cpp \
		   				flight/imu.cpp \
		   				flight/mixer.cpp \
		   				flight/lowpass.cpp \
		   				flight/filter.cpp \
		   				flight/navigation.cpp \
		   				flight/gps_conversion.c \

MAIN_CONFIG = config/config.cpp \
		   				config/runtime_config.cpp 

MAIN_DRIVERS =	drivers/bus_i2c_soft.cpp \
		   					drivers/serial.cpp\
		   					drivers/sound_beeper.c \
		   					drivers/system.c \

MAIN_IO = io/beeper.cpp \
       		io/oled_display.c \
		   		io/rc_controls.cpp \
		   		io/rc_curves.cpp \
		   		io/serial.cpp \
		   		io/serial_1wire.cpp \
		   		io/serial_cli.cpp \
		   		io/serial_msp.cpp \
		   		io/statusindicator.cpp \
		   		io/flashfs.cpp \
		   		io/gps.cpp \

MAIN_RX = rx/rx.cpp \
		   		rx/pwm.c \
		   		rx/msp.c \
		   		rx/sbus.c \
		   		rx/sumd.c \
		   		rx/sumh.c \
		   		rx/spektrum.c \
		   		rx/xbus.cpp \

MAIN_SENSOR = sensors/acceleration.cpp \
		   				sensors/battery.cpp \
		   				sensors/boardalignment.cpp \
		   				sensors/compass.cpp \
		   				sensors/gyro.cpp \
		   				sensors/initialisation.cpp \

MAIN_BLACKBOX = blackbox/blackbox.cpp \
		   					blackbox/blackbox_io.cpp \

COMMON_SRC = 	build_config.cpp \
		   				debug.cpp \
		   				version.cpp \
		   				main.cpp \
		   				mw.cpp \
							$(TARGET_SRC) \
		   				$(MAIN_CONFIG) \
		   				$(MAIN_COMMON) \
		   				$(MAIN_FLIGHT) \
		   				$(MAIN_DRIVERS) \
		   				$(MAIN_IO) \
		   				$(MAIN_RX) \
		   				$(MAIN_SENSOR) \
		   				$(MAIN_BLACKBOX) \
		   				$(CMSIS_SRC) \
		   				$(DEVICE_STDPERIPH_SRC) \

HIGHEND_SRC = \
		   flight/gtune.c \
		   flight/navigation.c \
		   flight/gps_conversion.c \
		   common/colorconversion.c \
		   io/gps.c \
		   io/ledstrip.c \
		   io/display.c \
		   telemetry/telemetry.c \
		   telemetry/frsky.c \
		   telemetry/hott.c \
		   telemetry/msp.c \
		   telemetry/smartport.c \
		   sensors/sonar.c \
		   sensors/barometer.c \
		   blackbox/blackbox.c \
		   blackbox/blackbox_io.c

VCP_SRC = \
		   vcp/hw_config.c \
		   vcp/stm32_it.c \
		   vcp/usb_desc.c \
		   vcp/usb_endp.c \
		   vcp/usb_istr.c \
		   vcp/usb_prop.c \
		   vcp/usb_pwr.c \
		   drivers/serial_usb_vcp.c

DRONA_FLIGHT = 	flight/acrobats.cpp \
								flight/posControl.cpp \
            		flight/posEstimate.cpp \
            		flight/opticflow.cpp \

DRONA_DRIVERS = drivers/opticflow_paw3903.cpp \
								drivers/display_ug2864hsweg01 \
            		drivers/ranging_vl53l0x.cpp \
            		drivers/sc18is602b.cpp \

DRONA_COMMAND = command/command.cpp \
            		command/localisationCommand.cpp \

DRONA_API =	API/Specifiers.cpp \
		    		API/Peripheral.cpp \
		    		API/XRanging.cpp \
						API/Sensor.cpp \
						API/Control.cpp \
						API/Estimate.cpp \
						API/Utils.cpp\
						API/User.cpp\
						API/Motor.cpp\
						API/API-Utils.cpp\
						API/RxConfig.cpp\
						API/Localisation.cpp\

DRONA_SRC = $(DRONA_FLIGHT) \
						$(DRONA_DRIVERS) \
						$(DRONA_COMMAND) \
						$(DRONA_API) \

PRIMUSV3R_DRIVERS = drivers/adc.cpp \
		   							drivers/adc_stm32f10x.cpp \
		   							drivers/accgyro_mpu.cpp \
		   							drivers/accgyro_mpu6500.cpp \
		   							drivers/bus_i2c_stm32f10x.c \
		   							drivers/bus_spi.c \
		   							drivers/compass_ak8963.cpp \
		   							drivers/gpio_stm32f10x.cpp \
		   							drivers/light_led_stm32f10x.cpp \
		   							drivers/flash_m25p16.cpp \
		   							drivers/pwm_mapping.cpp \
		   							drivers/pwm_output.cpp \
		   							drivers/pwm_rx.cpp \
		   							drivers/serial_uart.c \
		   							drivers/serial_uart_stm32f10x.c \
		   							drivers/sound_beeper_stm32f10x.cpp \
		   							drivers/system_stm32f10x.cpp \
		   							drivers/timer.cpp \
		   							drivers/timer_stm32f10x.cpp \
		   							drivers/barometer_ms5611.cpp \

PRIMUSV3R_SENSORS = 	sensors/barometer.cpp \

PRIMUSV3R_SRC = startup_stm32f10x_md_gcc.S \
		  					$(COMMON_SRC) \
      					$(DRONA_SRC) \
		  					$(RANGING_SRC) \
		  					$(PRIMUSV3R_DRIVERS) \
		  					$(PRIMUSV3R_SENSORS) \
								hardware_revision.cpp \

PRIMUSX_DRIVERS = 	drivers/adc.cpp \
		   							drivers/adc_stm32f30x.c \
		   							drivers/accgyro_mpu.cpp \
		   							drivers/accgyro_mpu6500.cpp \
		   							drivers/accgyro_icm20948.cpp \
		   							drivers/bus_i2c_stm32f30x.c \
		   							drivers/bus_spi.c \
		   							drivers/compass_ak8963.cpp \
		   							drivers/compass_ak09916.cpp \
		   							drivers/gpio_stm32f30x.c \
		   							drivers/light_led_stm32f30x.c \
		   							drivers/flash_m25p16.cpp \
		   							drivers/pwm_mapping.cpp \
		   							drivers/pwm_output.cpp \
		   							drivers/pwm_rx.cpp \
		   							drivers/serial_uart.c \
		   							drivers/serial_uart_stm32f30x.c \
		   							drivers/sound_beeper_stm32f30x.c \
		   							drivers/system_stm32f30x.c \
		   							drivers/timer.cpp \
		   							drivers/timer_stm32f30x.c \
		   							drivers/barometer_ms5611.cpp \
		   							drivers/barometer_icp10111.cpp \

PRIMUSX_SENSORS = 	sensors/barometer.cpp \

PRIMUSX_SRC = startup_stm32f30x_md_gcc.S \
		  					$(RANGING_SRC) \
		  					$(PRIMUSX_DRIVERS) \
		  					$(PRIMUSX_SENSORS) \
		  					$(COMMON_SRC) \
      					$(DRONA_SRC) \

ifeq ($(BUILD_TYPE),BIN)
$(TARGET)_SRC:=$($(TARGET)_SRC)\
			API/PlutoPilot.cpp
endif               

# Search path and source files for the ST stdperiph library
VPATH	:=	$(VPATH) \
					$(STDPERIPH_DIR)/src

###############################################################################
# Compiler and Tools
CC = arm-none-eabi-g++
C = arm-none-eabi-gcc
AR = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size


ifeq ($(DEBUG),GDB)
OPTIMIZE	=	-O0
LTO_FLAGS	=	$(OPTIMIZE)
else
OPTIMIZE	=	-Os
LTO_FLAGS	=	-flto --use-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS	 = -ggdb3 -DDEBUG

CFLAGS	=	$(ARCH_FLAGS) \
		   		$(LTO_FLAGS) \
		   		$(addprefix -D,$(OPTIONS)) \
		   		$(addprefix -I,$(INCLUDE_DIRS)) \
		   		$(DEBUG_FLAGS) \
		   		-std=gnu17 \
		   		-Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
					-Wshadow -Wundef -Wconversion -Wsign-conversion \
		   		-ffunction-sections \
		   		-fdata-sections \
		   		-ffat-lto-objects\
		   		$(DEVICE_FLAGS) \
		   		-DUSE_STDPERIPH_DRIVER \
		   		$(TARGET_FLAGS) \
		   		-save-temps=obj \
		   		-MMD -MP


CCFLAGS	=	$(ARCH_FLAGS) \
		   		$(LTO_FLAGS) \
		   		$(addprefix -D,$(OPTIONS)) \
		   		$(addprefix -I,$(INCLUDE_DIRS)) \
		   		$(DEBUG_FLAGS) \
		   		-std=gnu++17 \
		   		-Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
					-Wshadow -Wundef -Wconversion -Wsign-conversion \
		   		-ffunction-sections \
		   		-fdata-sections \
		   		-ffat-lto-objects\
		   		$(DEVICE_FLAGS) \
		   		-DUSE_STDPERIPH_DRIVER \
		   		$(TARGET_FLAGS) \
		   		-save-temps=obj \
		   		-MMD -MP

ASFLAGS	= $(ARCH_FLAGS) \
		   		-x assembler-with-cpp \
		   		$(addprefix -I,$(INCLUDE_DIRS)) \
		  		-MMD -MP

LDFLAGS	= -lm \
		   		-nostartfiles \
		   		--specs=nosys.specs \
		   		-lc \
		   		-lnosys \
		   		$(ARCH_FLAGS) \
		   		$(LTO_FLAGS) \
		   		$(DEBUG_FLAGS) \
		   		-static \
		   		-Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   		-Wl,-L$(LINKER_DIR) \
		   		-T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK = 	cppcheck $(CSOURCES) --enable=all --platform=unix64 \
		   			--std=c99 --inline-suppr --quiet --force \
		   			$(addprefix -I,$(INCLUDE_DIRS)) \
		   			-I/usr/include -I/usr/include/linux


## all         : default task; compile C code, build firmware

ifeq ($(BUILD_TYPE),BIN)
all: binary
else 
all: libcreate
endif
#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

TARGET_BIN	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).bin
TARGET_HEX	 = $(BIN_DIR)/Experience_PLUTOX.hex
TARGET_ELF	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_DEPS	 = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF) 

# Compile


libs/libpluto_$(LIB_MAJOR_VERSION).$(LIB_MINOR_VERSION).a: $(TARGET_OBJS)
	mkdir -p $(dir $@)
	$(AR) rcs $@ $^


$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CCFLAGS) $<


$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(C) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<


libcreate: libs/libpluto_$(LIB_MAJOR_VERSION).$(LIB_MINOR_VERSION).a

## clean       : clean up all temporary / machine-generated files
clean:
	rm -f $(TARGET_BIN) $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(OBJECT_DIR)/$(TARGET)
	cd src/test && $(MAKE) clean || true

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## flash       : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

st-flash_$(TARGET): $(TARGET_BIN)
	st-flash --reset write $< 0x08000000

## st-flash    : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

binary: $(TARGET_HEX)

unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## unbrick     : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck    : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

## help        : print this help message and exit
help: Makefile
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
	@sed -n 's/^## //p' $<

## test        : run the cleanflight test suite
test:
	cd src/test && $(MAKE) test || true

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# include auto-generated dependencies
-include $(TARGET_DEPS)