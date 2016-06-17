/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drv_oreoled.h
 *
 * Oreo led device API
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

/* oreoled device path */
#define OREOLED0_DEVICE_PATH "/dev/oreoled0"

/*
 * ioctl() definitions
 */

#define _OREOLEDIOCBASE		(0x2d00)
#define _OREOLEDIOC(_n)		(_IOC(_OREOLEDIOCBASE, _n))

/** set constant RGB values */
#define OREOLED_SET_RGB			_OREOLEDIOC(1)

/** run macro */
#define OREOLED_RUN_MACRO		_OREOLEDIOC(2)

/** send bytes */
#define OREOLED_SEND_BYTES		_OREOLEDIOC(3)

/** send reset */
#define OREOLED_SEND_RESET		_OREOLEDIOC(4)

/** force an i2c gencall */
#define OREOLED_FORCE_SYNC		_OREOLEDIOC(5)

/* Oreo LED driver supports up to 4 leds */
#define OREOLED_NUM_LEDS		4

/* instance of 0xff means apply to all instances */
#define OREOLED_ALL_INSTANCES	0xff

/* maximum command length that can be sent to LEDs */
#define OREOLED_CMD_LENGTH_MAX	70

/* maximum command length that can be read from LEDs */
#define OREOLED_CMD_READ_LENGTH_MAX	10

/* maximum number of commands retries */
#define OEROLED_COMMAND_RETRIES	2

/* enum passed to OREOLED_SET_MODE ioctl()
 *	defined by hardware */
enum oreoled_pattern {
	OREOLED_PATTERN_OFF = 0,
	OREOLED_PATTERN_SINE = 1,
	OREOLED_PATTERN_SOLID = 2,
	OREOLED_PATTERN_SIREN = 3,
	OREOLED_PATTERN_STROBE = 4,
	OREOLED_PATTERN_FADEIN = 5,
	OREOLED_PATTERN_FADEOUT = 6,
	OREOLED_PATTERN_PARAMUPDATE = 7,
	OREOLED_PATTERN_ENUM_COUNT
};

/* enum passed to OREOLED_SET_MODE ioctl()
 *	defined by hardware */
enum oreoled_param {
	OREOLED_PARAM_BIAS_RED = 0,
	OREOLED_PARAM_BIAS_GREEN = 1,
	OREOLED_PARAM_BIAS_BLUE = 2,
	OREOLED_PARAM_AMPLITUDE_RED = 3,
	OREOLED_PARAM_AMPLITUDE_GREEN = 4,
	OREOLED_PARAM_AMPLITUDE_BLUE = 5,
	OREOLED_PARAM_PERIOD = 6,
	OREOLED_PARAM_REPEAT = 7,
	OREOLED_PARAM_PHASEOFFSET = 8,
	OREOLED_PARAM_MACRO = 9,
	OREOLED_PARAM_RESET = 10,
	OREOLED_PARAM_APP_CHECKSUM = 11,
	OREOLED_PARAM_ENUM_COUNT
};

/* enum of available macros
 * 	defined by hardware */
enum oreoled_macro {
	OREOLED_PARAM_MACRO_RESET = 0,
	OREOLED_PARAM_MACRO_COLOUR_CYCLE = 1,
	OREOLED_PARAM_MACRO_BREATH = 2,
	OREOLED_PARAM_MACRO_STROBE = 3,
	OREOLED_PARAM_MACRO_FADEIN = 4,
	OREOLED_PARAM_MACRO_FADEOUT = 5,
	OREOLED_PARAM_MACRO_RED = 6,
	OREOLED_PARAM_MACRO_GREEN = 7,
	OREOLED_PARAM_MACRO_BLUE = 8,
	OREOLED_PARAM_MACRO_YELLOW = 9,
	OREOLED_PARAM_MACRO_WHITE = 10,
	OREOLED_PARAM_MACRO_AUTOMOBILE = 11,
	OREOLED_PARAM_MACRO_AVIATION = 12,
	OREOLED_PARAM_MACRO_ENUM_COUNT
};

/*
  structure passed to OREOLED_SET_RGB ioctl()
  Note that the driver scales the brightness to 0 to 255, regardless
  of the hardware scaling
 */
typedef struct {
	uint8_t instance;
	oreoled_pattern pattern;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} oreoled_rgbset_t;

/*
  structure passed to OREOLED_RUN_MACRO ioctl()
 */
typedef struct {
	uint8_t instance;
	oreoled_macro macro;
} oreoled_macrorun_t;

/*
  structure passed to send_bytes method (only used for testing)
 */
typedef struct {
	uint8_t led_num;
	uint8_t num_bytes;
	uint8_t buff[OREOLED_CMD_LENGTH_MAX];
} oreoled_cmd_t;

