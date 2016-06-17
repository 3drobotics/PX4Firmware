/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file oreoled_bootloader.h
 * @author Angus Peaty <angusp@gmail.com>
 */

#ifndef OREOLED_BOOTLOADER_H
#define OREOLED_BOOTLOADER_H

#include "oreoled_bootloader.h"

enum oreoled_bl_target_t {
	OREOLED_BL_TARGET_INVALID,
	OREOLED_BL_TARGET_AVR,
	OREOLED_BL_TARGET_ENUM_COUNT
};

#define OREOLED_BL_TARGET_DEFAULT		OREOLED_BL_TARGET_AVR

class OREOLED_BOOTLOADER
{
public:
	OREOLED_BOOTLOADER() {};
	virtual ~OREOLED_BOOTLOADER() {};

	virtual int		init() = 0;
	virtual int		info() = 0;
	virtual int		ioctl(unsigned cmd, unsigned long arg) = 0;

	/* Start the update process */
	virtual void	start() = 0;

	/* Kill the update process */
	virtual void	kill() = 0;

	/* returns true once the driver finished bootloading and ready for commands */
	virtual bool	is_ready() = 0;

	void cycle_trampoline(void *arg);

protected:
	virtual void	cycle() = 0;
};

#endif /* OREOLED_BOOTLOADER_H */
