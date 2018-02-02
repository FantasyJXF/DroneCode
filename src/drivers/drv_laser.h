/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file drv_baro.h
 *
 * Barometric pressure sensor driver interface.
 */

#ifndef _DRV_LASER_H
#define _DRV_LASER_H

#include <px4_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define TF02_BASE_DEVICE_PATH	"/dev/laserTF02"
#define TF02_DEVICE_PATH	"/dev/laserTF02"

#include <uORB/topics/sensor_laserTF02.h>
#define laserTF02_report sensor_laserTF02_s


// #define NRA24_BASE_DEVICE_PATH	"/dev/laserNRA24"
// #define NRA24_DEVICE_PATH	"/dev/laserNRA24"

// #define liteV3_BASE_DEVICE_PATH	"/dev/liteV3"
// #define liteV3_DEVICE_PATH	"/dev/liteV3"

// #include <uORB/topics/sensor_laserNRA24.h>
// #define laserNRA24_report sensor_laserNRA24_s

// #define ulanding_BASE_DEVICE_PATH	"/dev/ulanding"
// #define ulanding_DEVICE_PATH	"/dev/ulanding"

// #include <uORB/topics/sensor_ulanding.h>
// #define ulanding_report sensor_ulanding_s



#define ID_ULANDIGN   0
#define ID_NRA24      1
#define ID_liteV3     2
#define ID_TF02       3


/*
 * ioctl() definitions
 */
#if 0
#define _BAROIOCBASE		(0x2200)
#define _BAROIOC(_n)		(_PX4_IOC(_BAROIOCBASE, _n))

/** set corrected MSL pressure in pascals */
#define BAROIOCSMSLPRESSURE	_BAROIOC(0)

/** get current MSL pressure in pascals */
#define BAROIOCGMSLPRESSURE	_BAROIOC(1)
#endif

#endif /* _DRV_BARO_H */
