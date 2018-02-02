/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file TF02.hpp
 *
 * UAVCAN --> ORB bridge for TF02 messages:
 *     uavcan.equipment.TF02data
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 */

#pragma once


#include <uavcan/equipment/laser/TF02data.hpp>
#include "sensor_bridge.hpp"
#include <drivers/drv_laser.h>
#include <drivers/device/ringbuffer.h>
#include <uORB/topics/distance_sensor.h>

class UavcanTF02Bridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanTF02Bridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	/**
	 * laser tf02 message will be reported via this callback.
	 */
	int				_orb_class_instance;
    orb_advert_t			_tf02_topic;
	struct distance_sensor_s ds_report;
	void laserTF02_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::laser::TF02data> &msg);

	typedef uavcan::MethodBinder < UavcanTF02Bridge *,
		void (UavcanTF02Bridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::laser::TF02data> &) >
		laserTF02CbBinder;
	uavcan::Subscriber<uavcan::equipment::laser::TF02data, laserTF02CbBinder> _sub_laserTF02;
};


