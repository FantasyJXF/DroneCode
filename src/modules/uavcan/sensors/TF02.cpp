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
 * @file gnss.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 *
 */
#include <drivers/drv_hrt.h>
#include "TF02.hpp"
const char *const UavcanTF02Bridge::NAME = "laserTF02";

UavcanTF02Bridge::UavcanTF02Bridge(uavcan::INode &node) :
        UavcanCDevSensorBridgeBase("uavcan_laserTF02", "/dev/uavcan/laserTF02",TF02_BASE_DEVICE_PATH,ORB_ID(TF02)),
	_sub_laserTF02(node)
{
}

int UavcanTF02Bridge::init()
{

	_tf02_topic = orb_advertise(ORB_ID(TF02), &ds_report);

		if (_tf02_topic == nullptr) {
			DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
			return -1;
		}

	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}
	res = _sub_laserTF02.start(laserTF02CbBinder(this, &UavcanTF02Bridge::laserTF02_sub_cb));

	if (res < 0) {
		warnx("TF02laser sub failed %i", res);
		return res;
	}

	return res;
}


void UavcanTF02Bridge::laserTF02_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::laser::TF02data> &msg)
{
	ds_report.timestamp = hrt_absolute_time();
	ds_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;
	ds_report.id = ID_TF02;
	ds_report.covariance = msg.sig;
	ds_report.current_distance = (msg.distanceH<<8 | msg.distanceL)/100.0f;	// make evident that this range sample is invalid
	
	ds_report.orientation = 0;
	orb_publish(ORB_ID(TF02), _tf02_topic, &ds_report);

	#if 1
	static int count;
	count++;
	if(count == 100)
	{
		count = 0;
		PX4_INFO("tf02 dis:%.4f sig is %.4f\r\n",(double)ds_report.current_distance,(double)ds_report.covariance);
	}
	#endif
}
