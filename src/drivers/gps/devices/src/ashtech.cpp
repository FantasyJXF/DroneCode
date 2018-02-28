/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <mathlib/mathlib.h>

#include "ashtech.h"


#define DEBUG_INFO  0



GPSDriverAshtech::GPSDriverAshtech(GPSCallbackPtr callback, void *callback_user,
				   struct vehicle_gps_position_s *gps_position,
				   struct satellite_info_s *satellite_info):
	GPSHelper(callback, callback_user),
	_satellite_info(satellite_info),
	_gps_position(gps_position),
	_gps_position_last(gps_position),
	_last_timestamp_time(0),
	_got_pashr_pos_message(false)
{
	decodeInit();
	_decode_state = NME_DECODE_UNINIT;
	_rx_buffer_bytes = 0;

	_gps_position_last->eph = 1.0;
	_gps_position_last->epv = 2.0;
	crc32Count = 0;
}

GPSDriverAshtech::~GPSDriverAshtech()
{
}


int GPSDriverAshtech::handleMessage(int len)
{
	char *endp;

	if (len < 7) { 
		PX4_INFO("gps recieve len<7");
		return 0; }

	int uiCalcComma = 0;

	for (int i = 0 ; i < len; i++) {
		if (_rx_buffer[i] == ',') { uiCalcComma++; }
	}

	char *bufptr = (char *)(_rx_buffer + 6);
	int ret = 0;
        if ((memcmp(_rx_buffer + 3, "GST,", 3) == 0) && (uiCalcComma == 8)) {

			#if DEBUG_INFO
			PX4_INFO("gst handle");
			#endif
		double ashtech_time __attribute__((unused)) = 0.0, lat_err = 0.0, lon_err = 0.0, alt_err = 0.0;
		double min_err __attribute__((unused)) = 0.0, maj_err __attribute__((unused)) = 0.0,
		deg_from_north __attribute__((unused)) = 0.0, rms_err __attribute__((unused)) = 0.0;

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { rms_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { maj_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { min_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { deg_from_north = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lat_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lon_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { alt_err = strtod(bufptr, &endp); bufptr = endp; }

		if(((lat_err) <=0.0) || ((lat_err) <=0.0))
		{
			_gps_position->eph = _gps_position_last->eph;
			_gps_position->epv = _gps_position_last->epv;
		}
		else
		{
			_gps_position->eph = sqrtf(static_cast<float>(lat_err) * static_cast<float>(lat_err)
					   + static_cast<float>(lon_err) * static_cast<float>(lon_err));
			_gps_position->epv = static_cast<float>(alt_err);
			_gps_position_last->eph = _gps_position->eph;
			_gps_position_last->epv = _gps_position->epv;
		}
		_gps_position->timestamp = gps_absolute_time();

		ret = 1;
	} 
	else if ((memcmp(_rx_buffer + 3, "GGA,", 3) == 0)&& (uiCalcComma == 14) ) {
		#if DEBUG_INFO
		PX4_INFO("GGA handle");		
		#endif
		double ashtech_time __attribute__((unused)) = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
		int num_of_sv __attribute__((unused)) = 0, fix_quality = 0;
		double hdop __attribute__((unused)) = 99.9;
		char ns = '?', ew = '?';

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }//UTC时间

		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }//纬度

		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }//纬度半球

		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }//经度

		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }// 经度半球

		if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }//定位质量指示

		if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }//使用卫星数量

		if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }//水平精确度

		if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }//天线离海平面的高度

		if (ns == 'S') {
			lat = -lat;
		}

		if (ew == 'W') {
			lon = -lon;
		}


		//if((fabs(lat)<=0.0) || (fabs(lon)<=0.0))
		//{
		//	_gps_position->lat = _gps_position_last->lat;
		//	_gps_position->lon = _gps_position_last->lon;
		//	_gps_position->alt = _gps_position_last->alt;
		//	_gps_position->satellites_used = _gps_position_last->satellites_used;
		//}
		//else
		//{
				/* convert from degrees, minutes and seconds to degrees * 1e7 */
			_gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
			_gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
			_gps_position->alt = static_cast<int>(alt * 1000);
			_gps_position->satellites_used = num_of_sv;

		//	_gps_position_last->lat = _gps_position->lat;
		//	_gps_position_last->lon = _gps_position->lon;
		//	_gps_position_last->alt = _gps_position->alt;
		//	_gps_position_last->satellites_used = _gps_position->satellites_used;
		//}


		_rate_count_lat_lon++;
		
		if (fix_quality <= 0) {
			_gps_position->fix_type = 0;

		} else {
			/*
			 * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
			 * and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position->fix_type
			 */
			if (fix_quality == 5) { fix_quality = 3; }

			/*
			 * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
			 */
			_gps_position->fix_type = 3 + fix_quality - 1;
		}

		_gps_position->timestamp = gps_absolute_time();
		ret = 2;

	} 
	else if ((memcmp(_rx_buffer + 3, "GSA,", 3) == 0)&& (uiCalcComma == 17) ) {
		#if DEBUG_INFO
		PX4_INFO("GSA handle");		
		#endif
		int i = 0;
		double prn __attribute__((unused)) = 99.9;;
		double pdop __attribute__((unused)) = 99.9;
		double hdop __attribute__((unused)) = 99.9;
		double vdop __attribute__((unused)) = 99.9;
		char ns __attribute__((unused)) = '?';
		char ew __attribute__((unused)) = '?';

		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }//1
		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }//2

           	for(i=0;i<12;i++)//prn1-prn12
           	{

           		if (bufptr && *(++bufptr) != ',') { prn = strtod(bufptr, &endp); bufptr = endp; }//1
           	}
		
		if (bufptr && *(++bufptr) != ',') { pdop = strtod(bufptr, &endp); bufptr = endp; }//2
		if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }
		if (bufptr && *(++bufptr) != ',') { vdop = strtod(bufptr, &endp); bufptr = endp; }

	//	if((fabs(hdop)<=0.0) || (fabs(vdop)<=0.0))
		//{
		//	_gps_position->hdop = _gps_position_last->hdop;
		//	_gps_position->hdop = _gps_position_last->hdop;
		//}
		//else
		//{
			_gps_position->hdop = (float)hdop;
			_gps_position->vdop = (float)vdop;
		//	_gps_position_last->hdop = (float)hdop;
		//	_gps_position_last->vdop = (float)vdop;
		//}
		

		_gps_position->timestamp = gps_absolute_time();
		
		ret = 4;

	} 


	else if ((memcmp(_rx_buffer + 3, "ZDA,", 3) == 0) && (uiCalcComma == 6)) {
		#if DEBUG_INFO
		PX4_INFO("zda handle");	
		#endif
		double ashtech_time = 0.0;
		int day = 0, month = 0, year = 0, local_time_off_hour __attribute__((unused)) = 0,
		    local_time_off_min __attribute__((unused)) = 0;

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { day = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { month = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { year = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { local_time_off_hour = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { local_time_off_min = strtol(bufptr, &endp, 10); bufptr = endp; }


		int ashtech_hour = static_cast<int>(ashtech_time / 10000);
		int ashtech_minute = static_cast<int>((ashtech_time - ashtech_hour * 10000) / 100);
		double ashtech_sec = static_cast<float>(ashtech_time - ashtech_hour * 10000 - ashtech_minute * 100);

		/*
		 * convert to unix timestamp
		 */
		struct tm timeinfo;
		timeinfo.tm_year = year - 1900;
		timeinfo.tm_mon = month - 1;
		timeinfo.tm_mday = day;
		timeinfo.tm_hour = ashtech_hour;
		timeinfo.tm_min = ashtech_minute;
		timeinfo.tm_sec = int(ashtech_sec);

#ifndef NO_MKTIME
		time_t epoch = mktime(&timeinfo);

		epoch = epoch + 28800;//8 hour
		if (epoch > GPS_EPOCH_SECS) {
			uint64_t usecs = static_cast<uint64_t>((ashtech_sec - static_cast<uint64_t>(ashtech_sec))) * 1000000;

			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
			// and control its drift. Since we rely on the HRT for our monotonic
			// clock, updating it from time to time is safe.

			timespec ts;
			ts.tv_sec = epoch;
			ts.tv_nsec = usecs * 1000;

			setClock(ts);

			_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_gps_position->time_utc_usec += usecs;

		} else {
			_gps_position->time_utc_usec = 0;
		}

#else
		_gps_position->time_utc_usec = 0;
#endif

		_last_timestamp_time = gps_absolute_time();
		ret = 8;
	}


	else if ((memcmp(_rx_buffer, "#BESTVELA,", 9) == 0) ) {
		#if DEBUG_INFO
		PX4_INFO("BESTVELA handle");
		#endif	
		int j = 0;
		uiCalcComma = 0;
		double velocityGnd __attribute__((unused)) = 0.0;
		double courseGnd __attribute__((unused)) = 0.0;
		double velocityVert __attribute__((unused)) = 0.0;
		double heading __attribute__((unused)) = 0.0;
		while(_rx_buffer[j++] != '*');
		while(uiCalcComma != 4)
		{
			if(_rx_buffer[j--] == ',')
			{
				uiCalcComma++;
			}
				
		}
 		bufptr = (char *)(_rx_buffer + j + 1);


 		if (bufptr && *(++bufptr) != ',') { velocityGnd = strtod(bufptr, &endp); bufptr = endp; }//UTC时间
		if (bufptr && *(++bufptr) != ',') { courseGnd = strtod(bufptr, &endp); bufptr = endp; }//纬度
		if (bufptr && *(++bufptr) != ',') { velocityVert = strtod(bufptr, &endp); bufptr = endp; }//经度

		//if((fabs(velocityGnd)<=0.0) || (fabs(courseGnd)<=0.0) || (fabs(velocityVert)<=0.0))
		//{
		//	_gps_position->vel_m_s = _gps_position_last->vel_m_s;       
		//	_gps_position->vel_n_m_s = _gps_position_last->vel_n_m_s;
		//	_gps_position->vel_e_m_s = _gps_position_last->vel_e_m_s;
		//	_gps_position->vel_d_m_s = _gps_position_last->vel_d_m_s;
		//	_gps_position->cog_rad = _gps_position_last->cog_rad;   //"#BESTVELA,"
		//}
		//else
		//{
			if(courseGnd > 180)
				courseGnd = courseGnd - 360;
			else if(courseGnd < -180)
				courseGnd = courseGnd + 360;
       		 heading = math::radians(courseGnd);

			_gps_position->vel_m_s = velocityGnd;       
			_gps_position->vel_n_m_s = velocityGnd * cos(heading);
			_gps_position->vel_e_m_s = velocityGnd * sin(heading);
			_gps_position->vel_d_m_s = -velocityVert;
			_gps_position->cog_rad = heading;   //"#BESTVELA,"

		//	_gps_position_last->vel_m_s = _gps_position->vel_m_s;       
		//	_gps_position_last->vel_n_m_s = _gps_position->vel_n_m_s;
		//	_gps_position_last->vel_e_m_s = _gps_position->vel_e_m_s;
		//	_gps_position_last->vel_d_m_s = _gps_position->vel_d_m_s;
		//	_gps_position_last->cog_rad = _gps_position->cog_rad;   //"#BESTVELA,"
		//}
		                        
		

    	_gps_position->vel_n_m_s = velocityGnd * cos(heading);
        _gps_position->vel_e_m_s = velocityGnd * sin(heading);
		_gps_position->vel_d_m_s = -velocityVert;
		_gps_position->cog_rad = heading;   //"#BESTVELA,"

		_gps_position->vel_ned_valid = true;				/** Flag to indicate if NED speed is valid */
		_gps_position->c_variance_rad = 0.1f;
		_rate_count_vel++;
		_gps_position->timestamp = gps_absolute_time();
		ret = 16;

	} 

	if (ret > 0) {
		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	}

	return ret;
}


int GPSDriverAshtech::receive(unsigned timeout)
{
	{

		uint8_t buf[GPS_READ_BUFFER_SIZE];

		/* timeout additional to poll */
		uint64_t time_started = gps_absolute_time();

		int j = 0;
		ssize_t bytes_count = 0;
		//int retTemp=-1;

		uint8_t flagRev = 0;
		

		while (true) {

			/* pass received bytes to the packet decoder */
			while (j < bytes_count) {
				int l = 0;
				int flagRevTemp = 0;
				if ((l = parseChar(buf[j])) > 0) {
					if ((flagRevTemp = handleMessage(l)) > 0) {
						flagRev = flagRev | flagRevTemp;
						if((flagRev & 0x1F) == 0X1F)
						{
							return 1;
						}
						//else
							//return flagRevTemp;
							
					}
					else {
						int rr=0;
						for(rr=0;rr<j;rr++)
							printf("%c",buf[rr]);
						PX4_INFO("handleMessage err");
						if( (j == bytes_count) && (bytes_count != 0) )
						{
							PX4_INFO("(j == bytes_count retTemp");
						}
					}
				}

				j++;
			}

			/* everything is read */
			j = bytes_count = 0;

			/* then poll or read for new data */
			//int ret = read(buf, sizeof(buf), timeout * 2);

			int ret = read(buf, sizeof(buf),timeout * 2);

			if (ret < 0) {
				PX4_INFO("ret = read err");
				/* something went wrong when polling */
				return -1;

			} else if (ret == 0) {
				/* Timeout while polling or just nothing read if reading, let's
				 * stay here, and use timeout below. */

			} else if (ret > 0) {
				/* if we have new data from GPS, go handle it */
				bytes_count = ret;
				//PX4_INFO("bytes_count is %d",bytes_count);
			}

			/* in case we get crap from GPS or time out */
			if (time_started + timeout * 1000 * 2 < gps_absolute_time()) {
				return -1;
			}
		}
	}

}
#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

#define HEXDIGIT_char(d) ((char)((d) + (((d) < 0xA) ? '0' : 'a'-0xA)))

/**crc32   start*/
#define CRC32_POLYNOMIAL   0xEDB88320L

uint32_t GPSDriverAshtech::CRC32Value(uint32_t i)
{
    uint32_t j;
    uint32_t ulCRC;
	ulCRC = i;
    for ( j = 8 ; j > 0; j-- )
    {
        if ( ulCRC & 1 )
        	ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
        	ulCRC >>= 1;
    }
    return ulCRC;
}


uint32_t GPSDriverAshtech::CalculateBlockCRC32(
	uint32_t ulCount,     /* Number of bytes in the data block */
	unsigned char *ucBuffer ) /* Data block */
	{
    uint32_t ulTemp1;
    uint32_t ulTemp2;
    uint32_t ulCRC = 0;
    while ( ulCount-- != 0 )
    {
    	ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
    	ulTemp2 = CRC32Value( ((uint32_t) ulCRC ^ *ucBuffer++ ) & 0xff );
    	ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}
/**crc32 end*/


int GPSDriverAshtech::parseChar(uint8_t b)
{
	int iRet = 0;

	switch (_decode_state) {
	/* First, look for sync1 */
	case NME_DECODE_UNINIT:
		if (b == '$') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;
			_rx_buffer[_rx_buffer_bytes++] = b;
		}
		else if (b == '#') {
			_decode_state = NME_DECODE_GOT_SYNC2;
			_rx_buffer_bytes = 0;
			_rx_buffer[_rx_buffer_bytes++] = b;
		}

		break;

	case NME_DECODE_GOT_SYNC1:
		if (b == '$') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;

		} else if (b == '*') {
			_decode_state = NME_DECODE_GOT_ASTERIKS;
		}

		if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
			_decode_state = NME_DECODE_UNINIT;
			_rx_buffer_bytes = 0;

		} else {
			_rx_buffer[_rx_buffer_bytes++] = b;
		}

		break;

	

	case NME_DECODE_GOT_ASTERIKS:
		_rx_buffer[_rx_buffer_bytes++] = b;
		_decode_state = NME_DECODE_GOT_FIRST_CS_BYTE;
		break;

	case NME_DECODE_GOT_FIRST_CS_BYTE:
		{
			_rx_buffer[_rx_buffer_bytes++] = b;
			uint8_t checksum = 0;
			uint8_t *buffer = _rx_buffer + 1;
			uint8_t *bufend = _rx_buffer + _rx_buffer_bytes - 3;

			for (; buffer < bufend; buffer++) { checksum ^= *buffer; }

			if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
			    (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
				iRet = _rx_buffer_bytes;
			}
			else
			{
				PX4_INFO("%d $gp crc err",gps_absolute_time());
			}

			_decode_state = NME_DECODE_UNINIT;
			_rx_buffer_bytes = 0;
			break;
		}
	case NME_DECODE_GOT_SYNC2:
		if (b == '#') {
			_decode_state = NME_DECODE_GOT_SYNC2;
			_rx_buffer_bytes = 0;

		} else if (b == '*') {
			_decode_state = NME_DECODE_GOT_ASTERIKS2;
		}

		if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 11)) {
			_decode_state = NME_DECODE_UNINIT;
			_rx_buffer_bytes = 0;

		} else {
			_rx_buffer[_rx_buffer_bytes++] = b;
		}
		break;

	case NME_DECODE_GOT_ASTERIKS2:
		_rx_buffer[_rx_buffer_bytes++] = b;
		crc32Count++;
		if(crc32Count == 7)
		{
			_decode_state = NME_DECODE_GOT_FIRST_CS_BYTE2;
			crc32Count = 0;			
		}
		break;

	case NME_DECODE_GOT_FIRST_CS_BYTE2:

		_rx_buffer[_rx_buffer_bytes++] = b;
		buffercrc = _rx_buffer + 1;
		bufendcrc = _rx_buffer + _rx_buffer_bytes - 9;

		uint8_t crctemp[4];
		uint8_t crc[8];
		uint8_t crcraw[8];


		crcraw[0] = *(bufendcrc + 1);
		crcraw[1] = *(bufendcrc + 2);
		crcraw[2] = *(bufendcrc + 3);
		crcraw[3] = *(bufendcrc + 4);
		crcraw[4] = *(bufendcrc + 5);
		crcraw[5] = *(bufendcrc + 6);
		crcraw[6] = *(bufendcrc + 7);
		crcraw[7] = *(bufendcrc + 8);

		
		crc32 = CalculateBlockCRC32((_rx_buffer_bytes-10),buffercrc);


		crctemp[0] = crc32;
		crctemp[1] = crc32>>8;
		crctemp[2] = crc32>>16;
		crctemp[3] = crc32>>24;


		crc[0] = HEXDIGIT_char(crctemp[3]>>4);
		crc[1] = HEXDIGIT_char(crctemp[3]& 0x0F);
		crc[2] = HEXDIGIT_char(crctemp[2]>>4);
		crc[3] = HEXDIGIT_char(crctemp[2]& 0x0F);
		crc[4] = HEXDIGIT_char(crctemp[1]>>4);
		crc[5] = HEXDIGIT_char(crctemp[1]& 0x0F);
		crc[6] = HEXDIGIT_char(crctemp[0]>>4);
		crc[7] = HEXDIGIT_char(crctemp[0]& 0x0F);


		if( (crc[0]==crcraw[0]) && (crc[1]==crcraw[1]) && (crc[2]==crcraw[2]) && (crc[3]==crcraw[3]) && (crc[4]==crcraw[4]) && (crc[5]==crcraw[5]) && (crc[6]==crcraw[6]) && (crc[7]==crcraw[7]))
		{

			iRet = _rx_buffer_bytes;

		}
		else
		{	
			PX4_INFO("%d #bestvela crc err",gps_absolute_time());
		}
		_decode_state = NME_DECODE_UNINIT;
		_rx_buffer_bytes = 0;

		break;
		default:			
		break;
	}

	return iRet;
}

void GPSDriverAshtech::decodeInit(void)
{

}


#if 0
const char comm[] = "unlogall\r\n"\
                    "com com2 115200 n 8 1 n off on\r\n"\
                    "com com1 115200 n 8 1 n off on\r\n"\
                    "interfacemode com2 rtcmv3 none off\r\n"\
                    "BESTVELTYPE doppler\r\n"\
                    "log com1 bestvela ontime 0.2\r\n"\
                    "log com1 gpggalong ontime 0.2\r\n"\
                    "log com1 gpgsa ontime 0.2\r\n"\
                    "log com1 gpzda ontime 0.2\r\n"\
                    "log com1 gpgst ontime 0.2\r\n"\
                    "Saveconfig\r\n";
#endif

int GPSDriverAshtech::configure(unsigned &baudrate, OutputMode output_mode)
{
	if (output_mode != OutputMode::GPS) {
		GPS_WARN("ASHTECH: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}
	baudrate = 115200;
	return setBaudrate(baudrate);
}
