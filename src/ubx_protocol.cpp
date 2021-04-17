#include "../include/rcraicer_gps/ubx_protocol.h"

UbxProtocol::UbxProtocol()
{

}

UbxProtocol::~UbxProtocol()
{

}

int UbxProtocol::parseChar(const uint8_t b)
{
    int ret = 0;

    switch(decode_state)
    {
        case UBX_DECODE_SYNC1:
            if (b == UBX_SYNC1){
                decode_state = UBX_DECODE_SYNC2;
            }
            break;
        case UBX_DECODE_SYNC2:
            if (b == UBX_SYNC2)
            {
                decode_state = UBX_DECODE_CLASS;
            } else
            {
                // reset parser
                decodeInit();
            }
            break;
        case UBX_DECODE_CLASS:
            addByteToChecksum(b);
            rx_msg = b;
            decode_state = UBX_DECODE_ID;            

            break;
        case UBX_DECODE_ID:
            addByteToChecksum(b);
            rx_msg |= b << 8;
            decode_state = UBX_DECODE_LENGTH1;
            break;
        case UBX_DECODE_LENGTH1:
            addByteToChecksum(b);
            rx_payload_length = b;
            decode_state = UBX_DECODE_LENGTH2;
            break;
        case UBX_DECODE_LENGTH2:
            addByteToChecksum(b);
            rx_payload_length |= b << 8;
            
            if (payloadRxInit() != 0){
                decodeInit(); // discard message, payload will be ignored
            } else
            {
                decode_state = (rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
            }
            break;
        case UBX_DECODE_PAYLOAD:
            addByteToChecksum(b);
            switch(rx_msg)
            {
                case UBX_MSG_NAV_SAT:
                    ret = payloadRxAddNavSat(b);
                    break;
                
                default:
			        ret = payloadRxAdd(b);		// add a payload byte
			        break;            
            }

            if (ret < 0) {
                decodeInit();
            } 
            else if (ret > 0)
            {
                decode_state = UBX_DECODE_CHKSUM1;
            }
            
            ret = 0;
            break;
        case UBX_DECODE_CHKSUM1:
            if (rx_ck_a != b) {  
                decodeInit();

            } else {
                decode_state = UBX_DECODE_CHKSUM2;
            }

            break;

        /* Expecting second checksum byte */
        case UBX_DECODE_CHKSUM2:
            if (rx_ck_b == b) {                            
                ret = payloadRxDone();	// finish payload processing
            }

            decodeInit();
            break;        
    }

    return ret;
}

int UbxProtocol::payloadRxInit()  // -1 = abort, 0 = continue
{
    int ret = 0;

    rx_state = UBX_RXMSG_HANDLE;	// handle by default

    switch(rx_msg)
    {
        case UBX_MSG_NAV_SAT:
            // clear data from GPS Status message in preparation for new data
            clearGPSStatusMsg();
            break;

        case UBX_MSG_NAV_PVT:
            if ((rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7)		/* u-blox 7 msg format */
                && (rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8)) {	/* u-blox 8+ msg format */
                rx_state = UBX_RXMSG_ERROR_LENGTH;

            } else if (!configured) {
                rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

            } else if (!use_nav_pvt) {
                rx_state = UBX_RXMSG_DISABLE;        // disable if not using NAV-PVT
            }

            break;
        case UBX_MSG_INF_DEBUG:
        case UBX_MSG_INF_ERROR:
        case UBX_MSG_INF_NOTICE:
        case UBX_MSG_INF_WARNING:
            if (rx_payload_length >= sizeof(ubx_buf_t)) {
                rx_payload_length = sizeof(ubx_buf_t) - 1; //avoid buffer overflow
            }

            break;
        case UBX_MSG_NAV_POSLLH:
            if (rx_payload_length != sizeof(ubx_payload_rx_nav_posllh_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;

            } else if (!configured) {
                rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

            } else if (use_nav_pvt) {
                rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
            }

            break;
        case UBX_MSG_NAV_SOL:
            if (rx_payload_length != sizeof(ubx_payload_rx_nav_sol_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;

            } else if (!configured) {
                rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

            } else if (use_nav_pvt) {
                rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
            }

            break;
        case UBX_MSG_NAV_DOP:
            if (rx_payload_length != sizeof(ubx_payload_rx_nav_dop_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;

            } else if (!configured) {
                rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

            }

            break;

        case UBX_MSG_NAV_RELPOSNED:
            if (rx_payload_length != sizeof(ubx_payload_rx_nav_relposned_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;

            } else if (!configured) {
                rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

            }

            break;

        case UBX_MSG_NAV_TIMEUTC:
            if (rx_payload_length != sizeof(ubx_payload_rx_nav_timeutc_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;

            } else if (!configured) {
                rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

            } else if (use_nav_pvt) {
                rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
            }

            break;


    }
    return ret;
}

int UbxProtocol::payloadRxAdd(const uint8_t b)
{
    int ret = 0;
	uint8_t *p_buf = (uint8_t *)&buf;

	p_buf[rx_payload_index] = b;

	if (++rx_payload_index >= rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

int UbxProtocol::payloadRxAddMonVer(const uint8_t b)
{

}

int UbxProtocol::payloadRxAddNavSat(const uint8_t b)
{
    int ret = 0;
	uint8_t *p_buf = (uint8_t *)&buf;

	if (rx_payload_index < sizeof(ubx_payload_rx_nav_sat_part1_t)) {
		// Fill Part 1 buffer
		p_buf[rx_payload_index] = b;

	} else {
		if (rx_payload_index == sizeof(ubx_payload_rx_nav_sat_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
            gpsStatusMsg.satellites_visible = buf.payload_rx_nav_sat_part1.numSvs;			
		}

		if (rx_payload_index < sizeof(ubx_payload_rx_nav_sat_part1_t) + gpsStatusMsg.satellites_visible * sizeof(
			    ubx_payload_rx_nav_sat_part2_t)) {
			// Still room in _satellite_info: fill Part 2 buffer
			unsigned buf_index = (rx_payload_index - sizeof(ubx_payload_rx_nav_sat_part1_t)) % sizeof(
						     ubx_payload_rx_nav_sat_part2_t);
			p_buf[buf_index] = b;

			if (buf_index == sizeof(ubx_payload_rx_nav_sat_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index = (rx_payload_index - sizeof(ubx_payload_rx_nav_sat_part1_t)) /
						     sizeof(ubx_payload_rx_nav_sat_part2_t);

				// convert gnssId:svId to a 8 bit number (use svId numbering from NAV-SVINFO)
				uint8_t ubx_sat_gnssId = static_cast<uint8_t>(buf.payload_rx_nav_sat_part2.gnssId);
				uint8_t ubx_sat_svId = static_cast<uint8_t>(buf.payload_rx_nav_sat_part2.svId);

				uint8_t svinfo_svid = 255;

				switch (ubx_sat_gnssId) {
				case 0:  // GPS: G1-G23 -> 1-32
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 32) {
						svinfo_svid = ubx_sat_svId;
					}

					break;

				case 1:  // SBAS: S120-S158 -> 120-158
					if (ubx_sat_svId >= 120 && ubx_sat_svId <= 158) {
						svinfo_svid = ubx_sat_svId;
					}

					break;

				case 2:  // Galileo: E1-E36 -> 211-246
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 36) {
						svinfo_svid = ubx_sat_svId + 210;
					}

					break;

				case 3:  // BeiDou: B1-B37 -> 159-163,33-64
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 4) {
						svinfo_svid = ubx_sat_svId + 158;

					} else if (ubx_sat_svId >= 5 && ubx_sat_svId <= 37) {
						svinfo_svid = ubx_sat_svId + 28;
					}

					break;

				case 4:  // IMES: I1-I10 -> 173-182
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 10) {
						svinfo_svid = ubx_sat_svId + 172;
					}

					break;

				case 5:  // QZSS: Q1-A10 -> 193-202
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 10) {
						svinfo_svid = ubx_sat_svId + 192;
					}

					break;

				case 6:  // GLONASS: R1-R32 -> 65-96, R? -> 255
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 32) {
						svinfo_svid = ubx_sat_svId + 64;
					}

					break;
				}

				// gpsStatusMsg->svid[sat_index]	  = svinfo_svid;
                uint8_t used = buf.payload_rx_nav_sat_part2.flags & 0x01;
                int32_t svid = static_cast<int32_t>(svinfo_svid);

                if (used)
                {
                    gpsStatusMsg.satellites_used += 1;
                    gpsStatusMsg.satellite_used_prn.push_back(svid);
                }

				gpsStatusMsg.satellites_visible++;
				gpsStatusMsg.satellite_visible_z.push_back(static_cast<int32_t>(buf.payload_rx_nav_sat_part2.elev));
				gpsStatusMsg.satellite_visible_azimuth.push_back(static_cast<int32_t>(static_cast<float>(buf.payload_rx_nav_sat_part2.azim) *
						255.0f / 360.0f));
				gpsStatusMsg.satellite_visible_snr.push_back(static_cast<int32_t>(buf.payload_rx_nav_sat_part2.cno));
				gpsStatusMsg.satellite_visible_prn.push_back(svid);				
			}
		}
	}

	if (++rx_payload_index >= rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

int UbxProtocol::payloadRxDone()
{
    int ret = 0;

	// return if no message handled
	if (rx_state != UBX_RXMSG_HANDLE) {
		return ret;
	}

    switch(rx_msg)
    {
        case UBX_MSG_NAV_PVT:
            //Check if position fix flag is good
            if ((buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1) {                
                gpsStatusMsg.status = rcraicer_msgs::msg::GPSStatus::STATUS_FIX;

                if (buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN) {                    
                    gpsStatusMsg.status = rcraicer_msgs::msg::GPSStatus::STATUS_DGPS_FIX;
                }

                uint8_t carr_soln = buf.payload_rx_nav_pvt.flags >> 6;

                if (carr_soln == 1) {
                    // _gps_position->fix_type = 5; //Float RTK

                } else if (carr_soln == 2) {
                    // _gps_position->fix_type = 6; //Fixed RTK
                }

                // _gps_position->vel_ned_valid = true;

            } else {
                fixMsg.status.status = rcraicer_msgs::msg::GPSStatus::STATUS_NO_FIX;                
            }
            

		    fixMsg.latitude	= buf.payload_rx_nav_pvt.lat;
		    fixMsg.longitude = buf.payload_rx_nav_pvt.lon;
		    fixMsg.altitude = buf.payload_rx_nav_pvt.height;

            break;

    }
}

void UbxProtocol::addByteToChecksum(const uint8_t b)
{
    rx_ck_a = rx_ck_a + b;
	rx_ck_b = rx_ck_b + rx_ck_a;
}

void UbxProtocol::decodeInit()
{
    decode_state = UBX_DECODE_SYNC1;
}

void UbxProtocol::clearGPSStatusMsg()
{
    gpsStatusMsg.satellites_used = 0;
    gpsStatusMsg.satellite_used_prn.clear();
    gpsStatusMsg.satellites_visible = 0;
    gpsStatusMsg.satellite_visible_prn.clear();
    gpsStatusMsg.satellite_visible_z.clear();
    gpsStatusMsg.satellite_visible_azimuth.clear();
    gpsStatusMsg.satellite_visible_snr.clear();
}