#include "../include/rcraicer_gps/ubx_protocol.h"
#include <iostream>
#include <cstring>

UbxProtocol::UbxProtocol(bool msgDebug, std::string portPath, int baudRate, bool baseStation) : serialPort(NULL)
{
    this->msgDebug = msgDebug;
    this->portPath = portPath;
    this->baudRate = baudRate;
    this->baseStation = baseStation;

    configured = false;
    use_nav_pvt = true;
}

UbxProtocol::~UbxProtocol()
{
    if (serialPort != NULL)
        delete serialPort;
}

bool UbxProtocol::connect()
{
    serialPort = new SerialPort(portPath, baudRate);
    serialPort->registerDataCallback(std::bind(&UbxProtocol::serial_data_callback, this, std::placeholders::_1));    

    if (serialPort->isConnected())
    {
        return true;
    }

    return false;    
}

void UbxProtocol::serial_data_callback(const uint8_t data)
{
    int ret = parseChar(data);
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
            outputMsgType();
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

        case UBX_MSG_ACK_ACK:
            break;
        
        case UBX_MSG_ACK_NAK:
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

        case UBX_MSG_NAV_SVIN:
            if (rx_payload_length != sizeof(ubx_payload_rx_nav_svin_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;

            } else if (!configured) {
                rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

            }

            break;
        
        case UBX_MSG_NAV_COV:
            if (rx_payload_length != sizeof(ubx_payload_rx_nav_cov_t)) {
                rx_state = UBX_RXMSG_ERROR_LENGTH;
            }else if (!configured) {
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
        default:
            rx_state = UBX_RXMSG_DISABLE;	// disable all other messages
            break;
    }

    switch (rx_state) {
	case UBX_RXMSG_HANDLE:	// handle message
	case UBX_RXMSG_IGNORE:	// ignore message but don't report error
		ret = 0;
		break;

	case UBX_RXMSG_DISABLE:	// disable unexpected messages
		ret = -1;	// return error, abort handling this message
		break;

	case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
		ret = -1;	// return error, abort handling this message
		break;

	default:	// invalid message state
		ret = -1;	// return error, abort handling this message
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

void UbxProtocol::outputMsgType()
{
    if (!msgDebug)
        return;

    uint8_t mt1, mt2;

    mt2 = rx_msg >> 8;
    mt1 = rx_msg;    

    switch(rx_msg)
    {
        case UBX_MSG_ACK_NAK:
            std::cout <<"UBX_MSG_ACK_NAK" << "\r\n";
            break;            

        case UBX_MSG_ACK_ACK:
            std::cout <<"UBX_MSG_ACK_ACK" << "\r\n";
            break;            

        case UBX_MSG_NAV_SIG:
            std::cout <<"UBX_MSG_NAV_SIG" << "\r\n";
            break;            

        case UBX_MSG_NAV_COV:
            std::cout <<"UBX_MSG_NAV_COV" << "\r\n";
            break;           

        case UBX_MSG_NAV_VELNED:
            std::cout <<"UBX_MSG_NAV_VELNED" << "\r\n";
            break;           

        case UBX_MSG_NAV_SVIN:
            std::cout <<"UBX_MSG_NAV_SVIN" << "\r\n";
            break;            
        
        case UBX_MSG_NAV_STATUS:
            std::cout <<"UBX_MSG_NAV_STATUS" << "\r\n";
            break;            

        case UBX_MSG_NAV_DOP:
            std::cout <<"UBX_MSG_NAV_DOP" << "\r\n";
            break;            

        case UBX_MSG_NAV_POSLLH:
            std::cout <<"UBX_MSG_NAV_POSLLH" << "\r\n";
            break;            

        case UBX_MSG_NAV_PVT:
            std::cout <<"UBX_MSG_NAV_PVT" << "\r\n";
            break;            

        case UBX_MSG_NAV_SAT:
            std::cout <<"UBX_MSG_NAV_SAT" << "\r\n";
            break;            
        
        case UBX_MSG_NAV_TIMEUTC:
            std::cout <<"UBX_MSG_NAV_TIMEUTC" << "\r\n";
            break;            

        case UBX_MSG_NAV_RELPOSNED:
            std::cout <<"UBX_MSG_NAV_RELPOSNED" << "\r\n";
            break;            
        
        case UBX_MSG_MON_RF:
            std::cout <<"UBX_MSG_MON_RF" << "\r\n";
            break;            
        
        default:
             std::cout << "Message type: " << (int)mt1 << " : " << (int)mt2 << "\r\n";
             break;        
    }
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
        case UBX_MSG_ACK_NAK:
            acknowledged = false;
            isAckNacReady = true;
            configurationCallback();
            break;

        case UBX_MSG_ACK_ACK:
            acknowledged = true;
            isAckNacReady = true;
            configurationCallback();
            break;
            
        case UBX_MSG_NAV_SVIN:
            gpsSurveyMsg.mean_x = buf.payload_rx_nav_svin.meanX;
            gpsSurveyMsg.mean_y = buf.payload_rx_nav_svin.meanY;
            gpsSurveyMsg.mean_z = buf.payload_rx_nav_svin.meanZ;

            gpsSurveyMsg.observations = buf.payload_rx_nav_svin.obs;
            gpsSurveyMsg.duration = buf.payload_rx_nav_svin.dur;

            gpsSurveyMsg.accuracy = (float) buf.payload_rx_nav_svin.meanAcc / 10000; // convert to metres

            gpsSurveyMsg.active = buf.payload_rx_nav_svin.active;
            gpsSurveyMsg.valid = buf.payload_rx_nav_svin.valid;
            
            isGpsSurveyMsgReady = true;
            messageCallback();

            break;
        case UBX_MSG_NAV_DOP:
            gpsStatusMsg.hdop = (float)buf.payload_rx_nav_dop.hDOP / 100.0;
            gpsStatusMsg.vdop = (float)buf.payload_rx_nav_dop.vDOP / 100.0;
            gpsStatusMsg.pdop = (float)buf.payload_rx_nav_dop.pDOP / 100.0;

            break;

        case UBX_MSG_NAV_COV:
            fixMsg.position_covariance_type =  sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

            if ((buf.payload_rx_nav_cov.posCorValid == 1))
            {
                fixMsg.position_covariance_type =  sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;

                // convert from NED to ENU
                fixMsg.position_covariance.at(0) = buf.payload_rx_nav_cov.posCovEE;
                fixMsg.position_covariance.at(1) = buf.payload_rx_nav_cov.posCovNE;
                fixMsg.position_covariance.at(2) = buf.payload_rx_nav_cov.posCovED;

                fixMsg.position_covariance.at(3) = buf.payload_rx_nav_cov.posCovNE;
                fixMsg.position_covariance.at(4) = buf.payload_rx_nav_cov.posCovNN;
                fixMsg.position_covariance.at(5) = buf.payload_rx_nav_cov.posCovND;

                fixMsg.position_covariance.at(6) = buf.payload_rx_nav_cov.posCovED;
                fixMsg.position_covariance.at(7) = buf.payload_rx_nav_cov.posCovND;
                fixMsg.position_covariance.at(8) = buf.payload_rx_nav_cov.posCovDD;

            }
            break;
        case UBX_MSG_NAV_PVT:
            
            gpsStatusMsg.rtk_status = rcraicer_msgs::msg::GPSStatus::RTK_STATUS_NONE;

            //Check if position fix flag is good
            if ((buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1) {                
                gpsStatusMsg.status = rcraicer_msgs::msg::GPSStatus::STATUS_FIX;
                fixMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

                if (buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN) {                    
                    gpsStatusMsg.status = rcraicer_msgs::msg::GPSStatus::STATUS_DGPS_FIX;
                    fixMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                }

                uint8_t carr_soln = buf.payload_rx_nav_pvt.flags >> 6;

                if (carr_soln == 1) {
                    fixMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    gpsStatusMsg.rtk_status = rcraicer_msgs::msg::GPSStatus::RTK_STATUS_FLOAT;
                    // _gps_position->fix_type = 5; //Float RTK

                } else if (carr_soln == 2) {
                    fixMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    gpsStatusMsg.rtk_status = rcraicer_msgs::msg::GPSStatus::RTK_STATUS_FIXED;
                    // _gps_position->fix_type = 6; //Fixed RTK
                }

                // _gps_position->vel_ned_valid = true;

            } else {
                gpsStatusMsg.status = rcraicer_msgs::msg::GPSStatus::STATUS_NO_FIX;
                fixMsg.status.status = rcraicer_msgs::msg::GPSStatus::STATUS_NO_FIX;                
            }
            

		    fixMsg.latitude	= buf.payload_rx_nav_pvt.lat / 1e7; // convert  to degrees
		    fixMsg.longitude = buf.payload_rx_nav_pvt.lon  / 1e7; // copnvert to degrees
		    fixMsg.altitude = buf.payload_rx_nav_pvt.height /1000; // convert to metres from mm

            isGpsStatusMsgReady = true;
            isNavSatFixMsgReady = true;
            messageCallback();

            break;

        case UBX_MSG_INF_DEBUG:
        case UBX_MSG_INF_NOTICE: {
                uint8_t *p_buf = (uint8_t *)&buf;
                p_buf[rx_payload_length] = 0;
                debugMessage = reinterpret_cast<char *>(p_buf);
                isDebugMessageReady = true;
            }
            break;

        case UBX_MSG_INF_ERROR:
        case UBX_MSG_INF_WARNING: {
                uint8_t *p_buf = (uint8_t *)&buf;
                p_buf[rx_payload_length] = 0;
                warningMessage = reinterpret_cast<char *>(p_buf);
                isWarningMessageReady = true;
            }
            break;

        case UBX_MSG_NAV_TIMEUTC:
            // std::cout << "UBX_MSG_NAV_PVT\r\n";
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
    rx_ck_a = 0;
	rx_ck_b = 0;
	rx_payload_length = 0;
	rx_payload_index = 0;
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

bool UbxProtocol::gpsStatusMessageReady()
{
    return isGpsStatusMsgReady;
}

bool UbxProtocol::gpsSurveyMessageReady()
{
    return isGpsSurveyMsgReady;
}

bool UbxProtocol::navSatStatusMessageReady()
{
    return isNavSatStatusMsgReady;
}

bool UbxProtocol::navSatFixMessageReady()
{
    return isNavSatFixMsgReady;
}

bool UbxProtocol::ackNackMessageReady()
{
    return isAckNacReady;
}

rcraicer_msgs::msg::GPSStatus UbxProtocol::getGpsStatusMessage()
{        
    isGpsStatusMsgReady = false;
    return gpsStatusMsg;
}

rcraicer_msgs::msg::GPSSurvey UbxProtocol::getGpsSurveyMessage()
{        
    isGpsSurveyMsgReady = false;
    return gpsSurveyMsg;
}

sensor_msgs::msg::NavSatFix UbxProtocol::getNavSatFixMessage()
{    
    isNavSatFixMsgReady = false;
    return fixMsg;
}

sensor_msgs::msg::NavSatStatus UbxProtocol::getNavSatStatusMessage()
{    
    isNavSatStatusMsgReady = false;
    return statusMsg;
}

bool UbxProtocol::getAckStatus()
{
    return acknowledged;
}

void UbxProtocol::configure()
{    
    // configured = true;
    int cfg_valset_msg_size = initCfgValset();    

    // disable NMEA output on USB
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_NMEA, 0, cfg_valset_msg_size);        

    // set measurement rate to 10hz
    cfgValset<uint16_t>(UBX_CFG_KEY_RATE_MEAS, 100, cfg_valset_msg_size);    

    // set nav rate to 5hz
    cfgValset<uint16_t>(UBX_CFG_KEY_RATE_NAV, 2, cfg_valset_msg_size);    

    // set inf messages
    cfgValset<uint8_t>(UBX_CFG_KEY_INFMSG_UBX_USB, UBX_CFG_INF_MSG_ERROR_WARN_NOTICE, cfg_valset_msg_size); 
    

    bool ret = sendMessage(UBX_MSG_CFG_VALSET, (uint8_t*)&txbuf, cfg_valset_msg_size);        

    configured = true;
}

void UbxProtocol::configureSurveyIn(uint32_t minDuration, uint32_t accLimit)
{
    int cfg_valset_msg_size = initCfgValset();    

    // set min duration (seconds)
    cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_SVIN_MIN_DUR, minDuration, cfg_valset_msg_size);    

    // set acc limit (millimeters)
    cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_SVIN_ACC_LIMIT, accLimit, cfg_valset_msg_size);    

    bool ret = sendMessage(UBX_MSG_CFG_VALSET, (uint8_t*)&txbuf, cfg_valset_msg_size);        
}

int UbxProtocol::initCfgValset()
{
	memset(&txbuf.payload_tx_cfg_valset, 0, sizeof(txbuf.payload_tx_cfg_valset));
	txbuf.payload_tx_cfg_valset.layers = UBX_CFG_LAYER_RAM;    
	return sizeof(txbuf.payload_tx_cfg_valset) - sizeof(txbuf.payload_tx_cfg_valset.cfgData);
}

template<typename T>
bool UbxProtocol::cfgValset(uint32_t key_id, T value, int &msg_size)
{
	if (msg_size + sizeof(key_id) + sizeof(value) > sizeof(buf)) {
		// If this happens use several CFG-VALSET messages instead of one		
		return false;
	}

	uint8_t *buffer = (uint8_t *)&txbuf.payload_tx_cfg_valset;
	memcpy(buffer + msg_size, &key_id, sizeof(key_id));
	msg_size += sizeof(key_id);
	memcpy(buffer + msg_size, &value, sizeof(value));
	msg_size += sizeof(value);
	return true;
}

bool UbxProtocol::sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
    ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2, 0, 0};
	ubx_checksum_t checksum = {0, 0};

	// Populate header
	header.msg	= msg;
	header.length	= length;

	// Calculate checksum
	calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

	if (payload != nullptr) {
		calcChecksum(payload, length, &checksum);
	}

	// Send message
	if (serialPort->writePort((uint8_t *)&header, sizeof(header)) != sizeof(header)) {
		return false;
	}

	if (payload && serialPort->writePort((uint8_t *)payload, length) != length) {
		return false;
	}

	if (serialPort->writePort((uint8_t *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
		return false;
	}

	return true;
}

void UbxProtocol::calcChecksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
{
	for (uint16_t i = 0; i < length; i++) {
		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}
}


void UbxProtocol::registerMessageCallback(MessageCallback callback)
{
    messageCallback = callback;
}

void UbxProtocol::registerConfigurationCallback(ConfigurationCallback callback)
{
    configurationCallback = callback;
}

std::string UbxProtocol::getDebugMessage()
{
    isDebugMessageReady = false;
    return debugMessage;
}

std::string UbxProtocol::getWarningMessage()
{
    isWarningMessageReady = false;
    return warningMessage;
}

bool UbxProtocol::debugMessageReady()
{
    return isDebugMessageReady;
}

bool UbxProtocol::warningMessageReady()
{
    return isWarningMessageReady;
}
