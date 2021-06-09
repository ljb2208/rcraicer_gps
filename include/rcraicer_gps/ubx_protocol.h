# include <stdint.h>
#include <string>

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "rcraicer_msgs/msg/gps_status.hpp"
#include "rcraicer_msgs/msg/gps_survey.hpp"
#include "serial_port.h"
#include <functional>

#define UBX_SYNC1             0xB5
#define UBX_SYNC2             0x62

/* Message Classes */
#define UBX_CLASS_NAV         0x01
#define UBX_CLASS_RXM         0x02
#define UBX_CLASS_INF         0x04
#define UBX_CLASS_ACK         0x05
#define UBX_CLASS_CFG         0x06
#define UBX_CLASS_MON         0x0A
#define UBX_CLASS_RTCM3       0xF5

/* Message IDs */
#define UBX_ID_NAV_COV		  0x36
#define UBX_ID_NAV_STATUS	  0x03
#define UBX_ID_NAV_SIG		  0x43
#define UBX_ID_NAV_POSLLH     0x02
#define UBX_ID_NAV_DOP        0x04
#define UBX_ID_NAV_SOL        0x06
#define UBX_ID_NAV_PVT        0x07
#define UBX_ID_NAV_VELNED     0x12
#define UBX_ID_NAV_TIMEUTC    0x21
#define UBX_ID_NAV_SVINFO     0x30
#define UBX_ID_NAV_SAT        0x35
#define UBX_ID_NAV_SVIN       0x3B
#define UBX_ID_NAV_RELPOSNED  0x3C
#define UBX_ID_RXM_SFRBX      0x13
#define UBX_ID_RXM_RAWX       0x15
#define UBX_ID_INF_DEBUG      0x04
#define UBX_ID_INF_ERROR      0x00
#define UBX_ID_INF_NOTICE     0x02
#define UBX_ID_INF_WARNING    0x01
#define UBX_ID_ACK_NAK        0x00
#define UBX_ID_ACK_ACK        0x01
#define UBX_ID_CFG_PRT        0x00 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_MSG        0x01 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_RATE       0x08 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_CFG        0x09 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_NAV5       0x24 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_RST        0x04
#define UBX_ID_CFG_SBAS       0x16
#define UBX_ID_CFG_TMODE3     0x71 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_GNSS       0x3E
#define UBX_ID_CFG_VALSET     0x8A
#define UBX_ID_CFG_VALGET     0x8B
#define UBX_ID_CFG_VALDEL     0x8C
#define UBX_ID_MON_VER        0x04
#define UBX_ID_MON_HW         0x09 // deprecated in protocol version >= 27 -> use MON_RF
#define UBX_ID_MON_RF         0x38

/* UBX ID for RTCM3 output messages */
/* Minimal messages for RTK: 1005, 1077 + (1087 or 1127) */
/* Reduced message size using MSM4: 1005, 1074 + (1084 or 1124)  */
#define UBX_ID_RTCM3_1005     0x05    /**< Stationary RTK reference station ARP */
#define UBX_ID_RTCM3_1074     0x4A    /**< GPS MSM4 */
#define UBX_ID_RTCM3_1077     0x4D    /**< GPS MSM7 */
#define UBX_ID_RTCM3_1084     0x54    /**< GLONASS MSM4 */
#define UBX_ID_RTCM3_1087     0x57    /**< GLONASS MSM7 */
#define UBX_ID_RTCM3_1094     0x5E    /**< Galileo MSM4 */
#define UBX_ID_RTCM3_1097     0x61    /**< Galileo MSM7 */
#define UBX_ID_RTCM3_1124     0x7C    /**< BeiDou MSM4 */
#define UBX_ID_RTCM3_1127     0x7F    /**< BeiDou MSM7 */
#define UBX_ID_RTCM3_1230     0xE6    /**< GLONASS code-phase biases */
#define UBX_ID_RTCM3_4072     0xFE    /**< Reference station PVT (u-blox proprietary RTCM Message) - Used for moving baseline */


/* Message Classes & IDs */
#define UBX_MSG_NAV_COV 	  ((UBX_CLASS_NAV) | UBX_ID_NAV_COV << 8)
#define UBX_MSG_NAV_STATUS 	  ((UBX_CLASS_NAV) | UBX_ID_NAV_STATUS << 8)
#define UBX_MSG_NAV_SIG  	  ((UBX_CLASS_NAV) | UBX_ID_NAV_SIG << 8)
#define UBX_MSG_NAV_POSLLH    ((UBX_CLASS_NAV) | UBX_ID_NAV_POSLLH << 8)
#define UBX_MSG_NAV_SOL       ((UBX_CLASS_NAV) | UBX_ID_NAV_SOL << 8)
#define UBX_MSG_NAV_DOP       ((UBX_CLASS_NAV) | UBX_ID_NAV_DOP << 8)
#define UBX_MSG_NAV_PVT       ((UBX_CLASS_NAV) | UBX_ID_NAV_PVT << 8)
#define UBX_MSG_NAV_VELNED    ((UBX_CLASS_NAV) | UBX_ID_NAV_VELNED << 8)
#define UBX_MSG_NAV_TIMEUTC   ((UBX_CLASS_NAV) | UBX_ID_NAV_TIMEUTC << 8)
#define UBX_MSG_NAV_SVINFO    ((UBX_CLASS_NAV) | UBX_ID_NAV_SVINFO << 8)
#define UBX_MSG_NAV_SAT       ((UBX_CLASS_NAV) | UBX_ID_NAV_SAT << 8)
#define UBX_MSG_NAV_SVIN      ((UBX_CLASS_NAV) | UBX_ID_NAV_SVIN << 8)
#define UBX_MSG_NAV_RELPOSNED ((UBX_CLASS_NAV) | UBX_ID_NAV_RELPOSNED << 8)
#define UBX_MSG_RXM_SFRBX     ((UBX_CLASS_RXM) | UBX_ID_RXM_SFRBX << 8)
#define UBX_MSG_RXM_RAWX      ((UBX_CLASS_RXM) | UBX_ID_RXM_RAWX << 8)
#define UBX_MSG_INF_DEBUG     ((UBX_CLASS_INF) | UBX_ID_INF_DEBUG << 8)
#define UBX_MSG_INF_ERROR     ((UBX_CLASS_INF) | UBX_ID_INF_ERROR << 8)
#define UBX_MSG_INF_NOTICE    ((UBX_CLASS_INF) | UBX_ID_INF_NOTICE << 8)
#define UBX_MSG_INF_WARNING   ((UBX_CLASS_INF) | UBX_ID_INF_WARNING << 8)
#define UBX_MSG_ACK_NAK       ((UBX_CLASS_ACK) | UBX_ID_ACK_NAK << 8)
#define UBX_MSG_ACK_ACK       ((UBX_CLASS_ACK) | UBX_ID_ACK_ACK << 8)
#define UBX_MSG_CFG_PRT       ((UBX_CLASS_CFG) | UBX_ID_CFG_PRT << 8)
#define UBX_MSG_CFG_MSG       ((UBX_CLASS_CFG) | UBX_ID_CFG_MSG << 8)
#define UBX_MSG_CFG_RATE      ((UBX_CLASS_CFG) | UBX_ID_CFG_RATE << 8)
#define UBX_MSG_CFG_CFG       ((UBX_CLASS_CFG) | UBX_ID_CFG_CFG << 8)
#define UBX_MSG_CFG_NAV5      ((UBX_CLASS_CFG) | UBX_ID_CFG_NAV5 << 8)
#define UBX_MSG_CFG_RST       ((UBX_CLASS_CFG) | UBX_ID_CFG_RST << 8)
#define UBX_MSG_CFG_SBAS      ((UBX_CLASS_CFG) | UBX_ID_CFG_SBAS << 8)
#define UBX_MSG_CFG_TMODE3    ((UBX_CLASS_CFG) | UBX_ID_CFG_TMODE3 << 8)
#define UBX_MSG_CFG_GNSS      ((UBX_CLASS_CFG) | UBX_ID_CFG_GNSS << 8)
#define UBX_MSG_CFG_VALGET    ((UBX_CLASS_CFG) | UBX_ID_CFG_VALGET << 8)
#define UBX_MSG_CFG_VALSET    ((UBX_CLASS_CFG) | UBX_ID_CFG_VALSET << 8)
#define UBX_MSG_CFG_VALDEL    ((UBX_CLASS_CFG) | UBX_ID_CFG_VALDEL << 8)
#define UBX_MSG_MON_HW        ((UBX_CLASS_MON) | UBX_ID_MON_HW << 8)
#define UBX_MSG_MON_VER       ((UBX_CLASS_MON) | UBX_ID_MON_VER << 8)
#define UBX_MSG_MON_RF        ((UBX_CLASS_MON) | UBX_ID_MON_RF << 8)
#define UBX_MSG_RTCM3_1005    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1005 << 8)
#define UBX_MSG_RTCM3_1077    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1077 << 8)
#define UBX_MSG_RTCM3_1087    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1087 << 8)
#define UBX_MSG_RTCM3_1074    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1074 << 8)
#define UBX_MSG_RTCM3_1084    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1084 << 8)
#define UBX_MSG_RTCM3_1094    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1094 << 8)
#define UBX_MSG_RTCM3_1097    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1097 << 8)
#define UBX_MSG_RTCM3_1124    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1124 << 8)
#define UBX_MSG_RTCM3_1127    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1127 << 8)
#define UBX_MSG_RTCM3_1230    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1230 << 8)
#define UBX_MSG_RTCM3_4072    ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_4072 << 8)


#define UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7        (sizeof(ubx_payload_rx_nav_pvt_t) - 8)
#define UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8        (sizeof(ubx_payload_rx_nav_pvt_t))

/* RX NAV-PVT message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_PVT_VALID_VALIDDATE          0x01    /**< validDate (Valid UTC Date) */
#define UBX_RX_NAV_PVT_VALID_VALIDTIME          0x02    /**< validTime (Valid UTC Time) */
#define UBX_RX_NAV_PVT_VALID_FULLYRESOLVED      0x04    /**< fullyResolved (1 = UTC Time of Day has been fully resolved (no seconds uncertainty)) */

/*   Bitfield "flags" masks */
#define UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK          0x01    /**< gnssFixOK (A valid fix (i.e within DOP & accuracy masks)) */
#define UBX_RX_NAV_PVT_FLAGS_DIFFSOLN           0x02    /**< diffSoln (1 if differential corrections were applied) */
#define UBX_RX_NAV_PVT_FLAGS_PSMSTATE           0x1C    /**< psmState (Power Save Mode state (see Power Management)) */
#define UBX_RX_NAV_PVT_FLAGS_HEADVEHVALID       0x20    /**< headVehValid (Heading of vehicle is valid) */
#define UBX_RX_NAV_PVT_FLAGS_CARRSOLN           0xC0    /**< Carrier phase range solution (RTK mode) */

/* RX NAV-TIMEUTC message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDTOW       0x01    /**< validTOW (1 = Valid Time of Week) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDKWN       0x02    /**< validWKN (1 = Valid Week Number) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC       0x04    /**< validUTC (1 = Valid UTC Time) */
#define UBX_RX_NAV_TIMEUTC_VALID_UTCSTANDARD    0xF0    /**< utcStandard (0..15 = UTC standard identifier) */

#define UBX_CFG_LAYER_RAM                       (1 << 0)
#define UBX_CFG_LAYER_BBR                       (1 << 1)
#define UBX_CFG_LAYER_FLASH                     (1 << 2)

#define UBX_CFG_KEY_RATE_MEAS                   0x30210001
#define UBX_CFG_KEY_RATE_NAV                    0x30210002
#define UBX_CFG_KEY_CFG_USBOUTPROT_NMEA         0x10780002
#define UBX_CFG_KEY_NAVHPG_DGNSSMODE            0x20140011
#define UBX_CFG_KEY_TMODE_SVIN_MIN_DUR          0x40030010
#define UBX_CFG_KEY_TMODE_SVIN_ACC_LIMIT        0x40030011

#define UBX_CFG_KEY_INFMSG_UBX_USB 				0x20920004

#define UBX_CFG_INF_MSG_ERROR					(1 << 1)
#define UBX_CFG_INF_MSG_WARN					(1 << 2)
#define UBX_CFG_INF_MSG_NOTICE					(1 << 4)
#define UBX_CFG_INF_MSG_ERROR_WARN_NOTICE		(1 << 7)


/*** u-blox protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* General: Header */
typedef struct {
	uint8_t  sync1;
	uint8_t  sync2;
	uint16_t msg;
	uint16_t length;
} ubx_header_t;

/* General: Checksum */
typedef struct {
	uint8_t ck_a;
	uint8_t ck_b;
} ubx_checksum_t ;

/* Rx NAV-POSLLH */
typedef struct {
	uint32_t iTOW;   /**< GPS Time of Week [ms] */
	int32_t  lon;    /**< Longitude [1e-7 deg] */
	int32_t  lat;    /**< Latitude [1e-7 deg] */
	int32_t  height; /**< Height above ellipsoid [mm] */
	int32_t  hMSL;   /**< Height above mean sea level [mm] */
	uint32_t hAcc;   /**< Horizontal accuracy estimate [mm] */
	uint32_t vAcc;   /**< Vertical accuracy estimate [mm] */
} ubx_payload_rx_nav_posllh_t;

/* Rx NAV-DOP */
typedef struct {
	uint32_t iTOW; /**< GPS Time of Week [ms] */
	uint16_t gDOP; /**< Geometric DOP [0.01] */
	uint16_t pDOP; /**< Position DOP [0.01] */
	uint16_t tDOP; /**< Time DOP [0.01] */
	uint16_t vDOP; /**< Vertical DOP [0.01] */
	uint16_t hDOP; /**< Horizontal DOP [0.01] */
	uint16_t nDOP; /**< Northing DOP [0.01] */
	uint16_t eDOP; /**< Easting DOP [0.01] */
} ubx_payload_rx_nav_dop_t;

/* Rx NAV-COV */
typedef struct {
	uint32_t	iTOW; /**< GPS Time of Week [ms] */
	uint8_t		version;
	uint8_t		posCorValid;
	uint8_t		velCorValid;
	uint8_t		reserved0[9];
	float 		posCovNN; // - m^2 Position covariance matrix value p_NN
	float 		posCovNE; // - m^2 Position covariance matrix value p_NE
	float 		posCovND; // - m^2 Position covariance matrix value p_ND
	float 		posCovEE; // - m^2 Position covariance matrix value p_EE
	float 		posCovED; // - m^2 Position covariance matrix value p_ED
	float 		posCovDD; // - m^2 Position covariance matrix value p_DD
	float 		velCovNN; // - m^2/s^2 Velocity covariance matrix value v_NN
	float 		velCovNE; // - m^2/s^2 Velocity covariance matrix value v_NE
	float 		velCovND; // - m^2/s^2 Velocity covariance matrix value v_ND
	float 		velCovEE; // - m^2/s^2 Velocity covariance matrix value v_EE
	float 		velCovED; // - m^2/s^2 Velocity covariance matrix value v_ED
	float 		velCovDD; // - m^2/s^2 Velocity covariance matrix value v_DD		
} ubx_payload_rx_nav_cov_t;

/* Rx NAV-SOL */
typedef struct {
	uint32_t iTOW;     /**< GPS Time of Week [ms] */
	int32_t  fTOW;     /**< Fractional part of iTOW (range: +/-500000) [ns] */
	int16_t  week;     /**< GPS week */
	uint8_t  gpsFix;   /**< GPSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GPS + dead reckoning, 5 = time only fix */
	uint8_t  flags;
	int32_t  ecefX;
	int32_t  ecefY;
	int32_t  ecefZ;
	uint32_t pAcc;
	int32_t  ecefVX;
	int32_t  ecefVY;
	int32_t  ecefVZ;
	uint32_t sAcc;
	uint16_t pDOP;      /**< Position DOP [0.01] */
	uint8_t  reserved1;
	uint8_t  numSV;     /**< Number of SVs used in Nav Solution */
	uint32_t reserved2;
} ubx_payload_rx_nav_sol_t;

/* Rx NAV-PVT (ubx8) */
typedef struct {
	uint32_t iTOW;          /**< GPS Time of Week [ms] */
	uint16_t year;          /**< Year (UTC)*/
	uint8_t  month;         /**< Month, range 1..12 (UTC) */
	uint8_t  day;           /**< Day of month, range 1..31 (UTC) */
	uint8_t  hour;          /**< Hour of day, range 0..23 (UTC) */
	uint8_t  min;           /**< Minute of hour, range 0..59 (UTC) */
	uint8_t  sec;           /**< Seconds of minute, range 0..60 (UTC) */
	uint8_t  valid;         /**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
	uint32_t tAcc;          /**< Time accuracy estimate (UTC) [ns] */
	int32_t  nano;          /**< Fraction of second (UTC) [-1e9...1e9 ns] */
	uint8_t  fixType;       /**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	uint8_t  flags;         /**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
	uint8_t  reserved1;
	uint8_t  numSV;         /**< Number of SVs used in Nav Solution */
	int32_t  lon;           /**< Longitude [1e-7 deg] */
	int32_t  lat;           /**< Latitude [1e-7 deg] */
	int32_t  height;        /**< Height above ellipsoid [mm] */
	int32_t  hMSL;          /**< Height above mean sea level [mm] */
	uint32_t hAcc;          /**< Horizontal accuracy estimate [mm] */
	uint32_t vAcc;          /**< Vertical accuracy estimate [mm] */
	int32_t  velN;          /**< NED north velocity [mm/s]*/
	int32_t  velE;          /**< NED east velocity [mm/s]*/
	int32_t  velD;          /**< NED down velocity [mm/s]*/
	int32_t  gSpeed;        /**< Ground Speed (2-D) [mm/s] */
	int32_t  headMot;       /**< Heading of motion (2-D) [1e-5 deg] */
	uint32_t sAcc;          /**< Speed accuracy estimate [mm/s] */
	uint32_t headAcc;       /**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
	uint16_t pDOP;          /**< Position DOP [0.01] */
	uint16_t reserved2;
	uint32_t reserved3;
	int32_t  headVeh;       /**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
	uint32_t reserved4;     /**< (ubx8+ only) */
} ubx_payload_rx_nav_pvt_t;

/* Rx NAV-TIMEUTC */
typedef struct {
	uint32_t iTOW;           /**< GPS Time of Week [ms] */
	uint32_t tAcc;           /**< Time accuracy estimate (UTC) [ns] */
	int32_t  nano;           /**< Fraction of second, range -1e9 .. 1e9 (UTC) [ns] */
	uint16_t year;           /**< Year, range 1999..2099 (UTC) */
	uint8_t  month;          /**< Month, range 1..12 (UTC) */
	uint8_t  day;            /**< Day of month, range 1..31 (UTC) */
	uint8_t  hour;           /**< Hour of day, range 0..23 (UTC) */
	uint8_t  min;            /**< Minute of hour, range 0..59 (UTC) */
	uint8_t  sec;            /**< Seconds of minute, range 0..60 (UTC) */
	uint8_t  valid;          /**< Validity Flags (see UBX_RX_NAV_TIMEUTC_VALID_...) */
} ubx_payload_rx_nav_timeutc_t;

/* Rx NAV-SVINFO Part 1 */
typedef struct {
	uint32_t iTOW;           /**< GPS Time of Week [ms] */
	uint8_t  numCh;          /**< Number of channels */
	uint8_t  globalFlags;
	uint16_t reserved2;
} ubx_payload_rx_nav_svinfo_part1_t;

/* Rx NAV-SVINFO Part 2 (repeated) */
typedef struct {
	uint8_t chn;            /**< Channel number, 255 for SVs not assigned to a channel */
	uint8_t svid;           /**< Satellite ID */
	uint8_t flags;          /**< svUsed, diffCorr, orbitAvail, orbitEph, unhealthy, orbitAlm, orbitAop, smoothed */
	uint8_t quality;        /**< 0: no signal, 1: search, 2: aquited, 3: unusable, 5-7: locked */
	uint8_t cno;            /**< Carrier to Noise Ratio (Signal Strength) [dbHz] */
	int8_t  elev;           /**< Elevation [deg] */
	int16_t azim;           /**< Azimuth [deg] */
	int32_t prRes;          /**< Pseudo range residual [cm] */
} ubx_payload_rx_nav_svinfo_part2_t;

/* Rx NAV-SAT Part 1 */
typedef struct {
	uint32_t iTOW;           /**< GPS Time of Week [ms] */
	uint8_t  version;        /**< Message version (1) */
	uint8_t  numSvs;         /**< Number of Satellites */
	uint16_t reserved;
} ubx_payload_rx_nav_sat_part1_t;

/* Rx NAV-SAT Part 2 (repeated) */
typedef struct {
	uint8_t  gnssId;         /**< GNSS identifier */
	uint8_t  svId;           /**< Satellite ID */
	uint8_t  cno;            /**< Carrier to Noise Ratio (Signal Strength) [dbHz] */
	int8_t   elev;           /**< Elevation [deg] range: +/-90 */
	int16_t  azim;           /**< Azimuth [deg] range: 0-360 */
	int16_t  prRes;          /**< Pseudo range residual [0.1 m] */
	uint32_t flags;
} ubx_payload_rx_nav_sat_part2_t;

/* Rx NAV-SVIN (survey-in info) */
typedef struct {
	uint8_t  version;
	uint8_t  reserved1[3];
	uint32_t iTOW;
	uint32_t dur;
	int32_t  meanX;
	int32_t  meanY;
	int32_t  meanZ;
	int8_t   meanXHP;
	int8_t   meanYHP;
	int8_t   meanZHP;
	int8_t   reserved2;
	uint32_t meanAcc;
	uint32_t obs;
	uint8_t  valid;
	uint8_t  active;
	uint8_t  reserved3[2];
} ubx_payload_rx_nav_svin_t;

/* Rx NAV-VELNED */
typedef struct {
	uint32_t iTOW;           /**< GPS Time of Week [ms] */
	int32_t  velN;           /**< North velocity component [cm/s]*/
	int32_t  velE;           /**< East velocity component [cm/s]*/
	int32_t  velD;           /**< Down velocity component [cm/s]*/
	uint32_t speed;          /**< Speed (3-D) [cm/s] */
	uint32_t gSpeed;         /**< Ground speed (2-D) [cm/s] */
	int32_t  heading;        /**< Heading of motion 2-D [1e-5 deg] */
	uint32_t sAcc;           /**< Speed accuracy estimate [cm/s] */
	uint32_t cAcc;           /**< Course / Heading accuracy estimate [1e-5 deg] */
} ubx_payload_rx_nav_velned_t;

/* Rx MON-HW (ubx6) */
typedef struct {
	uint32_t pinSel;
	uint32_t pinBank;
	uint32_t pinDir;
	uint32_t pinVal;
	uint16_t noisePerMS;
	uint16_t agcCnt;
	uint8_t  aStatus;
	uint8_t  aPower;
	uint8_t  flags;
	uint8_t  reserved1;
	uint32_t usedMask;
	uint8_t  VP[25];
	uint8_t  jamInd;
	uint16_t reserved3;
	uint32_t pinIrq;
	uint32_t pullH;
	uint32_t pullL;
} ubx_payload_rx_mon_hw_ubx6_t;

/* Rx MON-HW (ubx7+) */
typedef struct {
	uint32_t pinSel;
	uint32_t pinBank;
	uint32_t pinDir;
	uint32_t pinVal;
	uint16_t noisePerMS;
	uint16_t agcCnt;
	uint8_t  aStatus;
	uint8_t  aPower;
	uint8_t  flags;
	uint8_t  reserved1;
	uint32_t usedMask;
	uint8_t  VP[17];
	uint8_t  jamInd;
	uint16_t reserved3;
	uint32_t pinIrq;
	uint32_t pullH;
	uint32_t pullL;
} ubx_payload_rx_mon_hw_ubx7_t;

/* Rx MON-RF (replaces MON-HW, protocol 27+) */
typedef struct {
	uint8_t version;
	uint8_t nBlocks;         /**< number of RF blocks included */
	uint8_t reserved1[2];

	struct ubx_payload_rx_mon_rf_block_t {
		uint8_t  blockId;       /**< RF block id */
		uint8_t  flags;         /**< jammingState */
		uint8_t  antStatus;     /**< Status of the antenna superior state machine */
		uint8_t  antPower;      /**< Current power status of antenna */
		uint32_t postStatus;    /**< POST status word */
		uint8_t  reserved2[4];
		uint16_t noisePerMS;    /**< Noise level as measured by the GPS core */
		uint16_t agcCnt;        /**< AGC Monitor (counts SIGI xor SIGLO, range 0 to 8191 */
		uint8_t  jamInd;        /**< CW jamming indicator, scaled (0=no CW jamming, 255=strong CW jamming) */
		int8_t   ofsI;          /**< Imbalance of I-part of complex signal */
		uint8_t  magI;          /**< Magnitude of I-part of complex signal (0=no signal, 255=max magnitude) */
		int8_t   ofsQ;          /**< Imbalance of Q-part of complex signal */
		uint8_t  magQ;          /**< Magnitude of Q-part of complex signal (0=no signal, 255=max magnitude) */
		uint8_t  reserved3[3];
	};

	ubx_payload_rx_mon_rf_block_t block[1]; ///< only read out the first block
} ubx_payload_rx_mon_rf_t;

/* Rx MON-VER Part 1 */
typedef struct {
	uint8_t swVersion[30];
	uint8_t hwVersion[10];
} ubx_payload_rx_mon_ver_part1_t;

/* Rx MON-VER Part 2 (repeated) */
typedef struct {
	uint8_t extension[30];
} ubx_payload_rx_mon_ver_part2_t;

/* Rx ACK-ACK */
typedef union {
	uint16_t msg;
	struct {
		uint8_t clsID;
		uint8_t msgID;
	};
} ubx_payload_rx_ack_ack_t;

/* Rx ACK-NAK */
typedef union {
	uint16_t msg;
	struct {
		uint8_t clsID;
		uint8_t msgID;
	};
} ubx_payload_rx_ack_nak_t;

/* Tx CFG-PRT */
typedef struct {
	uint8_t  portID;
	uint8_t  reserved0;
	uint16_t txReady;
	uint32_t mode;
	uint32_t baudRate;
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	uint16_t reserved5;
} ubx_payload_tx_cfg_prt_t;

/* Tx CFG-RATE */
typedef struct {
	uint16_t measRate;      /**< Measurement Rate, GPS measurements are taken every measRate milliseconds */
	uint16_t navRate;       /**< Navigation Rate, in number of measurement cycles. This parameter cannot be changed, and must be set to 1 */
	uint16_t timeRef;       /**< Alignment to reference time: 0 = UTC time, 1 = GPS time */
} ubx_payload_tx_cfg_rate_t;

/* Tx CFG-CFG */
typedef struct {
	uint32_t clearMask;     /**< Clear settings */
	uint32_t saveMask;      /**< Save settings */
	uint32_t loadMask;      /**< Load settings */
	uint8_t  deviceMask;    /**< Storage devices to apply this top */
} ubx_payload_tx_cfg_cfg_t;

/* Tx CFG-VALSET (protocol version 27+) */
typedef struct {
	uint8_t version;        /**< Message version, set to 0 */
	uint8_t layers;         /**< The layers where the configuration should be applied (@see UBX_CFG_LAYER_*) */
	uint8_t reserved1[2];
	uint8_t cfgData;        /**< configuration data (key and value pairs, max 64) */
} ubx_payload_tx_cfg_valset_t;

/* Tx CFG-NAV5 */
typedef struct {
	uint16_t mask;
	uint8_t  dynModel;       /**< Dynamic Platform model: 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
	uint8_t  fixMode;        /**< Position Fixing Mode: 1 2D only, 2 3D only, 3 Auto 2D/3D */
	int32_t  fixedAlt;
	uint32_t fixedAltVar;
	int8_t   minElev;
	uint8_t  drLimit;
	uint16_t pDop;
	uint16_t tDop;
	uint16_t pAcc;
	uint16_t tAcc;
	uint8_t  staticHoldThresh;
	uint8_t  dgpsTimeOut;
	uint8_t  cnoThreshNumSVs;        /**< (ubx7+ only, else 0) */
	uint8_t  cnoThresh;              /**< (ubx7+ only, else 0) */
	uint16_t reserved;
	uint16_t staticHoldMaxDist;      /**< (ubx8+ only, else 0) */
	uint8_t  utcStandard;            /**< (ubx8+ only, else 0) */
	uint8_t  reserved3;
	uint32_t reserved4;
} ubx_payload_tx_cfg_nav5_t;

/* tx cfg-rst */
typedef struct {
	uint16_t navBbrMask;
	uint8_t  resetMode;
	uint8_t  reserved1;
} ubx_payload_tx_cfg_rst_t;

/* tx cfg-sbas */
typedef struct {
	uint8_t  mode;
	uint8_t  usage;
	uint8_t  maxSBAS;
	uint8_t  scanmode2;
	uint32_t scanmode1;
} ubx_payload_tx_cfg_sbas_t;

/* Tx CFG-MSG */
typedef struct {
	union {
		uint16_t msg;
		struct {
			uint8_t msgClass;
			uint8_t msgID;
		};
	};
	uint8_t rate;
} ubx_payload_tx_cfg_msg_t;

/* CFG-TMODE3 ublox 8 (protocol version >= 20) */
typedef struct {
	uint8_t  version;
	uint8_t  reserved1;
	uint16_t flags;
	int32_t  ecefXOrLat;
	int32_t  ecefYOrLon;
	int32_t  ecefZOrAlt;
	int8_t   ecefXOrLatHP;
	int8_t   ecefYOrLonHP;
	int8_t   ecefZOrAltHP;
	uint8_t  reserved2;
	uint32_t fixedPosAcc;
	uint32_t svinMinDur;
	uint32_t svinAccLimit;
	uint8_t  reserved3[8];
} ubx_payload_tx_cfg_tmode3_t;

typedef struct {
	uint8_t msgVer;           /**< Message version (expected 0x00) */
	uint8_t numTrkChHw;       /**< Number of tracking channels available (read only) */
	uint8_t numTrkChUse;      /**< Number of tracking channels to use (0xFF for numTrkChHw) */
	uint8_t numConfigBlocks;  /**< Count of repeated blocks */

	struct ubx_payload_tx_cgf_gnss_block_t {
		uint8_t gnssId;     /**< GNSS ID */
		uint8_t resTrkCh;   /**< Number of reseved (minimum) tracking channels */
		uint8_t maxTrkCh;   /**< Maximum number or tracking channels */
		uint8_t reserved1;
		uint32_t flags;     /**< Bitfield flags (see UBX_TX_CFG_GNSS_FLAGS_*) */
	};

	ubx_payload_tx_cgf_gnss_block_t block[7];  /**< GPS, SBAS, Galileo, BeiDou, IMES 0-8, QZSS, GLONASS */
} ubx_payload_tx_cfg_gnss_t;

/* NAV RELPOSNED (protocol version 27+) */
typedef struct {
	uint8_t     version;         /**< message version (expected 0x01) */
	uint8_t     reserved0;
	uint16_t    refStationId;    /**< Reference station ID. Must be in the range 0..4095 */
	uint32_t    iTOW;            /**< [ms] GPS time of week of the navigation epoch */
	int32_t     relPosN;         /**< [cm] North component of relative position vector */
	int32_t     relPosE;         /**< [cm] East component of relative position vector */
	int32_t     relPosD;         /**< [cm] Down component of relative position vector */
	int32_t     relPosLength;    /**< [cm] Length of the relative position vector */
	int32_t     relPosHeading;   /**< [1e-5 deg] Heading of the relative position vector */
	uint32_t    reserved1;
	int8_t      relPosHPN;       /**< [0.1 mm] High-precision North component of relative position vector */
	int8_t      relPosHPE;       /**< [0.1 mm] High-precision East component of relative position vector */
	int8_t      relPosHPD;       /**< [0.1 mm] High-precision Down component of relative position vector */
	int8_t      relPosHPLength;  /**< [0.1 mm] High-precision component of the length of the relative position vector */
	uint32_t    accN;            /**< [0.1 mm] Accuracy of relative position North component */
	uint32_t    accE;            /**< [0.1 mm] Accuracy of relative position East component */
	uint32_t    accD;            /**< [0.1 mm] Accuracy of relative position Down component */
	uint32_t    accLength;       /**< [0.1 mm] Accuracy of the length of the relative position vector */
	uint32_t    accHeading;      /**< [1e-5 deg] Accuracy of the heading of the relative position vector */
	uint32_t    reserved2;
	uint32_t    flags;
} ubx_payload_rx_nav_relposned_t;

/* General message and payload buffer union */
typedef union {
	ubx_payload_rx_nav_pvt_t          payload_rx_nav_pvt;
	ubx_payload_rx_nav_posllh_t       payload_rx_nav_posllh;
	ubx_payload_rx_nav_sol_t          payload_rx_nav_sol;
	ubx_payload_rx_nav_dop_t          payload_rx_nav_dop;
	ubx_payload_rx_nav_timeutc_t      payload_rx_nav_timeutc;
	ubx_payload_rx_nav_svinfo_part1_t payload_rx_nav_svinfo_part1;
	ubx_payload_rx_nav_svinfo_part2_t payload_rx_nav_svinfo_part2;
	ubx_payload_rx_nav_sat_part1_t    payload_rx_nav_sat_part1;
	ubx_payload_rx_nav_sat_part2_t    payload_rx_nav_sat_part2;
	ubx_payload_rx_nav_svin_t         payload_rx_nav_svin;
	ubx_payload_rx_nav_velned_t       payload_rx_nav_velned;
	ubx_payload_rx_nav_cov_t       	  payload_rx_nav_cov;
	ubx_payload_rx_mon_hw_ubx6_t      payload_rx_mon_hw_ubx6;
	ubx_payload_rx_mon_hw_ubx7_t      payload_rx_mon_hw_ubx7;
	ubx_payload_rx_mon_rf_t           payload_rx_mon_rf;
	ubx_payload_rx_mon_ver_part1_t    payload_rx_mon_ver_part1;
	ubx_payload_rx_mon_ver_part2_t    payload_rx_mon_ver_part2;
	ubx_payload_rx_ack_ack_t          payload_rx_ack_ack;
	ubx_payload_rx_ack_nak_t          payload_rx_ack_nak;
	ubx_payload_tx_cfg_prt_t          payload_tx_cfg_prt;
	ubx_payload_tx_cfg_rate_t         payload_tx_cfg_rate;
	ubx_payload_tx_cfg_nav5_t         payload_tx_cfg_nav5;
	ubx_payload_tx_cfg_rst_t          payload_tx_cfg_rst;
	ubx_payload_tx_cfg_sbas_t         payload_tx_cfg_sbas;
	ubx_payload_tx_cfg_msg_t          payload_tx_cfg_msg;
	ubx_payload_tx_cfg_tmode3_t       payload_tx_cfg_tmode3;
	ubx_payload_tx_cfg_cfg_t          payload_tx_cfg_cfg;
	ubx_payload_tx_cfg_valset_t       payload_tx_cfg_valset;
	ubx_payload_tx_cfg_gnss_t         payload_tx_cfg_gnss;
	ubx_payload_rx_nav_relposned_t    payload_rx_nav_relposned;
} ubx_buf_t;

#pragma pack(pop)

/* Decoder state */
typedef enum {
	UBX_DECODE_SYNC1 = 0,
	UBX_DECODE_SYNC2,
	UBX_DECODE_CLASS,
	UBX_DECODE_ID,
	UBX_DECODE_LENGTH1,
	UBX_DECODE_LENGTH2,
	UBX_DECODE_PAYLOAD,
	UBX_DECODE_CHKSUM1,
	UBX_DECODE_CHKSUM2,

	UBX_DECODE_RTCM3
} ubx_decode_state_t;

/* Rx message state */
typedef enum {
	UBX_RXMSG_IGNORE = 0,
	UBX_RXMSG_HANDLE,
	UBX_RXMSG_DISABLE,
	UBX_RXMSG_ERROR_LENGTH
} ubx_rxmsg_state_t;

/* ACK state */
typedef enum {
	UBX_ACK_IDLE = 0,
	UBX_ACK_WAITING,
	UBX_ACK_GOT_ACK,
	UBX_ACK_GOT_NAK
} ubx_ack_state_t;



class UbxProtocol
{
    public:
        UbxProtocol(bool msgDebug, std::string portPath, int baudRate, bool baseStation);
        ~UbxProtocol();

        int parseChar(const uint8_t b);

		bool ackNackMessageReady();
		bool gpsStatusMessageReady();
		bool gpsSurveyMessageReady();
		bool navSatStatusMessageReady();
		bool navSatFixMessageReady();
		bool debugMessageReady();
		bool warningMessageReady();

		bool getAckStatus();

		rcraicer_msgs::msg::GPSSurvey getGpsSurveyMessage();
		rcraicer_msgs::msg::GPSStatus getGpsStatusMessage();
		sensor_msgs::msg::NavSatFix getNavSatFixMessage();
		sensor_msgs::msg::NavSatStatus getNavSatStatusMessage();

		std::string getDebugMessage();
		std::string getWarningMessage();

		typedef std::function<void()> MessageCallback;
		typedef std::function<void()> ConfigurationCallback;

		void registerMessageCallback(MessageCallback callback);
		void registerConfigurationCallback(ConfigurationCallback callback);

		bool connect();
		void configure();
		void configureSurveyIn(uint32_t minDuration, uint32_t accLimit);

    private:
		SerialPort* serialPort;
		void serial_data_callback(const uint8_t data);
        void decodeInit();
        void clearGPSStatusMsg();

        void addByteToChecksum(const uint8_t b);
		void outputMsgType();
        
        int payloadRxInit();
        int payloadRxAdd(const uint8_t b);
	    int payloadRxAddMonVer(const uint8_t b);
	    int payloadRxAddNavSat(const uint8_t b);
        int payloadRxDone(void);

		bool sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length);
		void calcChecksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum);

		int initCfgValset();
		template<typename T>
			bool cfgValset(uint32_t key_id, T value, int &msg_size);


		std::string portPath;
		int baudRate;
		bool baseStation;
  
        ubx_decode_state_t  decode_state{};
        uint16_t rx_msg{};
        uint8_t rx_ck_a{0};
	    uint8_t rx_ck_b{0};
        uint16_t rx_payload_index{0};
    	uint16_t rx_payload_length{0};        

        ubx_rxmsg_state_t   rx_state{UBX_RXMSG_IGNORE};

        bool configured{false};
        bool use_nav_pvt{true};

		bool isGpsStatusMsgReady{false};
		bool isGpsSurveyMsgReady{false};
		bool isNavSatFixMsgReady{false};
		bool isNavSatStatusMsgReady{false};
		bool isAckNacReady{false};
		bool isDebugMessageReady{false};
		bool isWarningMessageReady{false};

		bool acknowledged = false;

		bool msgDebug;

		MessageCallback messageCallback;
		ConfigurationCallback configurationCallback;		

        rcraicer_msgs::msg::GPSStatus gpsStatusMsg;
		rcraicer_msgs::msg::GPSSurvey gpsSurveyMsg;
        sensor_msgs::msg::NavSatFix fixMsg;        
		sensor_msgs::msg::NavSatStatus statusMsg;        


        ubx_buf_t   buf{};
		ubx_buf_t   txbuf{};

		std::string debugMessage;
		std::string warningMessage;

};
