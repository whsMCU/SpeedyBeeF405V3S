/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __M8N_H
#define __M8N_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef enum {
  LON,
  LAT
} gps_axis_e;

typedef enum {
  NAV_MODE_NONE,
  NAV_MODE_POSHOLD,
  NAV_MODE_WP
} nav_mode_e;

/************************        AP FlightMode        **********************************/
/* Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks.*/
#define AP_MODE 40  // Create a deadspan for GPS.

typedef struct _M8N_UBX_NAV_POSLLH
{
	unsigned char CLASS;
	unsigned char ID;
	unsigned short length;

	unsigned int iTOW;
	signed int lon;
	signed int lat;
	signed int height;
	signed int hMSL;
	unsigned int hAcc;
	unsigned int vAcc;

	double lon_f64;
	double lat_f64;
}M8N_UBX_NAV_POSLLH;

extern M8N_UBX_NAV_POSLLH posllh_test;

extern uint8_t m8n_rx_buf[36];
extern uint8_t m8n_rx_cplt_flag;

unsigned char M8N_UBX_CHKSUM_Check(unsigned char* data, unsigned char len);
void M8N_UBX_NAV_POSLLH_Parsing(unsigned char* data, M8N_UBX_NAV_POSLLH* posllh);
void M8N_UART4_Initialization(void);
void M8N_Initialization(void);
void gpsUpdate(uint32_t currentTimeUs);

typedef struct {
  uint8_t nav_mode;
  int32_t  GPS_coord[2];
  int32_t  GPS_home[2];
  int32_t  GPS_hold[2];
  uint8_t  GPS_numSat;
  uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
  int16_t  GPS_directionToHome;                         // direction to home - unit: degree
  uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
  uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  uint8_t  GPS_update;                              // a binary toogle to distinct a GPS position update
  int16_t  GPS_angle[2];                      // the angles that must be applied for GPS correction
  uint16_t GPS_ground_course;                       //                   - unit: degree*10
  int16_t nav_takeoff_bearing;
  int16_t nav[2];
} GpsNav_t;

extern GpsNav_t GpsNav;

// NAV-POSLLH 구조체 (위치정보)
typedef struct {
    uint32_t iTOW;     // GPS Time of Week (ms)
    int32_t lon;       // 경도 (1e-7 도)
    int32_t lat;       // 위도 (1e-7 도)
    int32_t height;    // 해발고도 (mm)
    int32_t hMSL;      // 평균해수면 고도 (mm)
    uint32_t hAcc;     // 수평 오차 (mm)
    uint32_t vAcc;     // 수직 오차 (mm)
    double lon_deg;
    double lat_deg;
} UbxNavPosllh_t;

// NAV-SAT 위성정보 (채널 당 12바이트씩 반복)
typedef struct {
    uint8_t gnssId;    // GNSS 시스템 ID (0=GPS, 1=SBAS, 2=Galileo 등)
    uint8_t svId;      // 위성 PRN 번호
    uint8_t cno;       // 신호 세기 (dBHz)
    uint8_t elev;      // 위성 고도 (deg)
    int16_t azim;      // 위성 방위각 (deg)
    int16_t prRes;     // 위성 의사거리 잔차 (cm)
} UbxNavSat_SV_t;

#define MAX_SATS 16

typedef struct {
    uint32_t iTOW;      // GPS Time of Week
    uint8_t numSvs;     // 사용 위성 수
    uint8_t reserved1;
    uint16_t reserved2;
    UbxNavSat_SV_t sv[MAX_SATS];
} UbxNavSat_t;

typedef struct {
    uint32_t iTOW;     // GPS Time of Week in ms
    uint8_t gpsFix;    // 0: No Fix, 2: 2D Fix, 3: 3D Fix, 5: Time only
    uint8_t flags;     // Fix flags
    uint8_t fixStat;   // Fix status info
    uint8_t flags2;    // Additional flags
    uint32_t ttff;     // Time to first fix (ms)
    uint32_t msss;     // Time since startup/reset to last fix (ms)
} UbxNavStatus_t;

extern UbxNavPosllh_t posllh;
extern UbxNavSat_t sat;

extern UbxNavStatus_t nav_status;

void Ubx_HandleMessage(uint8_t cls, uint8_t id, uint8_t *payload, uint16_t length);

void GPS_calc_longitude_scaling(int32_t lat);
void GPS_reset_home_position(void);

#ifdef __cplusplus
}
#endif
#endif /*__ M8N_H */
