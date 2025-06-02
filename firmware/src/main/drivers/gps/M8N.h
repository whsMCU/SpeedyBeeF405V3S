/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __M8N_H
#define __M8N_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

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

extern UbxNavPosllh_t posllh;
extern UbxNavSat_t sat;

void Ubx_HandleMessage(uint8_t cls, uint8_t id, uint8_t *payload, uint16_t length);

#ifdef __cplusplus
}
#endif
#endif /*__ M8N_H */
