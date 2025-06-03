/*
 * M8N.c
 *
 *  Created on: Aug 22, 2019
 *      Author: Administrator
 */

#include "M8N.h"
#include "drivers/gps/gps.h"
#include "fc/runtime_config.h"

M8N_UBX_NAV_POSLLH posllh_test;

uint8_t m8n_rx_buf[36];
uint8_t m8n_rx_cplt_flag = 0;

const unsigned char UBX_CFG_PRT[] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
	0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x01, 0x00,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79
};

const unsigned char UBX_CFG_MSG_POSLLH[] = {
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x13, 0xBE
};

const unsigned char UBX_CFG_MSG_SAT[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x35, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x46, 0x23
};

const unsigned char UBX_CFG_MSG_STATUS[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x14, 0xC5
};

const unsigned char UBX_CFG_RATE[] = {
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00,
	0x01, 0x00, 0xDE, 0x6A
};

const unsigned char UBX_CFG_CFG[] = {
	0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31,
	0xBF
};

void M8N_Initialization(void)
{
	//M8N_UART4_Initialization();
	uartOpen(_DEF_UART6, 9600);

	uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_PRT[0], sizeof(UBX_CFG_PRT));
	HAL_Delay(100);
	uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_MSG_POSLLH[0], sizeof(UBX_CFG_MSG_POSLLH));
	HAL_Delay(100);
  uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_MSG_SAT[0], sizeof(UBX_CFG_MSG_SAT));
  HAL_Delay(100);
  uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_MSG_STATUS[0], sizeof(UBX_CFG_MSG_SAT));
  HAL_Delay(100);
	uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_RATE[0], sizeof(UBX_CFG_RATE));
	HAL_Delay(100);
	uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_CFG[0], sizeof(UBX_CFG_CFG));
}

void gpsUpdate(uint32_t currentTimeUs)
{
//  if(m8n_rx_cplt_flag == 1)
//  {
//    m8n_rx_cplt_flag = 0;
//
//    if(M8N_UBX_CHKSUM_Check(&m8n_rx_buf[0], 36) == 1)
//    {
//      M8N_UBX_NAV_POSLLH_Parsing(&m8n_rx_buf[0], &posllh_test);
//
//      //printf("LAT: %ld\tLON: %ld\tHeight: %ld\n", posllh.lat, posllh.lon, posllh.height);
//    }
//  }

}

unsigned char M8N_UBX_CHKSUM_Check(unsigned char* data, unsigned char len)
{
	unsigned char CK_A = 0, CK_B = 0;

	for(int i=2;i<len-2;i++)
	{
		CK_A = CK_A + data[i];
		CK_B = CK_B + CK_A;
	}

	return ((CK_A == data[len-2]) && (CK_B == data[len-1]));
}

enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} ubx_nav_fix_type;

enum {
    NAV_STATUS_FIX_VALID = 1,
    NAV_STATUS_TIME_WEEK_VALID = 4,
    NAV_STATUS_TIME_SECOND_VALID = 8
} ubx_nav_status_bit;

GpsNav_t GpsNav;

UbxNavPosllh_t posllh;
UbxNavSat_t sat;
UbxNavStatus_t nav_status;
static bool next_fix;

uint32_t posllh_dt, posllh_tmp, sat_dt, sat_tmp, status_dt, status_tmp;

void Ubx_HandleMessage(uint8_t cls, uint8_t id, uint8_t *payload, uint16_t length) {
    if (cls == 0x01 && id == 0x02) {  // NAV-POSLLH
        if (length < 28) return;

        posllh_dt = micros() - posllh_tmp;
        posllh_tmp = micros();

        posllh.iTOW = (payload[0]) | (payload[1]<<8) | (payload[2]<<16) | (payload[3]<<24);
        posllh.lon = (int32_t)(payload[4] | (payload[5]<<8) | (payload[6]<<16) | (payload[7]<<24));
        posllh.lat = (int32_t)(payload[8] | (payload[9]<<8) | (payload[10]<<16) | (payload[11]<<24));
        posllh.height = (int32_t)(payload[12] | (payload[13]<<8) | (payload[14]<<16) | (payload[15]<<24));
        posllh.hMSL = (int32_t)(payload[16] | (payload[17]<<8) | (payload[18]<<16) | (payload[19]<<24));
        posllh.hAcc = (uint32_t)(payload[20] | (payload[21]<<8) | (payload[22]<<16) | (payload[23]<<24));
        posllh.vAcc = (uint32_t)(payload[24] | (payload[25]<<8) | (payload[26]<<16) | (payload[27]<<24));
        // 위치값 사용 예: 경도, 위도는 1e-7 도 단위 → 실수로 변환 필요
        posllh.lon_deg = posllh.lon / 1e7f;
        posllh.lat_deg = posllh.lat / 1e7f;
        // 여기서 드론 위치 갱신 또는 출력 등 처리
    } else if (cls == 0x01 && id == 0x35) { // NAV-SAT
        if (length < 8) return;

        sat_dt = micros() - sat_tmp;
        sat_tmp = micros();

        sat.iTOW = (payload[0]) | (payload[1]<<8) | (payload[2]<<16) | (payload[3]<<24);
        sat.numSvs = payload[4];
        // payload[5], payload[6], payload[7]은 reserved
        uint8_t *p = &payload[8];
        int sv_count = (length - 8) / 12;
        if (sv_count > MAX_SATS) sv_count = MAX_SATS;
        for (int i=0; i<sv_count; i++) {
            sat.sv[i].gnssId = p[0];
            sat.sv[i].svId = p[1];
            sat.sv[i].cno = p[2];
            sat.sv[i].elev = p[3];
            sat.sv[i].azim = p[4] | (p[5]<<8);
            sat.sv[i].prRes = p[6] | (p[7]<<8) | (p[8]<<16) | (p[9]<<24);
            p += 12;
            // 여기서 위성 정보 활용 가능 (신호 세기, PRN 등)
        }
    } else if (cls == 0x01 && id == 0x03) { // NAV-STATUS
        if (length < 16) return;

        status_dt = micros() - status_tmp;
        status_tmp = micros();

        nav_status.iTOW   = payload[0] | (payload[1]<<8) | (payload[2]<<16) | (payload[3]<<24);
        nav_status.gpsFix = payload[4];
        nav_status.flags  = payload[5];
        nav_status.fixStat = payload[6];
        nav_status.flags2  = payload[7];
        nav_status.ttff    = payload[8] | (payload[9]<<8) | (payload[10]<<16) | (payload[11]<<24);
        nav_status.msss    = payload[12] | (payload[13]<<8) | (payload[14]<<16) | (payload[15]<<24);

        next_fix = (nav_status.flags & NAV_STATUS_FIX_VALID) && (nav_status.gpsFix == FIX_3D);
        if (!next_fix){
          DISABLE_STATE(GPS_FIX);
        }
        gpsSetFixState(next_fix);
    }
}


void M8N_UBX_NAV_POSLLH_Parsing(unsigned char* data, M8N_UBX_NAV_POSLLH* posllh)
{
  posllh->CLASS = data[2];
  posllh->ID = data[3];
  posllh->length = data[4] | data[5]<<8;

  posllh->iTOW = data[6] | data[7]<<8 | data[8]<<16 | data[9]<<24;
  posllh->lon = data[10] | data[11]<<8 | data[12]<<16 | data[13]<<24;
  posllh->lat = data[14] | data[15]<<8 | data[16]<<16 | data[17]<<24;
  posllh->height = data[18] | data[19]<<8 | data[20]<<16 | data[21]<<24;
  posllh->hMSL = data[22] | data[23]<<8 | data[24]<<16 | data[25]<<24;
  posllh->hAcc = data[26] | data[27]<<8 | data[28]<<16 | data[29]<<24;
  posllh->vAcc = data[30] | data[31]<<8 | data[32]<<16 | data[33]<<24;

//	posllh->lon_f64 = posllh->lon / 10000000.;
//	posllh->lat_f64 = posllh->lat / 10000000.;
}
