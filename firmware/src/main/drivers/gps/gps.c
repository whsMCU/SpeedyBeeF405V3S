/*
 * M8N.c
 *
 *  Created on: Aug 22, 2019
 *      Author: Administrator
 */

#include "hw.h"
#ifdef USE_GPS

#include "build/debug.h"

#include "drivers/gps/gps.h"
#include "fc/runtime_config.h"
#include "common/maths.h"
#include "flight/imu.h"

#include "sensors/compass.h"

typedef struct PID_PARAM_ {
  float kP;
  float kI;
  float kD;
  float Imax;
} PID_PARAM;

PID_PARAM posholdPID_PARAM;
PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;

int32_t get_P(int32_t error, struct PID_PARAM_* pid) {
  return (float)error * pid->kP;
}

int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) {
  pid->integrator += ((float)error * pid_param->kI) * *dt;
  pid->integrator = constrain(pid->integrator,-pid_param->Imax,pid_param->Imax);
  return pid->integrator;
}

int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) { // dt in milliseconds
  pid->derivative = (input - pid->last_input) / *dt;

  /// Low pass filter cut frequency for derivative calculation.
  float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  // discrete low pass filter, cuts out the
  // high frequency noise that can drive the controller crazy
  pid->derivative = pid->lastderivative + (*dt / ( filter + *dt)) * (pid->derivative - pid->lastderivative);
  // update state
  pid->last_input = input;
  pid->lastderivative    = pid->derivative;
  // add in derivative component
  return pid_param->kD * pid->derivative;
}

void reset_PID(struct PID_* pid) {
  pid->integrator = 0;
  pid->last_input = 0;
  pid->lastderivative = 0;
}

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

const unsigned char UBX_CFG_MSG_PVT[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x18, 0xE1
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

GpsNav_t GpsNav;

UbxNavPosllh_t posllh;
UbxNavSat_t sat;
UbxNavStatus_t nav_status;
nav_pvt_t pvt;
/////////////////////////////////////////////
gpsLocation_t GPS_home_llh;
gpsSolutionData_t gpsSol;

uint16_t GPS_distanceToHome;        // distance to home point in meters
uint32_t GPS_distanceToHomeCm;
int16_t GPS_directionToHome;        // direction to home or hol point in degrees * 10
uint32_t GPS_distanceFlownInCm;     // distance flown since armed in centimeters

#define GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S 15 // 0.54 km/h 0.335 mph

gpsSolutionData_t gpsSol;

uint8_t GPS_update = 0;             // toogle to distinct a GPS position update (directly or via MSP)

uint8_t GPS_numCh;

GPS_svinfo_t GPS_svinfo[GPS_SV_MAXSATS_M8N];

// Do we have a new valid fix?
static bool ubxHaveNewValidFix = false;

// Do we have new position information?
static bool ubxHaveNewPosition = false;

// Do we have new speed information?
static bool ubxHaveNewSpeed = false;
///////////////////////////////////////////////////
static bool next_fix;

static float gpsDataIntervalSeconds = 0.1f;
static float gpsDataFrequencyHz = 10.0f;
static uint16_t currentGpsStamp = 0; // logical timer for received position update

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

static void gpsResetSolution(void)
{
    GpsNav.eph = 9999;
    GpsNav.epv = 9999;
    GpsNav.GPS_numSat = 0;
    GpsNav.hdop = 9999;

    GpsNav.flags.validVelNE = false;
    GpsNav.flags.validVelD = false;
    GpsNav.flags.validMag = false;
    GpsNav.flags.validEPE = false;
    GpsNav.flags.validTime = false;
}

void gpsInit(void)
{
	uartOpen(_DEF_UART1, 9600);

	uartWrite(_DEF_UART1, (uint8_t*)&UBX_CFG_PRT[0], sizeof(UBX_CFG_PRT));
	HAL_Delay(100);
	uartWrite(_DEF_UART1, (uint8_t*)&UBX_CFG_MSG_POSLLH[0], sizeof(UBX_CFG_MSG_POSLLH));
	HAL_Delay(100);
  uartWrite(_DEF_UART1, (uint8_t*)&UBX_CFG_MSG_SAT[0], sizeof(UBX_CFG_MSG_SAT));
  HAL_Delay(100);
  uartWrite(_DEF_UART1, (uint8_t*)&UBX_CFG_MSG_STATUS[0], sizeof(UBX_CFG_MSG_SAT));
  HAL_Delay(100);
  uartWrite(_DEF_UART1, (uint8_t*)&UBX_CFG_MSG_PVT[0], sizeof(UBX_CFG_MSG_PVT));
  HAL_Delay(100);
	uartWrite(_DEF_UART1, (uint8_t*)&UBX_CFG_RATE[0], sizeof(UBX_CFG_RATE));
	HAL_Delay(100);
	uartWrite(_DEF_UART1, (uint8_t*)&UBX_CFG_CFG[0], sizeof(UBX_CFG_CFG));

  // Reset solution, timeout and prepare to start
  gpsResetSolution();
	GpsNav.nav_mode = NAV_MODE_NONE;
	GpsNav.GPS_wp_radius = GPS_WP_RADIUS;
	GpsNav.gps_set_home_point_once = false;

  posholdPID_PARAM.kP   = (float)0.11f;
  posholdPID_PARAM.kI   = (float)0.0f;
  posholdPID_PARAM.Imax = 2000;

  poshold_ratePID_PARAM.kP   = (float)2.0f;
  poshold_ratePID_PARAM.kI   = (float)0.08f;
  poshold_ratePID_PARAM.kD   = (float)0.045f;
  poshold_ratePID_PARAM.Imax = 2000;

  navPID_PARAM.kP   = (float)1.4f;
  navPID_PARAM.kI   = (float)0.2f;
  navPID_PARAM.kD   = (float)0.08f;
  navPID_PARAM.Imax = 2000;

  GpsNav.ubxHaveNewPositionAndNewSpeed = false;

}

//Apply moving average filter to GPS data
static void GPS_Filter(int32_t *GPS_coord)
{
  GpsNavFilter_t * GpsFilter = &GpsNav.GPS_filter;
  GpsFilter->GPS_filter_index = (GpsFilter->GPS_filter_index+1) % GPS_FILTER_VECTOR_LENGTH;
  for (int axis = 0; axis< 2; axis++) {
    GpsFilter->GPS_read[axis] = GPS_coord[axis]; //latest unfiltered data is in GPS_latitude and GPS_longitude
    GpsFilter->GPS_degree[axis] = GpsFilter->GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t

    // How close we are to a degree line ? its the first three digits from the fractions of degree
    // later we use it to Check if we are close to a degree line, if yes, disable averaging,
    GpsFilter->fraction3[axis] = (GpsFilter->GPS_read[axis]- GpsFilter->GPS_degree[axis]*10000000) / 10000;

    GpsFilter->GPS_filter_sum[axis] -= GpsFilter->GPS_filter[axis][GpsFilter->GPS_filter_index];
    GpsFilter->GPS_filter[axis][GpsFilter->GPS_filter_index] = GpsFilter->GPS_read[axis] - (GpsFilter->GPS_degree[axis]*10000000);
    GpsFilter->GPS_filter_sum[axis] += GpsFilter->GPS_filter[axis][GpsFilter->GPS_filter_index];
    GpsFilter->GPS_filtered[axis] = GpsFilter->GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GpsFilter->GPS_degree[axis]*10000000);
    if ( GpsNav.nav_mode == NAV_MODE_POSHOLD) {      //we use gps averaging only in poshold mode...
      if ( GpsFilter->fraction3[axis]>1 && GpsFilter->fraction3[axis]<999 ) GPS_coord[axis] = GpsFilter->GPS_filtered[axis];
    }
  }
}

// GPS epoch is Jan 6, 1980. Unix epoch is Jan 1, 1970.
// Offset in seconds from Unix epoch to GPS epoch
#define GPS_EPOCH_OFFSET_SECONDS 315964800
// Current GPS-UTC leap seconds offset (as of 2017, 18 leap seconds)
#define GPS_LEAP_SECONDS 18

// Convert date/time to Unix seconds (UTC)
static int64_t dateTimeToUnixSeconds(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
    int32_t days = 0;
    for (int y = 1970; y < year; y++) {
        days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
    }
    int isLeap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 1 : 0;
    days += gpsMonthDays[isLeap][month - 1] + (day - 1);
    return (int64_t)days * 86400 + hour * 3600 + min * 60 + sec;
}

// Convert gpsDateTime_t to Unix epoch seconds; returns 0 if not valid
uint32_t gpsDateTimeToEpoch(const gpsDateTime_t *dt)
{
    if (!dt || !dt->valid) {
        return 0;
    }

    if (dt->year < 1980 || dt->month < 1 || dt->month > 12 || dt->day < 1 ||
        dt->day > 31 || dt->hour > 23 || dt->min > 59 || dt->sec > 60) {
        return 0;
    }

    return (uint32_t)dateTimeToUnixSeconds(dt->year, dt->month, dt->day, dt->hour, dt->min, dt->sec);
}

// NAV-PVT already carries UTC calendar fields; only the nanosecond correction can
// require a one-second calendar adjustment.
static bool gpsDateTimeIsLeapYear(uint16_t year)
{
    return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

static uint8_t gpsDateTimeDaysInMonth(uint16_t year, uint8_t month)
{
    static const uint8_t daysInMonth[] = {
        31, 28, 31, 30, 31, 30,
        31, 31, 30, 31, 30, 31
    };

    if (month == 2 && gpsDateTimeIsLeapYear(year)) {
        return 29;
    }

    return daysInMonth[month - 1];
}

static void gpsDateTimeAddOneSecond(gpsDateTime_t *dt)
{
    if (++dt->sec <= 59) {
        return;
    }

    dt->sec = 0;
    if (++dt->min <= 59) {
        return;
    }

    dt->min = 0;
    if (++dt->hour <= 23) {
        return;
    }

    dt->hour = 0;
    if (++dt->day <= gpsDateTimeDaysInMonth(dt->year, dt->month)) {
        return;
    }

    dt->day = 1;
    if (++dt->month <= 12) {
        return;
    }

    dt->month = 1;
    dt->year++;
}

static void gpsDateTimeSubtractOneSecond(gpsDateTime_t *dt)
{
    if (dt->sec > 0) {
        dt->sec--;
        return;
    }

    dt->sec = 59;
    if (dt->min > 0) {
        dt->min--;
        return;
    }

    dt->min = 59;
    if (dt->hour > 0) {
        dt->hour--;
        return;
    }

    dt->hour = 23;
    if (dt->day > 1) {
        dt->day--;
        return;
    }

    if (dt->month > 1) {
        dt->month--;
    } else {
        dt->month = 12;
        dt->year--;
    }
    dt->day = gpsDateTimeDaysInMonth(dt->year, dt->month);
}

static void gpsDateTimeFromNavPvt(gpsDateTime_t *dt, const ubxNavPvt_t *navPvt)
{
    dt->valid = (navPvt->valid & NAV_VALID_DATE) && (navPvt->valid & NAV_VALID_TIME);
    if (!dt->valid) {
        return;
    }

    dt->year = navPvt->year;
    dt->month = navPvt->month;
    dt->day = navPvt->day;
    dt->hour = navPvt->hour;
    dt->min = navPvt->min;
    dt->sec = navPvt->sec;

    const int32_t nanoMs = navPvt->nano / 1000000;
    if (nanoMs < 0) {
        dt->millis = (uint16_t)(nanoMs + 1000);
        gpsDateTimeSubtractOneSecond(dt);
    } else if (nanoMs >= 1000) {
        dt->millis = (uint16_t)(nanoMs - 1000);
        gpsDateTimeAddOneSecond(dt);
    } else {
        dt->millis = (uint16_t)nanoMs;
    }
}

// Convert GPS week number and time-of-week to gpsDateTime_t
static void gpsWeekTimeToDateTime(gpsDateTime_t *dt, int16_t week, uint32_t timeOfWeekMs, int32_t timeOfWeekNs)
{
    int64_t gpsSeconds = (int64_t)week * 7 * 24 * 3600 + timeOfWeekMs / 1000;

    // Calculate milliseconds including nanosecond adjustment (can be negative)
    int32_t millis = (int32_t)(timeOfWeekMs % 1000) + timeOfWeekNs / 1000000;

    // Handle carry/borrow from nanosecond adjustment
    if (millis < 0) {
        millis += 1000;
        gpsSeconds--;
    } else if (millis >= 1000) {
        millis -= 1000;
        gpsSeconds++;
    }

    int64_t unixSeconds = gpsSeconds + GPS_EPOCH_OFFSET_SECONDS - GPS_LEAP_SECONDS;
    gpsUnixSecondsToDateTime(dt, unixSeconds, (uint16_t)millis);
}

#ifdef USE_RTC_TIME
// Set system clock once when GPS time is available
static void setRtcDateTimeFromGps(void) {
    if (!rtcHasTime() && gpsSol.dateTime.valid) {
        dateTime_t dt;
        dt.year = gpsSol.dateTime.year;
        dt.month = gpsSol.dateTime.month;
        dt.day = gpsSol.dateTime.day;
        dt.hours = gpsSol.dateTime.hour;
        dt.minutes = gpsSol.dateTime.min;
        dt.seconds = gpsSol.dateTime.sec;
        dt.millis = gpsSol.dateTime.millis;
        rtcSetDateTime(&dt);
    }
}
#endif

static void calculateNavInterval(void)
{
    // calculate the interval between nav packets, handling iTow wraparound at the end of the week
    const uint32_t weekDurationMs = 7 * 24 * 3600 * 1000;
    const uint32_t navDeltaTimeMs = (weekDurationMs + GpsNav.time - GpsNav.lastNavSolTs) % weekDurationMs;
    GpsNav.lastNavSolTs = GpsNav.time;
    // constrain the interval between 50ms / 20hz or 2.5s, when we would get a connection failure anyway
    GpsNav.navIntervalMs = constrain(navDeltaTimeMs, 50, 2500);
}

uint32_t posllh_dt, posllh_tmp, sat_dt, sat_tmp, status_dt, status_tmp, pvt_dt, pvt_tmp;
//#define GPS_FILTERING
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
        GpsNav.GPS_coord[LON] = posllh.lon;
        GpsNav.GPS_coord[LAT] = posllh.lat;
        GpsNav.altCm = posllh.height / 10;

        GpsNav.time = posllh.iTOW;
        //calculateNavInterval();

        GpsNav.eph = gpsConstrainEPE(posllh.hAcc / 10);
        GpsNav.epv = gpsConstrainEPE(posllh.vAcc / 10);

        GpsNav.flags.validEPE = true;

        //////////////////////////////////////////////////////
        gpsSol.llh.lon = posllh.lon;
        gpsSol.llh.lat = posllh.lat;
        gpsSol.llh.altCm = posllh.hMSL / 10;  //alt in cm
        gpsSol.time = posllh.iTOW;
        calculateNavInterval();
        gpsSetFixState(ubxHaveNewValidFix);
        ubxHaveNewPosition = true;
        //////////////////////////////////////////////////////
        #if defined(GPS_FILTERING)
          GPS_Filter(GpsNav.GPS_coord);
        #endif
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
            sat.sv[i].prRes = p[6] | (p[7]<<8);
            sat.sv[i].flags = p[8] | (p[9]<<8) | (p[10]<<16) | (p[11]<<24);
            p += 12;
            // 여기서 위성 정보 활용 가능 (신호 세기, PRN 등)
        }

        GPS_numCh = MIN(sat.numSvs, GPS_SV_MAXSATS_M8N);

        for (unsigned i = 0; i < ARRAYLEN(GPS_svinfo); i++) {
            if (i < GPS_numCh) {
                GPS_svinfo[i].chn = sat.sv[i].gnssId;
                GPS_svinfo[i].svid = sat.sv[i].svId;
                GPS_svinfo[i].cno = sat.sv[i].cno;
                GPS_svinfo[i].quality = sat.sv[i].flags;
            } else {
                GPS_svinfo[i] = (GPS_svinfo_t){ .chn = 255 };
            }
        }
        GPS_numCh = GPS_SV_MAXSATS_M8N;
        //GpsNav.GPS_numSat = sat.numSvs;

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

        ////////////////////////////////////////////////////////////////////////
        ubxHaveNewValidFix = (nav_status.fix_status & NAV_STATUS_FIX_VALID) && (nav_status.gpsFix == FIX_3D);
        if (!ubxHaveNewValidFix)
            DISABLE_STATE(GPS_FIX);
        ////////////////////////////////////////////////////////////////////////


    } else if (cls == 0x01 && id == 0x07) { // NAV-PVT
      if (length < 92) return;  // 최소 메시지 길이 확인

      pvt_dt = micros() - pvt_tmp;
      pvt_tmp = micros();

      pvt.iTOW = (payload[0]) | (payload[1]<<8) | (payload[2]<<16) | (payload[3]<<24);
      pvt.year = (uint16_t)(payload[4] | payload[5]<<8);
      pvt.month = payload[6];
      pvt.day = payload[7];
      pvt.hour = payload[8];
      pvt.min = payload[9];
      pvt.sec = payload[10];
      pvt.valid = payload[11];
      pvt.tAcc = (int32_t)(payload[12] | (payload[13]<<8) | (payload[14]<<16) | (payload[15]<<24));
      pvt.nano = (int32_t)(payload[16] | (payload[17]<<8) | (payload[18]<<16) | (payload[19]<<24));
      pvt.fixType = payload[20];
      pvt.numSV = payload[23];
      pvt.lon = (int32_t)(payload[24] | (payload[25]<<8) | (payload[26]<<16) | (payload[27]<<24));
      pvt.lat = (int32_t)(payload[28] | (payload[29]<<8) | (payload[30]<<16) | (payload[31]<<24));
      pvt.height = (int32_t)(payload[32] | (payload[33]<<8) | (payload[34]<<16) | (payload[35]<<24));
      pvt.hMSL = (int32_t)(payload[36] | (payload[37]<<8) | (payload[38]<<16) | (payload[39]<<24));
      pvt.hAcc = (uint32_t)(payload[40] | (payload[41]<<8) | (payload[42]<<16) | (payload[43]<<24));
      pvt.vAcc = (uint32_t)(payload[44] | (payload[45]<<8) | (payload[46]<<16) | (payload[47]<<24));
      pvt.velN = (int32_t)(payload[48] | (payload[49]<<8) | (payload[50]<<16) | (payload[51]<<24));
      pvt.velE = (int32_t)(payload[52] | (payload[53]<<8) | (payload[54]<<16) | (payload[55]<<24));
      pvt.velD = (int32_t)(payload[56] | (payload[57]<<8) | (payload[58]<<16) | (payload[59]<<24));
      pvt.gSpeed = (int32_t)(payload[60] | (payload[61]<<8) | (payload[62]<<16) | (payload[63]<<24));
      pvt.headMot = (int32_t)(payload[64] | (payload[65]<<8) | (payload[66]<<16) | (payload[67]<<24));
      pvt.sAcc = (int32_t)(payload[68] | (payload[69]<<8) | (payload[70]<<16) | (payload[71]<<24));
      pvt.headAcc = (int32_t)(payload[72] | (payload[73]<<8) | (payload[74]<<16) | (payload[75]<<24));
      pvt.pDOP = (uint16_t)(payload[76] | (payload[77]<<8));
      pvt.headVeh = (int32_t)(payload[84] | (payload[85]<<8) | (payload[86]<<16) | (payload[87]<<24));
    }
    GpsNav.time = pvt.iTOW;
    //calculateNavInterval();
    GpsNav.GPS_numSat = pvt.numSV;
    GpsNav.GPS_headVeh = pvt.headVeh;
    GpsNav.eph = gpsConstrainEPE(pvt.hAcc / 10);
    GpsNav.epv = gpsConstrainEPE(pvt.vAcc / 10);
    GpsNav.hdop = gpsConstrainHDOP(pvt.pDOP);
    GpsNav.flags.validVelNE = true;
    GpsNav.flags.validVelD = true;
    GpsNav.flags.validEPE = true;
    GpsNav.groundSpeed = pvt.gSpeed / 10; // cm/s
    GpsNav.groundCourse = (uint16_t) (pvt.headMot / 10000); // Heading 2D deg * 100000 rescaled to deg * 10
    GpsNav.velned.velN = (int16_t)(pvt.velN / 10); // cm/s
    GpsNav.velned.velE = (int16_t)(pvt.velE / 10); // cm/s
    GpsNav.velned.velD = (int16_t)(pvt.velD / 10); // cm/s
    GpsNav.ubxHaveNewPositionAndNewSpeed = true;

    ///////////////////////////////////////////////////////////
    ubxHaveNewValidFix = (pvt.flags & NAV_STATUS_FIX_VALID) && (pvt.fixType == FIX_3D);
    gpsSol.time = pvt.iTOW;
    calculateNavInterval();
    gpsSol.llh.lon = pvt.lon;
    gpsSol.llh.lat = pvt.lat;
    gpsSol.llh.altCm = pvt.hMSL / 10;  //alt in cm
    gpsSetFixState(ubxHaveNewValidFix);
    ubxHaveNewPosition = true;
    gpsSol.numSat = pvt.numSV;
    gpsSol.acc.hAcc = pvt.hAcc;
    gpsSol.acc.vAcc = pvt.vAcc;
    gpsSol.acc.sAcc = pvt.sAcc;
    gpsSol.acc.headAcc = pvt.headAcc;
    // gSpeed & velD are in mm/s (int32_t), gpsSol.speed3d in cm/s (uint16_t)
    float gs = (float)pvt.gSpeed;
    float vd = (float)pvt.velD;
    gpsSol.speed3d = (uint16_t)(sqrtf(sq(gs) + sq(vd)) * 0.1f);     // mm/s -> cm/s
    // Update 2D ground speed (mm/s -> cm/s) for NAV-PVT when NAV-VELNED is disabled
    gpsSol.groundSpeed = (uint16_t)(pvt.gSpeed / 10);    // cm/s
    gpsSol.groundCourse = (uint16_t)(pvt.headMot / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
    gpsSol.dop.pdop = pvt.pDOP;
    // NAV-PVT doesn't provide hDOP/vDOP, estimate from pDOP (pDOP >= hDOP/vDOP, so this is conservative)
    gpsSol.dop.hdop = pvt.pDOP;
    gpsSol.dop.vdop = pvt.pDOP;
    gpsSol.velned.velN = (int16_t)(pvt.velN / 10); // cm/s
    gpsSol.velned.velE = (int16_t)(pvt.velE / 10); // cm/s
    gpsSol.velned.velD = (int16_t)(pvt.velD / 10); // cm/s
    ubxHaveNewSpeed = true;
    // Store GPS date/time for telemetry, applying nano correction per u-blox spec.
    gpsDateTimeFromNavPvt(&gpsSol.dateTime, &pvt);
#ifdef USE_RTC_TIME
    setRtcDateTimeFromGps();
#endif
    ///////////////////////////////////////////////////////////
#ifdef USE_RTC_TIME
        //set clock, when gps time is available
        if (!rtcHasTime() && (_buffer.pvt.valid & NAV_VALID_DATE) && (_buffer.pvt.valid & NAV_VALID_TIME)) {
            dateTime_t dt;
            dt.year = _buffer.pvt.year;
            dt.month = _buffer.pvt.month;
            dt.day = _buffer.pvt.day;
            dt.hours = _buffer.pvt.hour;
            dt.minutes = _buffer.pvt.min;
            dt.seconds = _buffer.pvt.sec;
            dt.millis = (_buffer.pvt.nano > 0) ? _buffer.pvt.nano / 1000 : 0; //up to 5ms of error
            rtcSetDateTime(&dt);
        }
#endif
}

// Check for healthy communications
bool gpsIsHealthy(void)
{
    return (ubxHaveNewPositionAndNewSpeed);
}

void gpsSetFixState(bool state)
{
    if (state) {
        ENABLE_STATE(GPS_FIX);
    } else {
        DISABLE_STATE(GPS_FIX);
    }
}

//It was mobed here since even i2cgps code needs it
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
}

//reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav(void) {
  uint8_t i;

  for(i=0;i<2;i++) {
    GpsNav.nav_rated[i] = 0;
    GpsNav.nav[i] = 0;

    reset_PID(&GpsNav.posholdPID[i]);
    reset_PID(&GpsNav.poshold_ratePID[i]);
    reset_PID(&GpsNav.navPID[i]);
    GpsNav.nav_mode = NAV_MODE_NONE;
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold() {
  int32_t d;
  int32_t target_speed;
  uint8_t axis;

  for (axis=0;axis<2;axis++) {
    target_speed = get_P(GpsNav.error[axis], &posholdPID_PARAM); // calculate desired speed from lat/lon error
    target_speed = constrain(target_speed,-100,100);      // Constrain the target speed in poshold mode to 1m/s it helps avoid runaways..
    GpsNav.rate_error[axis] = target_speed - GpsNav.actual_speed[axis]; // calc the speed error

    GpsNav.nav[axis] = get_P(GpsNav.rate_error[axis], &poshold_ratePID_PARAM)
       + get_I(GpsNav.rate_error[axis] + GpsNav.error[axis], &GpsNav.dTnav, &GpsNav.poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = get_D(GpsNav.error[axis], &GpsNav.dTnav, &GpsNav.poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = constrain(d, -2000, 2000);
    // get rid of noise
    if(abs(GpsNav.actual_speed[axis]) < 50) d = 0;

    GpsNav.nav[axis] +=d;
    GpsNav.nav[axis]  = constrain(GpsNav.nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    GpsNav.navPID[axis].integrator = GpsNav.poshold_ratePID[axis].integrator;
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line
// when flying to a waypoint.
//
static void GPS_update_crosstrack(void) {
//  if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {  // If we are too far off or too close we don't do track following
//    float temp = (target_bearing - original_target_bearing) * RADX100;
//    crosstrack_error = sin(temp) * (wp_distance * CROSSTRACK_GAIN);  // Meters we are off track line
//    nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
//    nav_bearing = wrap_36000(nav_bearing);
//  }else{
//    nav_bearing = target_bearing;
//  }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void GPS_calc_nav_rate(uint16_t max_speed) {
  float trig[2];
  uint8_t axis;
  // push us towards the original track
  GPS_update_crosstrack();

//  // nav_bearing includes crosstrack
//  float temp = (9000l - nav_bearing) * RADX100;
//  trig[_X] = cos(temp);
//  trig[_Y] = sin(temp);
//
//  for (axis=0;axis<2;axis++) {
//    rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis];
//    rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
//    // P + I + D
//    nav[axis]      =
//        get_P(rate_error[axis],                        &navPID_PARAM)
//       +get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM)
//       +get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);
//
//    nav[axis]      = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
//    poshold_ratePID[axis].integrator = navPID[axis].integrator;
//  }
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp() {
  int32_t temp;
  temp = GpsNav.target_bearing - GpsNav.original_target_bearing;
  temp = wrap_18000(temp);
  return (abs(temp) > 10000);   // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow
// speed rampup when starting a navigation
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1.5 m/s as we hit the target
//
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow) {
  // max_speed is default 400 or 4m/s
  if(_slow){
    max_speed = min(max_speed, GpsNav.wp_distance / 2);
    //max_speed = max(max_speed, 0);
  }else{
    max_speed = min(max_speed, GpsNav.wp_distance);
    max_speed = max(max_speed, NAV_SPEED_MIN);  // go at least 100cm/s
  }

  // limit the ramp up of the speed
  // waypoint_speed_gov is reset to 0 at each new WP command
  if(max_speed > GpsNav.waypoint_speed_gov){
    GpsNav.waypoint_speed_gov += (int)(100.0 * GpsNav.dTnav); // increase at .5/ms
    max_speed = GpsNav.waypoint_speed_gov;
  }
  return max_speed;
}

static float GPS_cosLat = 1.0f;
    // this is used to offset the shrinking longitude as we go towards the poles
    // longitude difference * scale is approximate distance in degrees

void GPS_calc_longitude_scaling(int32_t lat)
{
    GPS_cosLat = cos_approx(DEGREES_TO_RADIANS((float)lat / GPS_DEGREES_DIVIDER));
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the distance flown from gps position data
//
static void GPS_calculateDistanceFlown(bool initialize)
{
    static gpsLocation_t lastLLH = {0};

    if (initialize) {
        GPS_distanceFlownInCm = 0;
    } else if (STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED)) {
        uint16_t speed = GpsNav.groundSpeed;
        // Only add up movement when speed is faster than minimum threshold
        if (speed > GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S) {
            uint32_t dist;
            GPS_distance_cm_bearing(&gpsSol.llh, &lastLLH, false, &dist, NULL);
            GPS_distanceFlownInCm += dist;
        }
    }
    lastLLH = gpsSol.llh;
}

void GPS_reset_home_position(void) {
  if (!STATE(GPS_FIX_HOME) || !GpsNav.gps_set_home_point_once) {
    if (STATE(GPS_FIX) && GpsNav.GPS_numSat >= 5) {
      GPS_home_llh = gpsSol.llh;
      GPS_calc_longitude_scaling(gpsSol.llh.lat);
      ENABLE_STATE(GPS_FIX_HOME);
      // no point beeping success here since:
      // when triggered by tryArm, the arming beep is modified to indicate the GPS home fix status on arming, and
      // when triggered by gyro cal, the gyro cal beep takes priority over the GPS beep, so we won't hear the GPS beep
      // PS: to test for gyro cal, check for !ARMED, since we cannot be here while disarmed other than via gyro cal
      GpsNav.GPS_home[LAT] = GpsNav.GPS_coord[LAT];
      GpsNav.GPS_home[LON] = GpsNav.GPS_coord[LON];
      GPS_calc_longitude_scaling(GpsNav.GPS_coord[LAT]);  //need an initial value for distance and bearing calc
      GpsNav.nav_takeoff_bearing = DECIDEGREES_TO_DEGREES(attitude.values.yaw);             //save takeoff heading
      //Set ground altitude

    }
  }
  GPS_calculateDistanceFlown(true); // Initialize
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm using spherical to Cartesian transform
// One one latitude unit, or one longitude unit at the equator, equals 1.113195 cm.
// Get bearing from pos1 to pos2, returns values with 0.01 degree precision
void GPS_distance_cm_bearing(const gpsLocation_t *from, const gpsLocation_t *to, bool dist3d, uint32_t *pDist, int32_t *pBearing)
{
    // TO DO : handle crossing the 180 degree meridian, as in `GPS_distance2d()`
    float dLat = (to->lat - from->lat) * EARTH_ANGLE_TO_CM;
    float dLon = (to->lon - from->lon) * GPS_cosLat * EARTH_ANGLE_TO_CM; // convert to local angle
    float dAlt = dist3d ? to->altCm - from->altCm : 0;

    if (pDist)
        *pDist = sqrtf(sq(dLat) + sq(dLon) + sq(dAlt));

    if (pBearing) {
        int32_t bearing = 9000.0f - RADIANS_TO_DEGREES(atan2_approx(dLat, dLon)) * 100.0f;      // Convert the output to 100xdeg / adjust to clockwise from North
        if (bearing < 0)
            bearing += 36000;
        *pBearing = bearing;
    }
}

static void GPS_calculateDistanceAndDirectionToHome(void)
{
    if (STATE(GPS_FIX_HOME)) {
        uint32_t dist;
        int32_t dir;
        GPS_distance_cm_bearing(&gpsSol.llh, &GPS_home_llh, false, &dist, &dir);
        GPS_distanceToHome = dist / 100; // m
        GPS_distanceToHomeCm = dist; // cm
        GPS_directionToHome = dir / 10; // degrees * 10 or decidegrees
    } else {
        // If we don't have home set, do not display anything
        GPS_distanceToHome = 0;
        GPS_distanceToHomeCm = 0;
        GPS_directionToHome = 0;
    }
}

// return distance vector in local, cartesian ENU coordinates
// note that parameter order is from, to
void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *distance)
{
    int32_t deltaLon = to->lon - from->lon;
    // In case we crossed the 180° meridian
    const int32_t deg180 = 180 * GPS_DEGREES_DIVIDER; // number of integer longitude steps in 180 degrees
    if (deltaLon > deg180) {
        deltaLon -= deg180;  // 360 * GPS_DEGREES_DIVIDER overflows int32_t, so use 180 twice
        deltaLon -= deg180;
    } else if (deltaLon <= -deg180) {
        deltaLon += deg180;
        deltaLon += deg180;
    }
    distance->x = deltaLon * GPS_cosLat * EARTH_ANGLE_TO_CM; // East-West distance, positive East
    distance->y = (float)(to->lat - from->lat) * EARTH_ANGLE_TO_CM;  // North-South distance, positive North
}

uint16_t gpsConstrainEPE(uint32_t epe)
{
    return (epe > 9999) ? 9999 : epe; // max 99.99m error
}

uint16_t gpsConstrainHDOP(uint32_t hdop)
{
    return (hdop > 9999) ? 9999 : hdop; // max 99.99m error
}

void gpsUpdate(uint32_t currentTimeUs)
{
  if(GpsNav.ubxHaveNewPositionAndNewSpeed == true)
  {
    GpsNav.ubxHaveNewPositionAndNewSpeed = false;
    if(STATE(GPS_FIX) && GpsNav.GPS_numSat >= 5)
    {
      // Set sensor as ready and available
      sensorsSet(SENSOR_GPS);
      GpsNav.Healthy = true;

      currentGpsStamp++; // new GPS data available

      gpsDataIntervalSeconds = GpsNav.navIntervalMs * 0.001f; // range for navIntervalMs is constrained to 50 - 2500
      gpsDataFrequencyHz = 1.0f / gpsDataIntervalSeconds;

      //dTnav calculation
      //Time for calculating x,y speed and navigation pids
      static uint32_t nav_loopTimer;
      GpsNav.dTnav = (float)(millis() - nav_loopTimer)/ 1000.0f;
      nav_loopTimer = millis();
      // prevent runup from bad GPS
      GpsNav.dTnav = MIN(GpsNav.dTnav, 1.0f);

      GPS_calculateDistanceAndDirectionToHome();

      if (ARMING_FLAG(ARMED)) {
        GPS_calculateDistanceFlown(false);
      }

      //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
      GPS_calc_velocity();

      if (STATE(NAV_POSHOLD_MODE) || STATE(NAV_RTH_MODE)){    //ok we are navigating
        //do gps nav calculations here, these are common for nav and poshold
        #if defined(GPS_LEAD_FILTER)
          GPS_distance_cm_bearing(&GPS_coord_lead[LAT],&GPS_coord_lead[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
          GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord_lead[LAT],&GPS_coord_lead[LON]);
        #else
          GPS_distance_cm_bearing(&GpsNav.GPS_coord[LAT],&GpsNav.GPS_coord[LON],&GpsNav.GPS_WP[LAT],&GpsNav.GPS_WP[LON],&GpsNav.wp_distance,&GpsNav.target_bearing);
          GPS_calc_location_error(&GpsNav.GPS_WP[LAT],&GpsNav.GPS_WP[LON],&GpsNav.GPS_coord[LAT],&GpsNav.GPS_coord[LON]);
        #endif
        switch (GpsNav.nav_mode) {
          case NAV_MODE_POSHOLD:
            //Desired output is in nav_lat and nav_lon where 1deg inclination is 100
            GPS_calc_poshold();
            break;
          case NAV_MODE_WP:
            int16_t speed = GPS_calc_desired_speed(NAV_SPEED_MAX, NAV_SLOW_NAV);      //slow navigation
            // use error as the desired rate towards the target
            //Desired output is in nav_lat and nav_lon where 1deg inclination is 100
            GPS_calc_nav_rate(speed);

            //Tail control
            if (NAV_CONTROLS_HEADING) {
              if (NAV_TAIL_FIRST) {
                magHold = wrap_18000(GpsNav.nav_bearing-18000)/100;
              } else {
                magHold = GpsNav.nav_bearing/100;
              }
            }
            // Are we there yet ?(within 2 meters of the destination)
            if ((GpsNav.wp_distance <= GpsNav.GPS_wp_radius) || check_missed_wp()){         //if yes switch to poshold mode
              GpsNav.nav_mode = NAV_MODE_POSHOLD;
              if (NAV_SET_TAKEOFF_HEADING) { magHold = GpsNav.nav_takeoff_bearing; }
            }
            break;
        }
      }
    }
  }

  DEBUG_SET(DEBUG_GPS_DATA, 0, (GpsNav.GPS_coord[LAT]));
  DEBUG_SET(DEBUG_GPS_DATA, 1, (GpsNav.GPS_coord[LON]));
  DEBUG_SET(DEBUG_GPS_DATA, 2, (GpsNav.GPS_home[LAT]));
  DEBUG_SET(DEBUG_GPS_DATA, 3, (GpsNav.GPS_home[LON]));
  DEBUG_SET(DEBUG_GPS_DATA, 4, (GPS_distanceToHomeCm));
  DEBUG_SET(DEBUG_GPS_DATA, 5, (GPS_directionToHome));
  DEBUG_SET(DEBUG_GPS_DATA, 6, (GpsNav.actual_speed[GPS_X]));
  DEBUG_SET(DEBUG_GPS_DATA, 7, (GPS_distanceFlownInCm));
}

bool isGPSHeadingValid(void)
{
    return sensors(SENSOR_GPS) && STATE(GPS_FIX) && GpsNav.GPS_numSat >= 6;// && gpsSol.groundSpeed >= 300;
}

// check if new data has been received since last check
// if client stamp is initialized to 0, gpsHasNewData will return false until first GPS position update
// if client stamp is initialized to ~0, gpsHasNewData will return true on first call
bool gpsHasNewData(uint16_t* stamp) {
    if (*stamp != currentGpsStamp) {
        *stamp = currentGpsStamp;
        return true;
    } else {
        return false;
    }
}

float getGpsDataIntervalSeconds(void)
{
    return gpsDataIntervalSeconds;
}

float getGpsDataFrequencyHz(void)
{
    return gpsDataFrequencyHz;
}
#endif
