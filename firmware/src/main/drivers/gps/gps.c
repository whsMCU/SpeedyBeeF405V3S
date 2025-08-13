/*
 * M8N.c
 *
 *  Created on: Aug 22, 2019
 *      Author: Administrator
 */

#include "hw.h"
#ifdef USE_GPS

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

uint8_t gps_set_home_point_once;

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

void gpsInit(void)
{
	uartOpen(_DEF_UART6, 9600);

	uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_PRT[0], sizeof(UBX_CFG_PRT));
	HAL_Delay(100);
	uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_MSG_POSLLH[0], sizeof(UBX_CFG_MSG_POSLLH));
	HAL_Delay(100);
  uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_MSG_SAT[0], sizeof(UBX_CFG_MSG_SAT));
  HAL_Delay(100);
  uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_MSG_STATUS[0], sizeof(UBX_CFG_MSG_SAT));
  HAL_Delay(100);
  uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_MSG_PVT[0], sizeof(UBX_CFG_MSG_PVT));
  HAL_Delay(100);
	uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_RATE[0], sizeof(UBX_CFG_RATE));
	HAL_Delay(100);
	uartWrite(_DEF_UART6, (uint8_t*)&UBX_CFG_CFG[0], sizeof(UBX_CFG_CFG));

	GpsNav.nav_mode = NAV_MODE_NONE;
	GpsNav.GPS_wp_radius = GPS_WP_RADIUS;
	gps_set_home_point_once = false;

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
nav_pvt_t pvt;

static bool next_fix;

uint32_t posllh_dt, posllh_tmp, sat_dt, sat_tmp, status_dt, status_tmp;
#define GPS_FILTERING
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
            sat.sv[i].prRes = p[6] | (p[7]<<8) | (p[8]<<16) | (p[9]<<24);
            p += 12;
            // 여기서 위성 정보 활용 가능 (신호 세기, PRN 등)
        }
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
    } else if (cls == 0x01 && id == 0x07) { // NAV-PVT
      if (length < 92) return;  // 최소 메시지 길이 확인

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
      pvt.gSpeed = (int32_t)(payload[60] | (payload[61]<<8) | (payload[62]<<16) | (payload[63]<<24));
      pvt.headMot = (int32_t)(payload[64] | (payload[65]<<8) | (payload[66]<<16) | (payload[67]<<24));
      pvt.pDOP = (uint16_t)(payload[76] | (payload[77]<<8));
      pvt.headVeh = (int32_t)(payload[84] | (payload[85]<<8) | (payload[86]<<16) | (payload[87]<<24));
    }
    GpsNav.GPS_numSat = pvt.numSV;
    GpsNav.GPS_headVeh = pvt.headVeh;
    GpsNav.groundSpeed = pvt.gSpeed / 10; // cm/s
    GpsNav.groundCourse = (uint16_t) (pvt.headMot / 10000); // Heading 2D deg * 100000 rescaled to deg * 10

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

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles
void GPS_calc_longitude_scaling(int32_t lat) {
  float rads = (fabsf((float)lat) / 10000000.0f) * 0.0174532925f;
  GPS_scaleLonDown = cos_approx(rads);
}

//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
//*******************************************************************************************************
static void GPS_calc_velocity(){
  static int16_t speed_old[2] = {0,0};
  static int32_t last[2] = {0,0};
  static uint8_t init = 0;

  if (init) {
    float tmp = 1.0/GpsNav.dTnav;
    GpsNav.actual_speed[GPS_X] = (float)(GpsNav.GPS_coord[LON] - last[LON]) *  GPS_scaleLonDown * tmp;
    GpsNav.actual_speed[GPS_Y] = (float)(GpsNav.GPS_coord[LAT]  - last[LAT])  * tmp;

#if !defined(GPS_LEAD_FILTER)
    GpsNav.actual_speed[GPS_X] = (GpsNav.actual_speed[GPS_X] + speed_old[GPS_X]) / 2;
    GpsNav.actual_speed[GPS_Y] = (GpsNav.actual_speed[GPS_Y] + speed_old[GPS_Y]) / 2;

    speed_old[GPS_X] = GpsNav.actual_speed[GPS_X];
    speed_old[GPS_Y] = GpsNav.actual_speed[GPS_Y];

#endif
  }
  init=1;

  last[LON] = GpsNav.GPS_coord[LON];
  last[LAT] = GpsNav.GPS_coord[LAT];

#if defined(GPS_LEAD_FILTER)
  GPS_coord_lead[LON] = xLeadFilter.get_position(GpsNav.GPS_coord[LON], GpsNav.actual_speed[GPS_X], GPS_LAG);
  GPS_coord_lead[LAT] = yLeadFilter.get_position(GpsNav.GPS_coord[LAT], GpsNav.actual_speed[GPS_Y], GPS_LAG);
#endif

}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng ) {
  GpsNav.error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;  // X Error
  GpsNav.error[LAT] = *target_lat - *gps_lat; // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t* lat, int32_t* lon) {
  GpsNav.GPS_WP[LAT] = *lat;
  GpsNav.GPS_WP[LON] = *lon;

  GPS_calc_longitude_scaling(*lat);
  GPS_distance_cm_bearing(&GpsNav.GPS_coord[LAT],&GpsNav.GPS_coord[LON],&GpsNav.GPS_WP[LAT],&GpsNav.GPS_WP[LON],&GpsNav.wp_distance,&GpsNav.target_bearing);

  GpsNav.nav_bearing = GpsNav.target_bearing;
  GPS_calc_location_error(&GpsNav.GPS_WP[LAT],&GpsNav.GPS_WP[LON],&GpsNav.GPS_coord[LAT],&GpsNav.GPS_coord[LON]);
  GpsNav.original_target_bearing = GpsNav.target_bearing;
  GpsNav.waypoint_speed_gov = NAV_SPEED_MIN;
}

////////////////////////////////////////////////////////////////////////////////////
#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS 1.113195f
#define TAN_89_99_DEGREES 5729.57795f
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing)
{
    float dLat = *destinationLat2 - *currentLat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*destinationLon2 - *currentLon1) * GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS;

    *bearing = 9000.0f + atan2_approx(-dLat, dLon) * TAN_89_99_DEGREES;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the distance flown and vertical speed from gps position data
//
//static void GPS_calculateDistanceFlownVerticalSpeed(bool initialize)
//{
//    static int32_t lastCoord[2] = { 0, 0 };
//    static int32_t lastAlt;
//    static int32_t lastMillis;
//
//    int currentMillis = millis();
//
//    if (initialize) {
//      GpsNav.GPS_distanceFlownInCm = 0;
//      GpsNav.GPS_verticalSpeedInCmS = 0;
//    } else {
//        if (STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED)) {
//            uint16_t speed = gpsConfig()->gps_use_3d_speed ? gpsSol.speed3d : gpsSol.groundSpeed;
//            // Only add up movement when speed is faster than minimum threshold
//            if (speed > GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S) {
//                uint32_t dist;
//                int32_t dir;
//                GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &lastCoord[GPS_LATITUDE], &lastCoord[GPS_LONGITUDE], &dist, &dir);
//                if (gpsConfig()->gps_use_3d_speed) {
//                    dist = sqrtf(powf(gpsSol.llh.altCm - lastAlt, 2.0f) + powf(dist, 2.0f));
//                }
//                GPS_distanceFlownInCm += dist;
//            }
//        }
//        GPS_verticalSpeedInCmS = (gpsSol.llh.altCm - lastAlt) * 1000 / (currentMillis - lastMillis);
//        GPS_verticalSpeedInCmS = constrain(GPS_verticalSpeedInCmS, -1500, 1500);
//    }
//    lastCoord[GPS_LONGITUDE] = gpsSol.llh.lon;
//    lastCoord[GPS_LATITUDE] = gpsSol.llh.lat;
//    lastAlt = gpsSol.llh.altCm;
//    lastMillis = currentMillis;
//}

void GPS_reset_home_position(void) {
  if (STATE(GPS_FIX) && GpsNav.GPS_numSat >= 5) {
    GpsNav.GPS_home[LAT] = GpsNav.GPS_coord[LAT];
    GpsNav.GPS_home[LON] = GpsNav.GPS_coord[LON];
    GPS_calc_longitude_scaling(GpsNav.GPS_coord[LAT]);  //need an initial value for distance and bearing calc
    GpsNav.nav_takeoff_bearing = DECIDEGREES_TO_DEGREES(attitude.values.yaw);             //save takeoff heading
    //Set ground altitude
    ENABLE_STATE(GPS_FIX_HOME);
  }
  //GPS_calculateDistanceFlownVerticalSpeed(true); //Initialize
}

void gpsSetFixState(bool state)
{
    if (state) {
        ENABLE_STATE(GPS_FIX);
        ENABLE_STATE(GPS_FIX_EVER);
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

void GPS_calculateDistanceAndDirectionToHome(void)
{
  //calculate distance and bearings for gui and other stuff continously - From home to copter
    if (STATE(GPS_FIX_HOME)) {      // If we don't have home set, do not display anything
        uint32_t dist;
        int32_t dir;
        GPS_distance_cm_bearing(&GpsNav.GPS_coord[LAT],&GpsNav.GPS_coord[LON],&GpsNav.GPS_home[LAT],&GpsNav.GPS_home[LON],&dist,&dir);
        GpsNav.GPS_distanceToHome = dist/100;
        GpsNav.GPS_directionToHome = dir/100;
    } else {
      GpsNav.GPS_distanceToHome = 0;
      GpsNav.GPS_directionToHome = 0;
    }
}

void gpsUpdate(uint32_t currentTimeUs)
{
  if(STATE(GPS_FIX) && GpsNav.GPS_numSat >= 5)
  {
    if(!FLIGHT_MODE(GPS_HOME_MODE) && ARMING_FLAG(ARMED))    //if home is not set set home position to WP#0 and activate it
    {
      GPS_reset_home_position();
    }

    //dTnav calculation
    //Time for calculating x,y speed and navigation pids
    static uint32_t nav_loopTimer;
    GpsNav.dTnav = (float)(millis() - nav_loopTimer)/ 1000.0f;
    nav_loopTimer = millis();
    // prevent runup from bad GPS
    GpsNav.dTnav = MIN(GpsNav.dTnav, 1.0f);

    GPS_calculateDistanceAndDirectionToHome();

  //  if (ARMING_FLAG(ARMED)) {
  //      GPS_calculateDistanceFlownVerticalSpeed(false);
  //  }

    //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
    GPS_calc_velocity();

    if (STATE(GPS_HOLD_MODE) || STATE(GPS_HOME_MODE)){    //ok we are navigating
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

bool isGPSHeadingValid(void)
{
    return sensors(SENSOR_GPS) && STATE(GPS_FIX) && GpsNav.GPS_numSat >= 6;// && gpsSol.groundSpeed >= 300;
}

#endif
