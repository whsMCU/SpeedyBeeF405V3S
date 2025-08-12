/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPS_H
#define __GPS_H
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

#define GPS_DEGREES_DIVIDER 10000000L
#define GPS_X 1
#define GPS_Y 0

#define GPS_LAG 0.5f                          //UBLOX GPS has a smaller lag than MTK and other

/************************        AP FlightMode        **********************************/
/* Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks.*/
#define AP_MODE 40  // Create a deadspan for GPS.

#define TASK_GPS_RATE       100
#define TASK_GPS_RATE_FAST  1000

#define RADX100                    0.000174532925
#define CROSSTRACK_GAIN            1
#define NAV_SPEED_MIN              100    // cm/sec
#define NAV_SPEED_MAX              300    // cm/sec
#define NAV_SLOW_NAV               true
#define NAV_BANK_MAX 3000        //30deg max banking when navigating (just for security and testing)

#define GPS_WP_RADIUS              200       // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
#define NAV_SLEW_RATE              30        // Adds a rate control to nav output, will smoothen out nav angle spikes

/* GPS navigation can control the heading */

#define NAV_CONTROLS_HEADING       true      // copter faces toward the navigation point, maghold must be enabled for it
#define NAV_TAIL_FIRST             false     // true - copter comes in with tail first
#define NAV_SET_TAKEOFF_HEADING    true      // true - when copter arrives to home position it rotates it's head to takeoff direction

#define GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S 15 // 5.4Km/h 3.35mph


//#define GPS_LEAD_FILTER                      // Adds a forward predictive filterig to compensate gps lag. Code based on Jason Short's lead filter implementation

//#define GPS_FILTERING                        // add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency comment out to disable

void gpsInit(void);
void gpsUpdate(uint32_t currentTimeUs);

#define GPS_FILTER_VECTOR_LENGTH 5

typedef struct {
  uint8_t GPS_filter_index;
  int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
  int32_t GPS_filter_sum[2];
  int32_t GPS_read[2];
  int32_t GPS_filtered[2];
  int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
  uint16_t fraction3[2];
} GpsNavFilter_t;

typedef struct PID_ {
  float   integrator; // integrator value
  int32_t last_input; // last input for derivative
  float   lastderivative; // last derivative for low-pass filter
  float   output;
  float   derivative;
} GPS_PID_t;

typedef struct {
  GpsNavFilter_t GPS_filter;
  uint8_t nav_mode;
  int32_t  GPS_coord[2];
  int32_t  GPS_home[2];
  int32_t  GPS_hold[2];
  uint8_t  GPS_numSat;
  int32_t  GPS_headVeh;
  uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
  int16_t  GPS_directionToHome;                         // direction to home - unit: degree
  uint32_t GPS_distanceFlownInCm;                       // distance flown since armed in centimeters
  uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
  uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  int16_t GPS_verticalSpeedInCmS;                        // vertical speed    - unit: cm/s
  uint8_t  GPS_update;                              // a binary toogle to distinct a GPS position update
  int16_t  GPS_angle[2];                      // the angles that must be applied for GPS correction
  uint16_t GPS_ground_course;                       //                   - unit: degree*10
  uint16_t GPS_wp_radius;
  int16_t nav_takeoff_bearing;
  int16_t nav[2];
  int16_t nav_rated[2];                      //Adding a rate controller to the navigation to make it smoother
  int32_t GPS_WP[2];                        //Currently used WP
  uint32_t wp_distance;
  int32_t target_bearing;                     // This is the angle from the copter to the "next_WP" location in degrees * 100
  int32_t nav_bearing;                        // with the addition of Crosstrack error in degrees * 100
  int32_t original_target_bearing;          // deg * 100, The original angle to the next_WP when the next_WP was set Also used to check when we pass a WP
  uint16_t waypoint_speed_gov;              // used for slow speed wind up when start navigation;
  int16_t rate_error[2];
  int32_t error[2];
  float  dTnav;                             // Delta Time in milliseconds for navigation computations, updated with every good GPS read
  int16_t actual_speed[2];
  int32_t altCm;                            // altitude in 0.01m

  GPS_PID_t posholdPID[2];
  GPS_PID_t poshold_ratePID[2];
  GPS_PID_t navPID[2];

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

typedef struct {
  uint32_t iTOW;        // GPS Time of Week (ms)
  uint16_t year;        // Year (UTC)
  uint8_t  month;       // Month (1-12, UTC)
  uint8_t  day;         // Day (1-31, UTC)
  uint8_t  hour;        // Hour (0-23, UTC)
  uint8_t  min;         // Minute (0-59, UTC)
  uint8_t  sec;         // Second (0-60, UTC, leap seconds)
  uint8_t  valid;       // Validity Flags (bitfield)
  uint32_t tAcc;        // Time accuracy (ns)
  int32_t  nano;        // Fraction of second (ns)
  uint8_t  fixType;     // GNSS Fix Type
  uint8_t  flags;       // Fix status flags (bitfield)
  uint8_t  flags2;      // Additional flags
  uint8_t  numSV;       // Number of satellites used
  int32_t  lon;         // Longitude (1e-7 deg)
  int32_t  lat;         // Latitude (1e-7 deg)
  int32_t  height;      // Height above ellipsoid (mm)
  int32_t  hMSL;        // Height above mean sea level (mm)
  uint32_t hAcc;        // Horizontal accuracy (mm)
  uint32_t vAcc;        // Vertical accuracy (mm)
  int32_t  velN;        // NED North velocity (mm/s)
  int32_t  velE;        // NED East velocity (mm/s)
  int32_t  velD;        // NED Down velocity (mm/s)
  int32_t  gSpeed;      // Ground speed (mm/s)
  int32_t  headMot;     // Heading of motion (1e-5 deg)
  uint32_t sAcc;        // Speed accuracy estimate (mm/s)
  uint32_t headAcc;     // Heading accuracy estimate (1e-5 deg)
  uint16_t pDOP;        // Position DOP (0.01 scale)
  uint8_t  reserved1[6];// Reserved
  int32_t  headVeh;     // Heading of vehicle (1e-5 deg)
  int16_t  magDec;      // Magnetic declination (1e-2 deg)
  uint16_t magAcc;      // Magnetic declination accuracy (1e-2 deg)
} nav_pvt_t;

extern UbxNavPosllh_t posllh;
extern UbxNavSat_t sat;

extern nav_pvt_t pvt;

extern UbxNavStatus_t nav_status;

void Ubx_HandleMessage(uint8_t cls, uint8_t id, uint8_t *payload, uint16_t length);

void GPS_calc_longitude_scaling(int32_t lat);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing);
void GPS_reset_home_position(void);
void gpsSetFixState(bool state);
int32_t wrap_18000(int32_t ang);
void GPS_reset_nav(void);

bool isGPSHeadingValid(void);

#ifdef __cplusplus
}
#endif
#endif /*__ GPS_H */
