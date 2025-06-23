/*
 * SDCard.c
 *
 *  Created on: Apr 14, 2024
 *      Author: WANG
 */


#include "config/sdcard.h"

#include "ff_gen_drv.h"
#include "diskio.h"
#include <drivers/sdcard/sd_diskio.h>

#include "ini/iniparser.h"

#include "sensors/gyro.h"
#include "flight/pid.h"

#include "sensors/opflow.h"
#include "sensors/rangefinder.h"

const char *defaultSDCardConfigFilename = "config.ini";

static bool write_ini_roll_in(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%.1f", _ROLL.in.kp);
  ret = iniparser_set(dictionary, "pid:roll.in.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _ROLL.in.ki);
  ret = iniparser_set(dictionary, "pid:roll.in.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _ROLL.in.kd);
  ret = iniparser_set(dictionary, "pid:roll.in.kd", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
  return ret;
}

static bool write_ini_roll_out(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%.1f", _ROLL.out.kp);
  ret = iniparser_set(dictionary, "pid:roll.out.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _ROLL.out.ki);
  ret = iniparser_set(dictionary, "pid:roll.out.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _ROLL.out.kd);
  ret = iniparser_set(dictionary, "pid:roll.out.kd", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
      return ret;
}

static bool write_ini_pitch_in(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%.1f", _PITCH.in.kp);
  ret = iniparser_set(dictionary, "pid:pitch.in.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _PITCH.in.ki);
  ret = iniparser_set(dictionary, "pid:pitch.in.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _PITCH.in.kd);
  ret = iniparser_set(dictionary, "pid:pitch.in.kd", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
      return ret;
}

static bool write_ini_pitch_out(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%.1f", _PITCH.out.kp);
  ret = iniparser_set(dictionary, "pid:pitch.out.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _PITCH.out.ki);
  ret = iniparser_set(dictionary, "pid:pitch.out.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _PITCH.out.kd);
  ret = iniparser_set(dictionary, "pid:pitch.out.kd", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
      return ret;
}

static bool write_ini_yaw_heading(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%.1f", _YAW_Heading.kp);
  ret = iniparser_set(dictionary, "pid:yaw_heading.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _YAW_Heading.ki);
  ret = iniparser_set(dictionary, "pid:yaw_heading.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _YAW_Heading.kd);
  ret = iniparser_set(dictionary, "pid:yaw_heading.kd", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
      return ret;
}

static bool write_ini_yaw_rate(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%.1f", _YAW_Rate.kp);
  ret = iniparser_set(dictionary, "pid:yaw_rate.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _YAW_Rate.ki);
  ret = iniparser_set(dictionary, "pid:yaw_rate.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _YAW_Rate.kd);
  ret = iniparser_set(dictionary, "pid:yaw_rate.kd", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
      return ret;
}

static bool write_ini_acc_offset(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%d", bmi270.accelerationTrims.values.roll);
  ret = iniparser_set(dictionary, "gyro:accOffset.roll", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%d", bmi270.accelerationTrims.values.pitch);
  ret = iniparser_set(dictionary, "gyro:accOffset.pitch", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%d", bmi270.accelerationTrims.values.yaw);
  ret = iniparser_set(dictionary, "gyro:accOffset.yaw", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
      return ret;
}

static bool write_ini_alt(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%.1f", _ALT.kp);
  ret = iniparser_set(dictionary, "pid:alt.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _ALT.ki);
  ret = iniparser_set(dictionary, "pid:alt.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", _ALT.kd);
  ret = iniparser_set(dictionary, "pid:alt.kd", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
      return ret;
}

static bool write_ini_alt_range(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%.1f", rangefinder.althold.KP);
  ret = iniparser_set(dictionary, "pid:alt.range.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", rangefinder.althold.KI);
  ret = iniparser_set(dictionary, "pid:alt.range.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", rangefinder.althold.KD);
  ret = iniparser_set(dictionary, "pid:alt.range.kd", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
      return ret;
}

static bool write_ini_pos_opflow(const char * ini_name)
{
  void *dictionary;
  FIL ini_file;
  FRESULT fp_ret;
  int ret = 0;

  char str[20];

  if (!ini_name) {
      fprintf(stderr, "Invalid argurment\n");
      return -1;
  }

  dictionary = iniparser_load(ini_name);
  if (!dictionary) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }

  /* set key/value pair */
  sprintf(str, "%.1f", opflow.poshold.KP);
  ret = iniparser_set(dictionary, "pid:pos.opflow.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", opflow.poshold.KI);
  ret = iniparser_set(dictionary, "pid:pos.opflow.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", opflow.poshold.KD);
  ret = iniparser_set(dictionary, "pid:pos.opflow.kd", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  fp_ret = f_open(&ini_file, ini_name, FA_READ | FA_WRITE);

  if (fp_ret != FR_OK) {
      fprintf(stderr, "iniparser: cannot create example.ini\n");
      ret = -1;
      goto free_dict;
  }

  iniparser_dump_ini(dictionary, &ini_file);
  f_close(&ini_file);

  free_dict:
      iniparser_freedict(dictionary);
      return ret;
}

static bool parse_ini(const char * ini_name)
{
  dictionary  *   ini ;

  /* Some temporary variables to hold query results */
  //int             b ;
  int             i ;
  double          d ;
  //const char  *   s ;

  ini = iniparser_load(ini_name);
  if (ini==NULL) {
      fprintf(stderr, "cannot parse file: %s\n", ini_name);
      return -1 ;
  }
  //iniparser_dump(ini, stderr);

  i = iniparser_getint(ini, "gyro:accOffset.roll", 0);
  bmi270.accelerationTrims.values.roll = i;
  i = iniparser_getint(ini, "gyro:accOffset.pitch", 0);
  bmi270.accelerationTrims.values.pitch = i;
  i = iniparser_getint(ini, "gyro:accOffset.yaw", 0);
  bmi270.accelerationTrims.values.yaw = i;

  d = iniparser_getdouble(ini, "pid:roll.in.kp", 0.0);
  _ROLL.in.kp = d;
  d = iniparser_getdouble(ini, "pid:roll.in.ki", 0.0);
  _ROLL.in.ki = d;
  d = iniparser_getdouble(ini, "pid:roll.in.kd", 0.0);
  _ROLL.in.kd = d;

  d = iniparser_getdouble(ini, "pid:roll.out.kp", 0.0);
  _ROLL.out.kp = d;
  d = iniparser_getdouble(ini, "pid:roll.out.ki", 0.0);
  _ROLL.out.ki = d;
  d = iniparser_getdouble(ini, "pid:roll.out.kd", 0.0);
  _ROLL.out.kd = d;

  d = iniparser_getdouble(ini, "pid:pitch.in.kp", 0.0);
  _PITCH.in.kp = d;
  d = iniparser_getdouble(ini, "pid:pitch.in.ki", 0.0);
  _PITCH.in.ki = d;
  d = iniparser_getdouble(ini, "pid:pitch.in.kd", 0.0);
  _PITCH.in.kd = d;

  d = iniparser_getdouble(ini, "pid:pitch.out.kp", 0.0);
  _PITCH.out.kp = d;
  d = iniparser_getdouble(ini, "pid:pitch.out.ki", 0.0);
  _PITCH.out.ki = d;
  d = iniparser_getdouble(ini, "pid:pitch.out.kd", 0.0);
  _PITCH.out.kd = d;

  d = iniparser_getdouble(ini, "pid:yaw_heading.kp", 0.0);
  _YAW_Heading.kp = d;
  d = iniparser_getdouble(ini, "pid:yaw_heading.ki", 0.0);
  _YAW_Heading.ki = d;
  d = iniparser_getdouble(ini, "pid:yaw_heading.kd", 0.0);
  _YAW_Heading.kd = d;

  d = iniparser_getdouble(ini, "pid:yaw_rate.kp", 0.0);
  _YAW_Rate.kp = d;
  d = iniparser_getdouble(ini, "pid:yaw_rate.ki", 0.0);
  _YAW_Rate.ki = d;
  d = iniparser_getdouble(ini, "pid:yaw_rate.kd", 0.0);
  _YAW_Rate.kd = d;

  d = iniparser_getdouble(ini, "pid:alt.kp", 0.0);
  _ALT.kp = d;
  d = iniparser_getdouble(ini, "pid:alt.ki", 0.0);
  _ALT.ki = d;
  d = iniparser_getdouble(ini, "pid:alt.kd", 0.0);
  _ALT.kd = d;

  d = iniparser_getdouble(ini, "pid:alt.range.kp", 0.0);
  rangefinder.althold.KP = d;
  d = iniparser_getdouble(ini, "pid:alt.range.ki", 0.0);
  rangefinder.althold.KI = d;
  d = iniparser_getdouble(ini, "pid:alt.range.kd", 0.0);
  rangefinder.althold.KD = d;

  d = iniparser_getdouble(ini, "pid:pos.opflow.kp", 0.0);
  opflow.poshold.KP = d;
  d = iniparser_getdouble(ini, "pid:pos.opflow.ki", 0.0);
  opflow.poshold.KI = d;
  d = iniparser_getdouble(ini, "pid:pos.opflow.kd ", 0.0);
  opflow.poshold.KD = d;

  return true;
}

bool loadFromSDCard(void)
{
  FRESULT result;
  FIL ini_file;

  result= f_open(&ini_file, defaultSDCardConfigFilename, FA_READ | FA_WRITE);
//  if (result == FR_OK)
//  {
//    f_printf(&ini_file,
//    "[gyro]\n"
//    "\n"
//    "accOffset.roll  = 29 ;\n"
//    "accOffset.pitch = -35 ;\n"
//    "accOffset.yaw   = -9 ;\n"
//    "\n"
//    "[pid]\n"
//    "\n"
//    "roll.in.kp = 3 ;\n"
//    "roll.in.ki = 2 ;\n"
//    "roll.in.kd = 0 ;\n"
//    "\n"
//    "roll.out.kp = 20 ;\n"
//    "roll.out.ki = 1.5 ;\n"
//    "roll.out.kd = 0 ;\n"
//    "\n"
//    "pitch.in.kp = 3 ;\n"
//    "pitch.in.ki = 2 ;\n"
//    "pitch.in.kd = 0 ;\n"
//    "\n"
//    "pitch.out.kp = 20 ;\n"
//    "pitch.out.ki = 1.5 ;\n"
//    "pitch.out.kd = 0 ;\n"
//    "\n"
//    "yaw_heading.kp = 25 ;\n"
//    "yaw_heading.ki = 0 ;\n"
//    "yaw_heading.kd = 0 ;\n"
//    "\n"
//    "yaw_rate.kp = 10 ;\n"
//    "yaw_rate.ki = 0 ;\n"
//    "yaw_rate.kd = 0 ;\n"
//    "\n");
//  }
  f_close(&ini_file);
  if(result != FR_OK)
  {
    return false;
  }
  return true;
}

bool writeSDCard(uint8_t type)
{
  bool result = 0;
  switch(type)
  {
    case PID_Roll_in:
      result = write_ini_roll_in(defaultSDCardConfigFilename);
      break;
    case PID_Roll_out:
      result = write_ini_roll_out(defaultSDCardConfigFilename);
      break;
    case PID_pitch_in:
      result = write_ini_pitch_in(defaultSDCardConfigFilename);
      break;
    case PID_pitch_out:
      result = write_ini_pitch_out(defaultSDCardConfigFilename);
      break;
    case PID_yaw_heading:
      result = write_ini_yaw_heading(defaultSDCardConfigFilename);
      break;
    case PID_yaw_rate:
      result = write_ini_yaw_rate(defaultSDCardConfigFilename);
      break;
    case ACC_offset:
      result = write_ini_acc_offset(defaultSDCardConfigFilename);
      break;
    case PID_ALT:
      result = write_ini_alt(defaultSDCardConfigFilename);
      break;

    case PID_ALT_Range:
      result = write_ini_alt_range(defaultSDCardConfigFilename);
      break;

    case PID_POS_Opflow:
      result = write_ini_pos_opflow(defaultSDCardConfigFilename);
      break;
  }

  return result;
}

bool readSDCard(void)
{
  bool result = parse_ini(defaultSDCardConfigFilename);

  return result;
}

