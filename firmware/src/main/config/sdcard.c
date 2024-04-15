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
  sprintf(str, "%.1f", roll.in.kp);
  ret = iniparser_set(dictionary, "pid:roll.in.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", roll.in.ki);
  ret = iniparser_set(dictionary, "pid:roll.in.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", roll.in.kd);
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
  sprintf(str, "%.1f", roll.out.kp);
  ret = iniparser_set(dictionary, "pid:roll.out.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", roll.out.ki);
  ret = iniparser_set(dictionary, "pid:roll.out.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", roll.out.kd);
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
  sprintf(str, "%.1f", pitch.in.kp);
  ret = iniparser_set(dictionary, "pid:pitch.in.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", pitch.in.ki);
  ret = iniparser_set(dictionary, "pid:pitch.in.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", pitch.in.kd);
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
  sprintf(str, "%.1f", pitch.out.kp);
  ret = iniparser_set(dictionary, "pid:pitch.out.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", pitch.out.ki);
  ret = iniparser_set(dictionary, "pid:pitch.out.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", pitch.out.kd);
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
  sprintf(str, "%.1f", yaw_heading.kp);
  ret = iniparser_set(dictionary, "pid:yaw_heading.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", yaw_heading.ki);
  ret = iniparser_set(dictionary, "pid:yaw_heading.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", yaw_heading.kd);
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
  sprintf(str, "%.1f", yaw_rate.kp);
  ret = iniparser_set(dictionary, "pid:yaw_rate.kp", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", yaw_rate.ki);
  ret = iniparser_set(dictionary, "pid:yaw_rate.ki", str);
  if (ret < 0) {
      fprintf(stderr, "cannot set key/value in: %s\n", ini_name);
      ret = -1;
      goto free_dict;
  }

  sprintf(str, "%.1f", yaw_rate.kd);
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
  roll.in.kp = d;
  d = iniparser_getdouble(ini, "pid:roll.in.ki", 0.0);
  roll.in.ki = d;
  d = iniparser_getdouble(ini, "pid:roll.in.kd", 0.0);
  roll.in.kd = d;

  d = iniparser_getdouble(ini, "pid:roll.out.kp", 0.0);
  roll.out.kp = d;
  d = iniparser_getdouble(ini, "pid:roll.out.ki", 0.0);
  roll.out.ki = d;
  d = iniparser_getdouble(ini, "pid:roll.out.kd", 0.0);
  roll.out.kd = d;

  d = iniparser_getdouble(ini, "pid:pitch.in.kp", 0.0);
  pitch.in.kp = d;
  d = iniparser_getdouble(ini, "pid:pitch.in.ki", 0.0);
  pitch.in.ki = d;
  d = iniparser_getdouble(ini, "pid:pitch.in.kd", 0.0);
  pitch.in.kd = d;

  d = iniparser_getdouble(ini, "pid:pitch.out.kp", 0.0);
  pitch.out.kp = d;
  d = iniparser_getdouble(ini, "pid:pitch.out.ki", 0.0);
  pitch.out.ki = d;
  d = iniparser_getdouble(ini, "pid:pitch.out.kd", 0.0);
  pitch.out.kd = d;

  d = iniparser_getdouble(ini, "pid:yaw_heading.kp", 0.0);
  yaw_heading.kp = d;
  d = iniparser_getdouble(ini, "pid:yaw_heading.ki", 0.0);
  yaw_heading.ki = d;
  d = iniparser_getdouble(ini, "pid:yaw_heading.kd", 0.0);
  yaw_heading.kd = d;

  d = iniparser_getdouble(ini, "pid:yaw_rate.kp", 0.0);
  yaw_rate.kp = d;
  d = iniparser_getdouble(ini, "pid:yaw_rate.ki", 0.0);
  yaw_rate.ki = d;
  d = iniparser_getdouble(ini, "pid:yaw_rate.kd", 0.0);
  yaw_rate.kd = d;

  return true;
}

bool loadFromSDCard(void)
{
  FRESULT result;
  FIL ini_file;

  result= f_open(&ini_file, defaultSDCardConfigFilename, FA_READ);
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
  }

  return result;
}

bool readSDCard(void)
{
  bool result = parse_ini(defaultSDCardConfigFilename);

  return result;
}

