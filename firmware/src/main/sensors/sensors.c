/*
 * sensor.c
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#include "filter.h"
#include "gyro_init.h"
#include "gyro.h"
//#include "scheduler/scheduler.h"
#include "common/time.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#include "sensors/sensors.h"


static bool is_init = false;

#ifdef _USE_HW_CLI
static void cliSensor(cli_args_t *args);
#endif

bool Sensor_Init(void)
{
	bool ret = true;

	is_init = bmi270_Init();
  
	if (is_init != true)
 	{
   		return false;
  	}

	#ifdef _USE_HW_CLI
  		cliAdd("Sensor", cliSensor);
	#endif

    return ret;
}

#ifdef _USE_HW_CLI
void cliSensor(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "test") == true)
  {
    while(cliKeepLoop())
    {
  
    }
	}

    ret = true;

  if (ret != true)
  {
    cliPrintf("lcd test\n");
    cliPrintf("lcd image\n");
  }
}
#endif
