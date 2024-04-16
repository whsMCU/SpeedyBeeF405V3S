/*
 * fatfs.c
 *
 *  Created on: 2020. 12. 25.
 *      Author: baram
 */


#include "fatfs.h"

#ifdef _USE_HW_FATFS
#include "ff_gen_drv.h"
#include "diskio.h"
#include <drivers/sdcard/sd_diskio.h>

#include "ini/iniparser.h"

static bool is_init = false;

FATFS SDFatFs;  /* File system object for SD card logical drive */
char SDPath[4]; /* SD card logical drive path */


#ifdef _USE_HW_CLI
static void cliFatfs(cli_args_t *args);
#endif

bool fatfsInit(void)
{
  bool ret = true;


  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
    {
      is_init = true;
    }
  }

#ifdef _USE_HW_CLI
  cliAdd("fatfs", cliFatfs);
#endif

  return ret;
}


#ifdef _USE_HW_CLI

FRESULT fatfsDir(char* path)
{
  FRESULT res;
  DIR dir;
  FILINFO fno;


  res = f_opendir(&dir, path);                       /* Open the directory */
  if (res == FR_OK)
  {
    for (;;)
    {
      res = f_readdir(&dir, &fno);                   /* Read a directory item */
      if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
      if (fno.fattrib & AM_DIR)
      {                    /* It is a directory */
        cliPrintf(" %s/%s \n\r", path, fno.fname);
      }
      else
      {                                       /* It is a file. */
        cliPrintf(" %s/%32s \t%d bytes\n\r", path, fno.fname, (int)fno.fsize);
      }
    }
    f_closedir(&dir);
  }

  return res;
}

static int parse_ini_file(char * ini_name)
{
    dictionary  *   ini ;

    /* Some temporary variables to hold query results */
    int             b ;
    int             i ;
    double          d ;
    const char  *   s ;

    ini = iniparser_load(ini_name);
    if (ini==NULL) {
        fprintf(stderr, "cannot parse file: %s\n", ini_name);
        return -1 ;
    }
    //iniparser_dump(ini, stderr);

    /* Get pizza attributes */
    cliPrintf("Pizza:\n\r");

    b = iniparser_getboolean(ini, "pizza:ham", -1);
    cliPrintf("Ham:       [%d]\n\r", b);
    b = iniparser_getboolean(ini, "pizza:mushrooms", -1);
    cliPrintf("Mushrooms: [%d]\n\r", b);
    b = iniparser_getboolean(ini, "pizza:capres", -1);
    cliPrintf("Capres:    [%d]\n\r", b);
    b = iniparser_getboolean(ini, "pizza:cheese", -1);
    cliPrintf("Cheese:    [%d]\n\r", b);

    /* Get wine attributes */
    cliPrintf("Wine:\n\r");
    s = iniparser_getstring(ini, "wine:grape", NULL);
    cliPrintf("Grape:     [%s]\n\r", s ? s : "UNDEF");

    i = iniparser_getint(ini, "wine:year", -1);
    cliPrintf("Year:      [%d]\n\r", i);

    s = iniparser_getstring(ini, "wine:country", NULL);
    cliPrintf("Country:   [%s]\n\r", s ? s : "UNDEF");

    d = iniparser_getdouble(ini, "wine:alcohol", -1.0);
    cliPrintf("Alcohol:   [%g]\n\r", d);

    iniparser_freedict(ini);
    return 0 ;
}

void cliFatfs(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info") == true)
  {
    cliPrintf("fatfs init \t: %d\n\r", is_init);

    if (is_init == true)
    {
      FATFS *fs;
       DWORD fre_clust, fre_sect, tot_sect;
       FRESULT res;

       /* Get volume information and free clusters of drive 1 */
       res = f_getfree("", &fre_clust, &fs);
       if (res == FR_OK)
       {
         /* Get total sectors and free sectors */
         tot_sect = (fs->n_fatent - 2) * fs->csize;
         fre_sect = fre_clust * fs->csize;

         /* Print the free space (assuming 512 bytes/sector) */
         cliPrintf("%10lu KiB total drive space.\n\r%10lu KiB available.\n\r", tot_sect / 2, fre_sect / 2);
       }
       else
       {
         cliPrintf(" err : %d\n\r", res);
       }
    }

    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "dir") == true)
  {
    FRESULT res;

    res = fatfsDir("/");
    if (res != FR_OK)
    {
      cliPrintf(" err : %d\n\r", res);
    }

    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "test") == true)
  {
    FRESULT fp_ret;
    FIL log_file;
    uint32_t pre_time;

    pre_time = millis();
    fp_ret = f_open(&log_file, "12.csv", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    if (fp_ret == FR_OK)
    {
      f_printf(&log_file, "test1, ");
      f_printf(&log_file, "test2, ");
      f_printf(&log_file, "test3, ");
      f_printf(&log_file, ", ");
      f_printf(&log_file, "\n\r");

      for (int i=0; i<256; i++)
      {
        f_printf(&log_file, "%d \n", i);
      }

      f_rewind(&log_file);


      UINT len;
      uint8_t data;

      while(cliKeepLoop())
      {
        len = 0;
        fp_ret = f_read(&log_file, &data, 1, &len);

        if (fp_ret != FR_OK)
        {
          break;
        }
        if (len == 0)
        {
          break;
        }

        cliPrintf("%c", data);
      }

      f_close(&log_file);
    }
    else
    {
      cliPrintf("f_open fail\r\n");
    }
    cliPrintf("%d ms\r\n", millis()-pre_time);

    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "ini") == true)
  {
    FRESULT fp_ret;
    FIL ini_file;
    uint32_t pre_time;

    pre_time = millis();
    fp_ret = f_open(&ini_file, "cfg.ini", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    if (fp_ret == FR_OK)
    {
      f_printf(&ini_file,
      "#\n"
      "# This is an example of ini file\n"
      "#\n"
      "\n"
      "[Pizza]\n"
      "\n"
      "Ham       = yes ;\n"
      "Mushrooms = TRUE ;\n"
      "Capres    = 0 ;\n"
      "Cheese    = Non ;\n"
      "\n"
      "\n"
      "[Wine]\n"
      "\n"
      "Grape     = Cabernet Sauvignon ;\n"
      "Year      = 1989 ;\n"
      "Country   = Spain ;\n"
      "Alcohol   = 12.5  ;\n"
      "\n");

      f_close(&ini_file);

      parse_ini_file("cfg.ini");

      //f_close(&ini_file);
    }
    else
    {
      cliPrintf("f_open fail\r\n");
    }
    cliPrintf("%d ms\r\n", millis()-pre_time);

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("fatfs info\n\r");
    cliPrintf("fatfs dir\n\r");
    cliPrintf("fatfs test\n\r");
    cliPrintf("fatfs ini\n\r");
  }
}

#endif



#endif
