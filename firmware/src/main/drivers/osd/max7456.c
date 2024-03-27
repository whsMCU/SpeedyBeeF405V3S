/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hw.h"

#ifdef USE_MAX7456

#include "build/debug.h"
#include "build/version.h"

#include "common/printf.h"

//#include "pg/max7456.h"
//#include "pg/vcd.h"
#include "osd/osd.h"

//#include "drivers/bus_spi.h"
//#include "drivers/dma.h"
//#include "drivers/io.h"
//#include "drivers/light_led.h"
#include "drivers/osd/max7456.h"
//#include "drivers/nvic.h"
#include "drivers/osd/osd.h"
#include "drivers/osd/osd_symbols.h"
//#include "drivers/time.h"

// 10 MHz max SPI frequency
#define MAX7456_MAX_SPI_CLK_HZ 10000000
#define MAX7456_INIT_MAX_SPI_CLK_HZ 5000000

// DEBUG_MAX7456_SIGNAL
#define DEBUG_MAX7456_SIGNAL_MODEREG       0
#define DEBUG_MAX7456_SIGNAL_SENSE         1
#define DEBUG_MAX7456_SIGNAL_REINIT        2
#define DEBUG_MAX7456_SIGNAL_ROWS          3

// DEBUG_MAX7456_SPICLOCK
#define DEBUG_MAX7456_SPICLOCK_OVERCLOCK   0
#define DEBUG_MAX7456_SPICLOCK_DEVTYPE     1
#define DEBUG_MAX7456_SPICLOCK_DIVISOR     2
#define DEBUG_MAX7456_SPICLOCK_X100        3

// VM0 bits
#define VIDEO_BUFFER_DISABLE        0x01
#define MAX7456_RESET               0x02
#define VERTICAL_SYNC_NEXT_VSYNC    0x04
#define OSD_ENABLE                  0x08

#define SYNC_MODE_AUTO              0x00
#define SYNC_MODE_INTERNAL          0x30
#define SYNC_MODE_EXTERNAL          0x20

#define VIDEO_MODE_PAL              0x40
#define VIDEO_MODE_NTSC             0x00
#define VIDEO_MODE_MASK             0x40
#define VIDEO_MODE_IS_PAL(val)      (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_PAL)
#define VIDEO_MODE_IS_NTSC(val)     (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_NTSC)

#define VIDEO_SIGNAL_DEBOUNCE_MS    100 // Time to wait for input to stabilize

// VM1 bits

// duty cycle is on_off
#define BLINK_DUTY_CYCLE_50_50 0x00
#define BLINK_DUTY_CYCLE_33_66 0x01
#define BLINK_DUTY_CYCLE_25_75 0x02
#define BLINK_DUTY_CYCLE_75_25 0x03

// blinking time
#define BLINK_TIME_0 0x00
#define BLINK_TIME_1 0x04
#define BLINK_TIME_2 0x08
#define BLINK_TIME_3 0x0C

// background mode brightness (percent)
#define BACKGROUND_BRIGHTNESS_0 0x00
#define BACKGROUND_BRIGHTNESS_7 0x01
#define BACKGROUND_BRIGHTNESS_14 0x02
#define BACKGROUND_BRIGHTNESS_21 0x03
#define BACKGROUND_BRIGHTNESS_28 0x04
#define BACKGROUND_BRIGHTNESS_35 0x05
#define BACKGROUND_BRIGHTNESS_42 0x06
#define BACKGROUND_BRIGHTNESS_49 0x07

#define BACKGROUND_MODE_GRAY 0x80

// STAT register bits

#define STAT_PAL      0x01
#define STAT_NTSC     0x02
#define STAT_LOS      0x04
#define STAT_NVR_BUSY 0x20

#define STAT_IS_PAL(val)  ((val) & STAT_PAL)
#define STAT_IS_NTSC(val) ((val) & STAT_NTSC)
#define STAT_IS_LOS(val)  ((val) & STAT_LOS)

#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val))

// DMM register bits
#define DMM_AUTO_INC 0x01

// Kluege warning!
// There are occasions that NTSC is not detected even with !LOS (AB7456 specific?)
// When this happens, lower 3 bits of STAT register is read as zero.
// To cope with this case, this macro defines !LOS && !PAL as NTSC.
// Should be compatible with MAX7456 and non-problematic case.

#define VIN_IS_NTSC_alt(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))

#define MAX7456_SIGNAL_CHECK_INTERVAL_MS 1000 // msec
#define MAX7456_STALL_CHECK_INTERVAL_MS  1000 // msec

// DMM special bits
#define CLEAR_DISPLAY 0x04
#define CLEAR_DISPLAY_VERT 0x06
#define INVERT_PIXEL_COLOR 0x08

// Special address for terminating incremental write
#define END_STRING 0xff

#define MAX7456ADD_READ         0x80
#define MAX7456ADD_VM0          0x00  //0b0011100// 00 // 00             ,0011100
#define MAX7456ADD_VM1          0x01
#define MAX7456ADD_HOS          0x02
#define MAX7456ADD_VOS          0x03
#define MAX7456ADD_DMM          0x04
#define MAX7456ADD_DMAH         0x05
#define MAX7456ADD_DMAL         0x06
#define MAX7456ADD_DMDI         0x07
#define MAX7456ADD_CMM          0x08
#define MAX7456ADD_CMAH         0x09
#define MAX7456ADD_CMAL         0x0a
#define MAX7456ADD_CMDI         0x0b
#define MAX7456ADD_OSDM         0x0c
#define MAX7456ADD_RB0          0x10
#define MAX7456ADD_RB1          0x11
#define MAX7456ADD_RB2          0x12
#define MAX7456ADD_RB3          0x13
#define MAX7456ADD_RB4          0x14
#define MAX7456ADD_RB5          0x15
#define MAX7456ADD_RB6          0x16
#define MAX7456ADD_RB7          0x17
#define MAX7456ADD_RB8          0x18
#define MAX7456ADD_RB9          0x19
#define MAX7456ADD_RB10         0x1a
#define MAX7456ADD_RB11         0x1b
#define MAX7456ADD_RB12         0x1c
#define MAX7456ADD_RB13         0x1d
#define MAX7456ADD_RB14         0x1e
#define MAX7456ADD_RB15         0x1f
#define MAX7456ADD_OSDBL        0x6c
#define MAX7456ADD_STAT         0xA0


#define NVM_RAM_SIZE            54
#define WRITE_NVR               0xA0

// Device type
#define MAX7456_DEVICE_TYPE_MAX 0
#define MAX7456_DEVICE_TYPE_AT  1

#define CHARS_PER_LINE      30 // XXX Should be related to VIDEO_BUFFER_CHARS_*?

typedef enum {
    DISPLAYPORT_LAYER_FOREGROUND,
    DISPLAYPORT_LAYER_BACKGROUND,
    DISPLAYPORT_LAYER_COUNT,
} displayPortLayer_e;

#define MAX7456_SUPPORTED_LAYER_COUNT (DISPLAYPORT_LAYER_BACKGROUND + 1)

typedef struct max7456Layer_s {
    uint8_t buffer[VIDEO_BUFFER_CHARS_PAL];
} max7456Layer_t;

uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

//Max bytes to update in one call to max7456DrawScreen()
#define MAX_BYTES2SEND          250
#define MAX_BYTES2SEND_POLLED   12
#define MAX_ENCODE_US           20
#define MAX_ENCODE_US_POLLED    10

max7456Register_t max7456Reg;

static bool fontIsLoading       = false;

// previous states initialized outside the valid range to force update on first call
#define INVALID_PREVIOUS_REGISTER_STATE 255


//-----------------------------------------------------------------------------
// Implements Max7456::printMax7456Chars
//-----------------------------------------------------------------------------
static void printMax7456Chars(uint8_t chars[], uint8_t size, uint8_t x, uint8_t y) {
	uint8_t         currentCharMax7456;
	uint8_t         posAddressLO;
	uint8_t         posAddressHI;
  unsigned int posAddress;

  posAddress = 30 * y + x;

  posAddressHI = posAddress >> 8;
  posAddressLO = posAddress;

  max7456Reg._regDmm.whole = 0x01;
  spiWriteReg(MAX7456, MAX7456ADD_DMM, max7456Reg._regDmm.whole);

  spiWriteReg(MAX7456, MAX7456ADD_DMAH, posAddressHI);

  spiWriteReg(MAX7456, MAX7456ADD_DMAL, posAddressLO);

  for (int i = 0; i < size; i++) {
    currentCharMax7456 = chars[i];
    spiWriteReg(MAX7456, MAX7456ADD_DMDI, currentCharMax7456);
  }

  //end character (we're done).
  spiWriteReg(MAX7456, MAX7456ADD_DMDI, 0xFF);
}
uint8_t buffer[VIDEO_BUFFER_CHARS_PAL];
static void printMax7456Charss(const char *chars, uint8_t size, uint8_t x, uint8_t y) {
	uint8_t         currentCharMax7456;
	uint8_t         posAddressLO;
	uint8_t         posAddressHI;
  unsigned int posAddress;

  posAddress = 30 * y + x;

  posAddressHI = posAddress >> 8;
  posAddressLO = posAddress;

  max7456Reg._regDmm.whole = 0x01;
  spiWriteReg(MAX7456, MAX7456ADD_DMM, max7456Reg._regDmm.whole);

  spiWriteReg(MAX7456, MAX7456ADD_DMAH, 0);

  spiWriteReg(MAX7456, MAX7456ADD_DMAL, 0);

//  for (int i = 0; i < size; i++) {
//    currentCharMax7456 = chars[i];
//    spiWriteReg(MAX7456, MAX7456ADD_DMDI, currentCharMax7456);
//  }
  if (y < VIDEO_LINES_PAL) {
		for (int i = 0; chars[i] && x + i < CHARS_PER_LINE; i++) {
				buffer[y * CHARS_PER_LINE + x + i] = chars[i];
		}
  }
  spiWrite(MAX7456, MAX7456ADD_DMDI);
  SPI_ByteWrite_DMA(MAX7456, buffer, (uint8_t)sizeof(buffer));

  //end character (we're done).
  spiWriteReg(MAX7456, MAX7456ADD_DMDI, 0xFF);
}

void printMax7456Char(const uint8_t address, uint8_t x, uint8_t y) {
	uint8_t ad = address;
  printMax7456Chars(&ad, 1, x, y);
}
#define MAX7456_TABLE_ASCII
//-----------------------------------------------------------------------------
// Implements Max7456::giveMax7456CharFromAsciiChar
//-----------------------------------------------------------------------------
uint8_t giveMax7456CharFromAsciiChar(char ascii) {
#ifdef MAX7456_TABLE_ASCII
  if (ascii >= ' ' && ascii <= 'z')
    return ascii - ' ';
  else
    return ascii;
#else
  return ascii;
#endif
}

//-----------------------------------------------------------------------------
// Implements Max7456::print
//-----------------------------------------------------------------------------
static void print(const char string[], uint8_t x, uint8_t y) {
  char  currentChar;
  uint8_t  size;
  uint8_t* chars = NULL;

  if (!string)
    return;

  size = 0;
  currentChar = string[0];

  while (currentChar != '\0') {
    currentChar = string[++size];
  }

  chars = (uint8_t*)malloc(size * sizeof(uint8_t));

  for (uint8_t i = 0; i < size; i++) {
    chars[i] = giveMax7456CharFromAsciiChar(string[i]);
  }

  printMax7456Chars(chars, size, x, y);
  free(chars);
}


void setDisplayOffsets(uint8_t horizontal, uint8_t vertical)
{
	//    // Setup values to write to registers
	//    hosRegValue = 32;
	//    vosRegValue = 16;

	max7456Reg._regHos.whole = 0;
	max7456Reg._regVos.whole = 0;

	max7456Reg._regHos.bits.horizontalPositionOffset = horizontal;
	max7456Reg._regVos.bits.verticalPositionOffset = vertical;

	spiWriteReg(MAX7456, MAX7456ADD_HOS, max7456Reg._regHos.whole);
	spiWriteReg(MAX7456, MAX7456ADD_VOS, max7456Reg._regVos.whole);
}

void setBlinkParams(uint8_t blinkBase, uint8_t blinkDC)
{
  max7456Reg._regVm1.bits.blinkingTime = blinkBase;
  max7456Reg._regVm1.bits.blinkingDutyCycle = blinkDC;
  spiWriteReg(MAX7456, MAX7456ADD_VM1, max7456Reg._regVm1.whole);
}

void activateOSD(bool act)
{
	if(max7456Reg._isActivatedOsd != act)
	{

		max7456Reg._regVm0.bits.videoSelect = 1;
		if(act){
			max7456Reg._regVm0.bits.enableOSD = 1;
		}
		else{
			max7456Reg._regVm0.bits.enableOSD = 0;
		}

    spiWriteReg(MAX7456, MAX7456ADD_VM0, max7456Reg._regVm0.whole);
    max7456Reg._isActivatedOsd = act;
	}
}

max7456InitStatus_e max7456Init(void)
{
    gpioPinWrite(4, _DEF_HIGH);

    delay(100);

    SPI_Set_Speed_hz(MAX7456, MAX7456_INIT_MAX_SPI_CLK_HZ);

    // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
    spiWrite(MAX7456, END_STRING);

    uint8_t osdm = spiReadRegMsk(MAX7456, MAX7456ADD_OSDM);

    if (osdm != 0x1B) {
        //IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_IPU);
        return MAX7456_INIT_NOT_FOUND;
    }

    max7456Reg._isActivatedOsd = false;

    max7456Reg._regVm1.whole = 0b01000111;

    max7456Reg._regVm0.whole = 0x00;
    max7456Reg._regVm0.bits.videoSelect=1; //PAL
    max7456Reg._regVm0.bits.softwareResetBit = 1;
    //uint8_t _regVm0 = 0b01000010;
    spiWriteReg(MAX7456, MAX7456ADD_VM0, max7456Reg._regVm0.whole);
    delay(500);


    for (int x = 0; x < 16; x++) {
    	max7456Reg._regRb[x].whole = 0x00;
    	max7456Reg._regRb[x].bits.characterWhiteLevel = 2;
      spiWriteReg(MAX7456, x + MAX7456ADD_RB0, max7456Reg._regRb[x].whole);
    }

    max7456Reg._regVm0.whole = 0x00;
    max7456Reg._regVm0.bits.verticalSynch = 1;
    //_regVm0 = 0b00000100;
    spiWriteReg(MAX7456, MAX7456ADD_VM0, max7456Reg._regVm0.whole);


    SPI_Set_Speed_hz(MAX7456, MAX7456_MAX_SPI_CLK_HZ);


    setDisplayOffsets(60,18);

    setBlinkParams(_8fields, _BT_BT);

    activateOSD(true);

//    printMax7456Char(SYM_BATT_FULL, 0, 1);
//    print("Hello world :)", 1, 3);
//    print("Current Arduino time :",1,4);

    char string_buffer[30];
    tfp_sprintf(string_buffer, "V%s", FC_VERSION_STRING);
    printMax7456Charss(string_buffer, strlen(string_buffer), 0, 5);
    // Real init will be made later when driver detect idle.
    return MAX7456_INIT_OK;
}

bool max7456DmaInProgress(void)
{
	return spiIsBusy(MAX7456);
}

bool max7456WriteNvm(uint8_t char_address, const uint8_t *font_data)
{
//    if (!max7456DeviceDetected) {
//        return false;
//    }

    // Block pending completion of any prior SPI access
    spiWait(MAX7456);

    // disable display
    fontIsLoading = true;
    spiWriteReg(MAX7456, MAX7456ADD_VM0, 0);

    spiWriteReg(MAX7456, MAX7456ADD_CMAH, char_address); // set start address high

    for (int x = 0; x < 54; x++) {
        spiWriteReg(MAX7456, MAX7456ADD_CMAL, x); //set start address low
        spiWriteReg(MAX7456, MAX7456ADD_CMDI, font_data[x]);
#ifdef LED0_TOGGLE
        LED0_TOGGLE;
#else
        //LED1_TOGGLE;
#endif
    }

    // Transfer 54 bytes from shadow ram to NVM
    spiWriteReg(MAX7456, MAX7456ADD_CMM, WRITE_NVR);

    // Wait until bit 5 in the status register returns to 0 (12ms)
    while ((spiReadRegMsk(MAX7456, MAX7456ADD_STAT) & STAT_NVR_BUSY) != 0x00);

    return true;
}

#ifdef MAX7456_NRST_PIN
static IO_t max7456ResetPin        = IO_NONE;
#endif

bool max7456_display_string(const char *str, uint8_t x, uint8_t y) {
	if (spiIsBusy(MAX7456)) {
		// Not finished yet
		return false;
	}
    // Send command to display string starting at specified position
    uint8_t str_data[32]; // Assuming maximum string length of 32 characters
    size_t len = strlen(str);
    if (len > 31) len = 31; // Ensure string length does not exceed buffer size

    str_data[0] = 0x07; // Select display memory
    str_data[1] = x + (y * 30); // Calculate starting address

    // Copy string characters into buffer
    strncpy((char *)&str_data[2], str, len);
    SPI_ByteWrite_DMA(MAX7456, str_data, len + 2);
    return true;
}

#endif // USE_MAX7456
