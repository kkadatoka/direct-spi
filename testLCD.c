/**
 * @example gpio_test.c
 *
 * Basic GPIO test. Blinks LED D5 (SD-Card activity LED).

 * The blocks surrounded by #`ifdef __XENO__` are used when compiling this
 * example using [Xenomai](http://www.xenomai.org). Use these snippets in your
 * program to get a single source file that can be compiled for both, regular
 * Linux or Xenomai.
 *
 * License
 * -------
 *
 * Copyright (c) 2013 OFFIS e.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "hw.h"
#include "hw.c"
#include "spi.h"

#ifdef __XENO__
#include <sys/mman.h>
#include <rtdk.h>
#define printf rt_printf
#endif

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
#define ILI9341_PWCTR6  0xFC

*/

// GPIO 10 (MOSI), GPIO 11 (SCLK), GPIO 9(MISO), GPIO 8(CE0), GPIO 7(CE1)
#define PIN_DC		25
#define PIN_CS		24
#define PIN_MOSI	10
#define PIN_SCLK	11
#define	PIN_CE0		8


// Color definitions
#define	ILI9341_BLACK   0x0000
#define	ILI9341_BLUE    0x001F
#define	ILI9341_RED     0xF800
#define	ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_YELLOW  0xFFE0  
#define ILI9341_WHITE   0xFFFF

int	g_HWSPI = 0;

int16_t _width = ILI9341_TFTWIDTH ;
int16_t _height = ILI9341_TFTHEIGHT ;

struct packed_struct		// used for issuing commands in LoSSI mode
{
	unsigned int command:9;
} pack;

void bbCmd(uint8_t c)
{
	memory_barrier();
	gpio_clear(PIN_DC);
	gpio_clear(PIN_CE0);
	uint8_t bit = 0;
	// fast bit-bang data write on MOSI
	for(bit = 0x80; bit; bit >>= 1) 
	{
		if(c & bit) 
		{
			//digitalWrite(_mosi, HIGH); 
			// *mosiport |=  mosipinmask;
			gpio_set(PIN_MOSI) ;
		} 
		else 
		{
			//digitalWrite(_mosi, LOW); 
			// *mosiport &= ~mosipinmask;
			gpio_clear(PIN_MOSI) ;
		}
		//digitalWrite(_sclk, HIGH);
		// *clkport |=  clkpinmask;
		gpio_set(PIN_SCLK);
		//digitalWrite(_sclk, LOW);
		//*clkport &= ~clkpinmask;
		gpio_clear(PIN_SCLK);
	}
	gpio_set(PIN_CE0);
	memory_barrier();

}

void writeCommand(uint8_t c)
{
	if (!g_HWSPI)
	{
		bbCmd(c);
		return;
	}
	memory_barrier();
	gpio_clear(PIN_DC);
	gpio_clear(24);
	memory_barrier();

	pack.command = 0x000 | c;	// LoSSI 9-bit Command mode
	spi_start(0);		// Start SPI transfer to CS0 destination
	// Bypass spi_write function here
	//while (!HW.SPI0.CS.B.TXD); // ensure no reads
	//HW.SPI0.FIFO = pack.command;
	spi_write(c) ;
	spi_flush();
	memory_barrier();
	gpio_set(24);
	memory_barrier();
	printf("Command:%.2X ",pack.command);
}

void bbData(uint8_t d)
{
	memory_barrier();
	gpio_set(PIN_DC);
	gpio_clear(PIN_CE0);
	uint8_t bit = 0;
	// fast bit-bang data write on MOSI
	for(bit = 0x80; bit; bit >>= 1) 
	{
		if(d & bit) 
		{
			//digitalWrite(_mosi, HIGH); 
			// *mosiport |=  mosipinmask;
			gpio_set(PIN_MOSI) ;
		} 
		else 
		{
			//digitalWrite(_mosi, LOW); 
			// *mosiport &= ~mosipinmask;
			gpio_clear(PIN_MOSI) ;
		}
		//digitalWrite(_sclk, HIGH);
		// *clkport |=  clkpinmask;
		gpio_set(PIN_SCLK);
		//digitalWrite(_sclk, LOW);
		//*clkport &= ~clkpinmask;
		gpio_clear(PIN_SCLK);
	}
	gpio_set(PIN_CE0);
	memory_barrier();

}

void writeData(uint8_t d)
{
	if (!g_HWSPI)
	{
		bbData(d);
		return;
	}
	memory_barrier();
	gpio_set(PIN_DC);
	gpio_clear(24);
	memory_barrier();
	pack.command = 0x100 | d;	// LoSSI 9-bit Parameter mode
	spi_start(0);				// Start SPI transfer to CS0 destination
	// Bypass spi_write function here
	//while (!HW.SPI0.CS.B.TXD); // ensure no reads
	//HW.SPI0.FIFO = pack.command;
	spi_write(d);
	spi_flush();
	memory_barrier();
	gpio_set(24);
	memory_barrier();
	//printf("Data:%.2X \n",pack.command);
}

uint8_t readCommand8(uint8_t c)
{
	if (!g_HWSPI)
	{
		return 0;
	}
	writeCommand(0xd9);
	writeData(0x10);
	writeCommand(c);
	memory_barrier();
	HW.SPI0.CS.B.REN=1;		// BCM2835 ref: Set REN to read from tristate
	uint8_t r = spi_read();
	HW.SPI0.CS.B.REN=0;		// Set 0 to be safe
	memory_barrier();
	return r;
}

void setupBitBanger()
{
	if (g_HWSPI)
	{
		return;
	}
	memory_barrier();
	int i;
	for (i = 7; i <= 11; i++) gpio_configure(i, Output);
	// GPIO 10 (MOSI), GPIO 11 (SCLK), GPIO 9(MISO), GPIO 8(CE0), GPIO 7(CE1)
	memory_barrier();
	g_HWSPI=0;
}

void setupSPI()
{
	memory_barrier(); // Dont know if other peripherals have been accessed
	spi_init(8000000); // init SPI at 32MHz div8
	HW.SPI0.CS.B.LEN=1 ; // Setup LoSSI mode
	//HW.SPI0.CS.B.LEN_LONG=1;
	//HW.SPI0.CS.B.CSPOL0=0; // CS is active-low
	//HW.SPI0.CS.B.CSPOL=0; // CS is active-low
	// ILI9341 likes CPHA=1, CPOL=0
	//HW.SPI0.CS.B.CPHA=0;	
	//HW.SPI0.CS.B.CPOL=0;
	//HW.SPI0.CS.B.ADCS=0;
	HW.SPI0.CS.B.CLEAR=0b11;
	memory_barrier();
	g_HWSPI = 1;
}

void begin()
{
	// SW Reset
	writeCommand(0x01);
	st_delay(120000);

	// Display off
	writeCommand(0x28);

	// Unknown junkie
	//writeCommand(0xEF);
	//writeData(0x03);
	//writeData(0x80);
	//writeData(0x02);

	writeCommand(0xCF);  
	writeData(0x00); 
	writeData(0X83); 
	writeData(0X30); 

	writeCommand(0xED);  
	writeData(0x64); 
	writeData(0x03); 
	writeData(0X12); 
	writeData(0X81); 

	writeCommand(0xE8);  
	writeData(0x85); 
	writeData(0x01); 
	writeData(0x79); 

	writeCommand(0xCB);  
	writeData(0x39); 
	writeData(0x2C); 
	writeData(0x00); 
	writeData(0x34); 
	writeData(0x02); 

	writeCommand(0xF7);  
	writeData(0x20); 

	writeCommand(0xEA);  
	writeData(0x00); 
	writeData(0x00); 
	
	// Power control
	writeCommand(ILI9341_PWCTR1);		//Power control 
	writeData(0x26);					//VRH[5:0] 

	writeCommand(ILI9341_PWCTR2);		//Power control 
	writeData(0x11);					//SAP[2:0];BT[3:0] 
	
	// VCOM
	writeCommand(ILI9341_VMCTR1);		//VCM control 
	writeData(0x35);					//对比度调节
	writeData(0x3e); 

	writeCommand(ILI9341_VMCTR2);		//VCM control2 
	writeData(0xbe);					//--

	
//	writeCommand(ILI9341_MADCTL);		// Memory Access Control 
//	writeData(0x55);

	// Memory Access control
	writeCommand(0x3A);//ILI9341_PIXFMT);    
	writeData(0x55); 

	// Framerate
	writeCommand(ILI9341_FRMCTR1);    
	writeData(0x00);  
	writeData(0x1B); 

//	writeCommand(0xF2);				// 3Gamma Function Disable 
//	writeData(0x00); 

	writeCommand(ILI9341_GAMMASET);   //Gamma curve selected 
	writeData(0x01); 

	writeCommand(0xB7);
	writeData(0x07);
	writeCommand(ILI9341_DFUNCTR);    // Display Function Control 
	writeData(0x0A); 
	writeData(0x82);
	writeData(0x27);
	writeData(0x00);

/*
	writeCommand(ILI9341_GMCTRP1);    //Set Gamma 
	writeData(0x0F); 
	writeData(0x31); 
	writeData(0x2B); 
	writeData(0x0C); 
	writeData(0x0E); 
	writeData(0x08); 
	writeData(0x4E); 
	writeData(0xF1); 
	writeData(0x37); 
	writeData(0x07); 
	writeData(0x10); 
	writeData(0x03); 
	writeData(0x0E); 
	writeData(0x09); 
	writeData(0x00); 

	writeCommand(ILI9341_GMCTRN1);    //Set Gamma 
	writeData(0x00); 
	writeData(0x0E); 
	writeData(0x14); 
	writeData(0x03); 
	writeData(0x11); 
	writeData(0x07); 
	writeData(0x31); 
	writeData(0xC1); 
	writeData(0x48); 
	writeData(0x08); 
	writeData(0x0F); 
	writeData(0x0C); 
	writeData(0x31); 
	writeData(0x36); 
	writeData(0x0F); 
*/
	writeCommand(ILI9341_SLPOUT);    //Exit Sleep 
	st_delay(120000); 		
	writeCommand(ILI9341_DISPON);    //Display on 
}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) 
{
	writeCommand(ILI9341_CASET); // Column addr set
	writeData(x0 >> 8);
	writeData(x0 & 0xFF);     // XSTART 
	writeData(x1 >> 8);
	writeData(x1 & 0xFF);     // XEND

	writeCommand(ILI9341_PASET); // Row addr set
	writeData(y0>>8);
	writeData(y0);     // YSTART
	writeData(y1>>8);
	writeData(y1);     // YEND
	writeCommand(ILI9341_RAMWR); // write to RAM
}

void pushColor(uint16_t color) 
{
	writeData(color >> 8);
	writeData(color);
}

void drawPixel(int16_t x, int16_t y, uint16_t color) 
{
	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

	setAddrWindow(x,y,x+1,y+1);

	writeData(color >> 8);
	writeData(color);
}

void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) 
{
	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) return;

	if((y+h-1) >= _height) 
	h = _height-y;

	setAddrWindow(x, y, x, y+h-1);

	uint8_t hi = color >> 8, lo = color;

	while (h--) 
	{
		writeData(hi);
		writeData(lo);
	}
}

void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) 
{
	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) return;
	if((x+w-1) >= _width)  w = _width-x;
	setAddrWindow(x, y, x+w-1, y);

	uint8_t hi = color >> 8, lo = color;
	while (w--) 
	{
		writeData(hi);
		writeData(lo);
	}
}

// fill a rectangle
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
{
	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) return;
	if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;

	setAddrWindow(x, y, x+w-1, y+h-1);

	uint8_t hi = color >> 8, lo = color;

	for(y=h; y>0; y--) 
	{
		for(x=w; x>0; x--) 
		{
			writeData(hi);
			writeData(lo);
		}
	}
}

void fillScreen(uint16_t color) 
{
	fillRect(0, 0,  _width, _height, color);
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) 
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void setRotation(uint8_t m) 
{
	writeCommand(ILI9341_MADCTL);
	uint8_t rotation = m % 4; // can't be higher than 3
	switch (rotation) 
	{
		case 0:
			writeData(MADCTL_MX | MADCTL_BGR);
			_width  = ILI9341_TFTWIDTH;
			_height = ILI9341_TFTHEIGHT;
			break;
		case 1:
			writeData(MADCTL_MV | MADCTL_BGR);
			_width  = ILI9341_TFTHEIGHT;
			_height = ILI9341_TFTWIDTH;
			break;
		case 2:
			writeData(MADCTL_MY | MADCTL_BGR);
			_width  = ILI9341_TFTWIDTH;
			_height = ILI9341_TFTHEIGHT;
			break;
		case 3:
			writeData(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
			_width  = ILI9341_TFTHEIGHT;
			_height = ILI9341_TFTWIDTH;
			break;
	}
}

void invertDisplay(int i) 
{
	writeCommand(i ? ILI9341_INVON : ILI9341_INVOFF);
}


int main(int argc, char *argv[])
{
#ifdef __XENO__
	struct sched_param param = { 99 };
	mlockall(MCL_CURRENT | MCL_FUTURE);
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
#endif

	if (!raspi_map_hw()) {
		perror("Could not map hardware registers");
		exit(1);
	}

#ifdef __XENO__
	pthread_set_mode_np(0, PTHREAD_WARNSW|PTHREAD_PRIMARY);
#endif


	// Configure GPIO#22 for reset and do a HW reset
	printf("Setting up for HWreset\n");
	memory_barrier();
	gpio_configure(25, Output);
	gpio_configure(24, Output);
	gpio_configure(22, Output);
	gpio_set(22);
	sleep(1);
	gpio_clear(22);
	sleep(1);
	gpio_set(22);
	memory_barrier();
	
	g_HWSPI=0;
	if (argc > 1)
	{
		g_HWSPI=1;
	}

	if (g_HWSPI)
	{
		printf("Setup HW SPI parameters...\n");
		setupSPI();
	}
	else
	{
		printf("Setup SW SPI (Bit-Banger) parameters...\n");
		setupBitBanger();
	}

	printf("HWReset done, initializing display\n") ;
	int i, colors[8] = { ILI9341_BLACK, ILI9341_BLUE, ILI9341_RED, ILI9341_GREEN, ILI9341_CYAN, ILI9341_MAGENTA, ILI9341_WHITE, ILI9341_YELLOW }; 
	begin();
	printf("Done intializing, now fill screens...\n");
	for (i=0;i<8 ;i++ )
	{
		fillScreen(ILI9341_BLACK);
		//sleep(1);
		fillScreen(colors[i]);
		//sleep (2);
	}
	printf("Setting up for HWreset\n");
	memory_barrier();
	gpio_configure(25, Output);
	gpio_configure(24, Output);
	gpio_configure(22, Output);
	gpio_set(22);
	sleep(1);
	gpio_clear(22);
	sleep(1);
	gpio_set(22);
	memory_barrier();
	printf("Bye now\n");
	sleep (5);

#ifdef __XENO__
	pthread_set_mode_np(PTHREAD_WARNSW, 0);
#endif

	return 1;
}
