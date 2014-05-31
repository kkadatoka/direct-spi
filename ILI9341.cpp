/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "ILI9341.h"
#include "hw.h"
#include "spi.h"
#include "hw.c"
#include <sys/time.h>

// Constructor when using software SPI.  All output pins are configurable.
ILI9341::ILI9341(int8_t cs, int8_t dc, int8_t rst, int8_t bl, int8_t mosi, int8_t sclk, int8_t miso) : GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) 
{
	// GPIO Pins for bit-banging
	_cs   = cs;
	_dc   = dc;
	_mosi  = mosi;
	_miso = miso;
	_sclk = sclk;
	_rst  = rst;
	_bl = bl;
	hwSPI = false;
	spiSet = false;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ILI9341::ILI9341(int8_t cs, int8_t dc, int8_t bl, int8_t rst) : GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) 
{
	// GPIO pins for D/C, CS and RST and HW SPI
	_cs   = cs;
	_dc   = dc;
	_rst  = rst;
	_bl	  = bl;
	hwSPI = true;
	_mosi  = _sclk = 0;
	spiSet = false;
}

ILI9341::~ILI9341()
{
	memory_barrier();
	gpio_set(_rst);
	delay(5);			// 5ms
	gpio_clear(_rst);
	st_delay(20);		// 20ms
	gpio_set(_rst);
	st_delay(150);			// 150ms
	gpio_clear(_bl);
	memory_barrier();
	raspi_unmap_hw();
}
void ILI9341::delay(uint32_t msecs)
{
	st_delay(msecs*1000);
}

double ILI9341::micros()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((tv.tv_sec*1000000.0) + (tv.tv_usec));
}

void ILI9341::spiSetup()
{
	if (spiSet)
	{
		return ;
	}
	if (hwSPI)
	{
		memory_barrier();
		gpio_configure(_bl, Output);
		gpio_set(_bl);
		memory_barrier(); // Dont know if other peripherals have been accessed
		spi_init(31250000); // init SPI at 32MHz div8
		//HW.SPI0.CS.B.LEN=1 ; // Setup LoSSI mode
		//HW.SPI0.CS.B.LEN_LONG=1;
		//HW.SPI0.CS.B.CSPOL0=0; // CS is active-low
		//HW.SPI0.CS.B.CSPOL=0; // CS is active-low
		// ILI9341 likes CPHA=1, CPOL=0
		HW.SPI0.CS.B.CPHA=1;	
		HW.SPI0.CS.B.CPOL=1;
		//HW.SPI0.CS.B.ADCS=0;
		//HW.SPI0.CS.B.CLEAR=0b11;
		memory_barrier();
		spiSet = true;
	}
	else
	{
		// bit-banger setup
		memory_barrier();
		gpio_configure(_bl, Output);
		gpio_set(_bl);
		gpio_configure(_dc, Output);
		gpio_configure(_cs, Output);
		gpio_configure(_mosi, Output);
		gpio_configure(_sclk, Output);
		gpio_configure(_miso, Input);
		for (int i = 7; i <= 11; i++) gpio_configure(i, Output);
		if (_rst > 0) 
		{
			gpio_configure(_rst, Output);
			gpio_clear(_rst);
		}
		memory_barrier();
		spiSet = true;
	}
}

void ILI9341::spiwrite(uint8_t c) 
{
	if (hwSPI) 
	{
		memory_barrier();
		spi_start(0);				// Start SPI transfer to CS0 destination
		// Bypass spi_write function here
		//while (!HW.SPI0.CS.B.TXD); // ensure no reads
		//HW.SPI0.FIFO = pack.command;
		spi_write(c);
		spi_flush();
		memory_barrier();
	} 
	else 
	{
		memory_barrier();			
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) 
		{
			if(c & bit) 
			{
				gpio_set(_mosi) ;
			} 
			else 
			{
				gpio_clear(_mosi) ;
			}
			gpio_set(_sclk);
			gpio_clear(_sclk);
		}
		memory_barrier();
	}
}


void ILI9341::writecommand(uint8_t c) 
{
	gpio_clear(_dc);	// LOW
	//if(!hwSPI)
	{
		memory_barrier();		
		gpio_clear(_cs);	// LOW
		memory_barrier();		
	}
	spiwrite(c);
	//if(!hwSPI)
	{
		memory_barrier();		
		gpio_set(_cs);		// HIGH
		memory_barrier();		
	}
}


void ILI9341::writedata(uint8_t c)
{
	gpio_set(_dc);	// HIGH
	//if(!hwSPI)
	{
		memory_barrier();		
		gpio_clear(_cs);	// LOW
		memory_barrier();		
	}
	spiwrite(c);
	//if(!hwSPI)
	{
		memory_barrier();		
		gpio_set(_cs);		// HIGH
		memory_barrier();		
	}
} 

int8_t ILI9341::begin(void) 
{
	if (!raspi_map_hw()) 
	{
		return 1;
	}

	spiSetup() ;
	// toggle RST low to reset
	if (_rst > 0) 
	{
		memory_barrier();
		gpio_set(_rst);
		st_delay(5000);			// 5ms
		gpio_clear(_rst);
		st_delay(20000);		// 20ms
		gpio_set(_rst);
		st_delay(150000);			// 150ms
		memory_barrier();
	}

	writecommand(0xEF);
	writedata(0x03);
	writedata(0x80);
	writedata(0x02);

	writecommand(0xCF);  
	writedata(0x00); 
	writedata(0XC1); 
	writedata(0X30); 

	writecommand(0xED);  
	writedata(0x64); 
	writedata(0x03); 
	writedata(0X12); 
	writedata(0X81); 

	writecommand(0xE8);  
	writedata(0x85); 
	writedata(0x00); 
	writedata(0x78); 

	writecommand(0xCB);  
	writedata(0x39); 
	writedata(0x2C); 
	writedata(0x00); 
	writedata(0x34); 
	writedata(0x02); 

	writecommand(0xF7);  
	writedata(0x20); 

	writecommand(0xEA);  
	writedata(0x00); 
	writedata(0x00); 

	writecommand(ILI9341_PWCTR1);    //Power control 
	writedata(0x23);   //VRH[5:0] 

	writecommand(ILI9341_PWCTR2);    //Power control 
	writedata(0x10);   //SAP[2:0];BT[3:0] 

	writecommand(ILI9341_VMCTR1);    //VCM control 
	writedata(0x3e); //对比度调节
	writedata(0x28); 

	writecommand(ILI9341_VMCTR2);    //VCM control2 
	writedata(0x86);  //--

	writecommand(ILI9341_MADCTL);    // Memory Access Control 
	writedata(0x48);

	writecommand(ILI9341_PIXFMT);    
	writedata(0x55); 

	writecommand(ILI9341_FRMCTR1);    
	writedata(0x00);  
	writedata(0x18); 

	writecommand(ILI9341_DFUNCTR);    // Display Function Control 
	writedata(0x08); 
	writedata(0x82);
	writedata(0x27);  

	writecommand(0xF2);    // 3Gamma Function Disable 
	writedata(0x00); 

	writecommand(ILI9341_GAMMASET);    //Gamma curve selected 
	writedata(0x01); 

	writecommand(ILI9341_GMCTRP1);    //Set Gamma 
	writedata(0x0F); 
	writedata(0x31); 
	writedata(0x2B); 
	writedata(0x0C); 
	writedata(0x0E); 
	writedata(0x08); 
	writedata(0x4E); 
	writedata(0xF1); 
	writedata(0x37); 
	writedata(0x07); 
	writedata(0x10); 
	writedata(0x03); 
	writedata(0x0E); 
	writedata(0x09); 
	writedata(0x00); 

	writecommand(ILI9341_GMCTRN1);    //Set Gamma 
	writedata(0x00); 
	writedata(0x0E); 
	writedata(0x14); 
	writedata(0x03); 
	writedata(0x11); 
	writedata(0x07); 
	writedata(0x31); 
	writedata(0xC1); 
	writedata(0x48); 
	writedata(0x08); 
	writedata(0x0F); 
	writedata(0x0C); 
	writedata(0x31); 
	writedata(0x36); 
	writedata(0x0F); 

	writecommand(ILI9341_SLPOUT);    //Exit Sleep 
	st_delay(120000); 		
	writecommand(ILI9341_DISPON);    //Display on 
	fillScreen(ILI9341_BLACK);
	return 0;
}


void ILI9341::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) 
{
	writecommand(ILI9341_CASET); // Column addr set
	writedata(x0 >> 8);
	writedata(x0 & 0xFF);     // XSTART 
	writedata(x1 >> 8);
	writedata(x1 & 0xFF);     // XEND

	writecommand(ILI9341_PASET); // Row addr set
	writedata(y0>>8);
	writedata(y0);     // YSTART
	writedata(y1>>8);
	writedata(y1);     // YEND

	writecommand(ILI9341_RAMWR); // write to RAM
}


void ILI9341::pushColor(uint16_t color) 
{
	writedata(color >> 8) ;
	writedata(color) ;
}

void ILI9341::drawPixel(int16_t x, int16_t y, uint16_t color) 
{
	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) 
		return;

	setAddrWindow(x,y,x+1,y+1);
	pushColor(color);
}


void ILI9341::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) 
{
	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) 
		return;

	if((y+h-1) >= _height) 
		h = _height-y;

	setAddrWindow(x, y, x, y+h-1);

	while (h--) 
	{
		pushColor(color);
	}
}


void ILI9341::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) 
{
	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) 
		return;

	if((x+w-1) >= _width)  
		w = _width-x;
	
	setAddrWindow(x, y, x+w-1, y);

	while (w--) 
	{
		pushColor(color);
	}
}

void ILI9341::fillScreen(uint16_t color) 
{
	fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
{
	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) 
		return;
	if((x + w - 1) >= _width)  
		w = _width  - x;
	if((y + h - 1) >= _height) 
		h = _height - y;

	setAddrWindow(x, y, x+w-1, y+h-1);

	for(y=h; y>0; y--) 
	{
		for(x=w; x>0; x--) 
		{
			pushColor(color);
		}
	}
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t ILI9341::color565(uint8_t r, uint8_t g, uint8_t b) 
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

void ILI9341::setRotation(uint8_t m) 
{
	writecommand(ILI9341_MADCTL);
	rotation = m % 4; // can't be higher than 3
	switch (rotation) 
	{
		case 0:
			writedata(MADCTL_MX | MADCTL_BGR);
			_width  = ILI9341_TFTWIDTH;
			_height = ILI9341_TFTHEIGHT;
			break;
		case 1:
			writedata(MADCTL_MV | MADCTL_BGR);
			_width  = ILI9341_TFTHEIGHT;
			_height = ILI9341_TFTWIDTH;
			break;
		case 2:
			writedata(MADCTL_MY | MADCTL_BGR);
			_width  = ILI9341_TFTWIDTH;
			_height = ILI9341_TFTHEIGHT;
			break;
		case 3:
			writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
			_width  = ILI9341_TFTHEIGHT;
			_height = ILI9341_TFTWIDTH;
			break;
	}
}

void ILI9341::invertDisplay(boolean i) 
{
	writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
}


////////// stuff not actively being used, but kept for posterity


uint32_t ILI9341::spiread(uint8_t reg, uint8_t mode, uint8_t index) 
{
	uint8_t r, d[mode];
	uint32_t ret;

	if (hwSPI) 
	{
		writecommand(0xd9);
		writedata(0x10);
		memory_barrier();
		HW.SPI0.CS.B.REN=1;		// BCM2835 ref: Set REN bit to read from tristate
		uint8_t r = spi_read();
		HW.SPI0.CS.B.REN=0;		// Set 0 to be safe
		memory_barrier();
		return r;
	} 
	else 
	{
		writecommand(0xd9);
		writedata(0x10);
		memory_barrier();
		gpio_configure(_mosi, Input); // read from MOSI
		for (uint8_t i=0;i<mode ;i++ )
		{
			for (uint8_t i=0; i<8; i++) 
			{
				gpio_clear(_sclk);
				gpio_set(_sclk);
				r <<= 1;
				if (gpio_read(_mosi))
				  r |= 0x1;
			}
			ret |= r;
		}
		gpio_configure(_mosi,Output);
		memory_barrier();
	}  
	return ret;
}

uint8_t ILI9341::readdata(void) 
{
	uint8_t r = 0;
	/*
	digitalWrite(_dc, HIGH);
	digitalWrite(_cs, LOW);
	uint8_t r = spiread();
	digitalWrite(_cs, HIGH);
	*/
	return r;
}
 

uint8_t ILI9341::readcommand8(uint8_t c, uint8_t index) 
{
	uint8_t r = 0;
	/*
	digitalWrite(_dc, LOW); // command
	digitalWrite(_cs, LOW);
	spiwrite(0xD9);  // woo sekret command?
	digitalWrite(_dc, HIGH); // data
	spiwrite(0x10 + index);
	digitalWrite(_cs, HIGH);

	digitalWrite(_dc, LOW);
	digitalWrite(_sclk, LOW);
	digitalWrite(_cs, LOW);
	spiwrite(c);

	digitalWrite(_dc, HIGH);
	uint8_t r = spiread();
	digitalWrite(_cs, HIGH);
	*/
	return r;
}


 
/*

 uint16_t ILI9341::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t ILI9341::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
