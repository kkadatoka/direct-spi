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
#include "GFX.h"
#include "ILI9341.h"
#include "hw.h"
#include "hw.c"
#include "spi.h"

#ifdef __XENO__
#include <sys/mman.h>
#include <rtdk.h>
#define printf rt_printf
#endif





void sl_delay(uint32t msec)
{
	st_delay(msec*1000) ;
}

uint32_t micros()
{
	return ST_NOW;
}

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9341_BLACK);
  tft.fillScreen(ILI9341_RED);
  tft.fillScreen(ILI9341_GREEN);
  tft.fillScreen(ILI9341_BLUE);
  tft.fillScreen(ILI9341_BLACK);
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!\n");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println("1234.56\n");
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println("0xDEADBEEF\n");
  tft.println("\n");
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop\n");
  tft.setTextSize(2);
  tft.println("I implore thee,\n");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.\n");
  tft.println("And hooptiously drangle me\n");
  tft.println("with crinkly bindlewurdles,\n");
  tft.println("Or I will rend thee\n");
  tft.println("in the gobberwarts\n");
  tft.println("with my blurglecruncheon,\n");
  tft.println("see if I don't!\n");
  return micros() - start;
}


unsigned long testLines(uint16_t color) 
{
	unsigned long start, t;
	int           x1, y1, x2, y2,
			w = tft.width(),
			h = tft.height();

	tft.fillScreen(ILI9341_BLACK);

	x1 = y1 = 0;
	y2    = h - 1;
	start = micros();
	for(x2=0; x2<w; x2+=6) 
		tft.drawLine(x1, y1, x2, y2, color);
	x2    = w - 1;
	for(y2=0; y2<h; y2+=6) 
		tft.drawLine(x1, y1, x2, y2, color);
	t     = micros() - start; // fillScreen doesn't count against timing

	tft.fillScreen(ILI9341_BLACK);

	x1    = w - 1;
	y1    = 0;
	y2    = h - 1;
	start = micros();
	for(x2=0; x2<w; x2+=6) 
		tft.drawLine(x1, y1, x2, y2, color);
	x2    = 0;
	for(y2=0; y2<h; y2+=6)
		tft.drawLine(x1, y1, x2, y2, color);
	t    += micros() - start;

	tft.fillScreen(ILI9341_BLACK);

	x1    = 0;
	y1    = h - 1;
	y2    = 0;
	start = micros();
	for(x2=0; x2<w; x2+=6) 
		tft.drawLine(x1, y1, x2, y2, color);
	x2    = w - 1;
	for(y2=0; y2<h; y2+=6)
		tft.drawLine(x1, y1, x2, y2, color);
	t    += micros() - start;

	tft.fillScreen(ILI9341_BLACK);

	x1    = w - 1;
	y1    = h - 1;
	y2    = 0;
	start = micros();
	for(x2=0; x2<w; x2+=6) 
		tft.drawLine(x1, y1, x2, y2, color);
	x2    = 0;
	for(y2=0; y2<h; y2+=6) 
		tft.drawLine(x1, y1, x2, y2, color);

	return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) 
{
	unsigned long start;
	int           x, y, w = tft.width(), h = tft.height();

	tft.fillScreen(ILI9341_BLACK);
	start = micros();
	for(y=0; y<h; y+=5) 
		tft.drawFastHLine(0, y, w, color1);
	for(x=0; x<w; x+=5) 
		tft.drawFastVLine(x, 0, h, color2);

	return micros() - start;
}

unsigned long testRects(uint16_t color) 
{
	unsigned long start;
	int           n, i, i2,
				cx = tft.width()  / 2,
				cy = tft.height() / 2;

	tft.fillScreen(ILI9341_BLACK);
	n     = min(tft.width(), tft.height());
	start = micros();
	for(i=2; i<n; i+=6) 
	{
		i2 = i / 2;
		tft.drawRect(cx-i2, cy-i2, i, i, color);
	}

	return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) 
{
	unsigned long start, t = 0;
	int n, i, i2,
			cx = tft.width()  / 2 - 1,
			cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	n = min(tft.width(), tft.height());
	for(i=n; i>0; i-=6) 
	{
		i2    = i / 2;
		start = micros();
		tft.fillRect(cx-i2, cy-i2, i, i, color1);
		t    += micros() - start;
		// Outlines are not included in timing results
		tft.drawRect(cx-i2, cy-i2, i, i, color2);
	}

	return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) 
{
	unsigned long start;
	int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

	tft.fillScreen(ILI9341_BLACK);
	start = micros();
	for(x=radius; x<w; x+=r2) 
	{
		for(y=radius; y<h; y+=r2) 
		{
			tft.fillCircle(x, y, radius, color);
		}
	}

	return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) 
{
	unsigned long start;
	int x, y, r2 = radius * 2,
				w = tft.width()  + radius,
				h = tft.height() + radius;

	// Screen is not cleared for this one -- this is
	// intentional and does not affect the reported time.
	start = micros();
	for(x=0; x<w; x+=r2) 
	{
		for(y=0; y<h; y+=r2) 
		{
			tft.drawCircle(x, y, radius, color);
		}
	}

	return micros() - start;
}

unsigned long testTriangles() 
{
	unsigned long start;
	int	n, i, cx = tft.width()  / 2 - 1,
				  cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	n     = min(cx, cy);
	start = micros();
	for(i=0; i<n; i+=5) 
	{
		tft.drawTriangle(
			cx    , cy - i, // peak
			cx - i, cy + i, // bottom left
			cx + i, cy + i, // bottom right
			tft.color565(0, 0, i));
	}

	return micros() - start;
}

unsigned long testFilledTriangles() 
{
	unsigned long start, t = 0;
	int i, cx = tft.width()  / 2 - 1,
			   cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	start = micros();
	for(i=min(cx,cy); i>10; i-=5) 
	{
		start = micros();
		tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
			tft.color565(0, i, i));
		t += micros() - start;
		tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
			tft.color565(i, i, 0));
	}

  return t;
}

unsigned long testRoundRects() 
{
	unsigned long start;
	int           w, i, i2,
				cx = tft.width()  / 2 - 1,
				cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	w     = min(tft.width(), tft.height());
	start = micros();
	for(i=0; i<w; i+=6) 
	{
		i2 = i / 2;
		tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
	}

	return st_elapsed(start, ST_NOW, ST_NOW - start);
}

unsigned long testFilledRoundRects() 
{
	unsigned long start;
	int           i, i2,
		cx = tft.width()  / 2 - 1,
		cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	start = micros();
	for(i=min(tft.width(), tft.height()); i>20; i-=6) 
	{
		i2 = i / 2;
		tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
	}
	return st_elapsed(start, ST_NOW, ST_NOW - start);
}

// GPIO 10 (MOSI), GPIO 11 (SCLK), GPIO 9(MISO), GPIO 8(CE0), GPIO 7(CE1)
#define PIN_DC		25
#define PIN_CS		24
#define PIN_MOSI	10
#define PIN_MISO	9
#define PIN_SCLK	11
#define	PIN_CE0		8
#define	PIN_RESET	22
#define	PIN_BL		18

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

	//	ILI9341(int8_t _CS, int8_t _DC, int8_t _RST, int8_t _BL, int8_t _MOSI, int8_t _SCLK, int8_t _MISO);
	ILI9341 tft = ILI9341(PIN_CS, PIN_DC, PIN_RESET, PIN_BL, PIN_MOSI, PIN_SCLK, PIN_MISO);

	printf("Setting up TFT...\n");
	tft.begin();
	
	g_HWSPI=0;
	if (argc > 1)
	{
		g_HWSPI=1;
	}

	if (g_HWSPI)
	{
		printf("Setup HW SPI parameters...\n");
	}
	else
	{
		printf("Setup SW SPI (Bit-Banger) parameters...\n");
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

	printf("Benchmark\tTime (microseconds)\n");

	printf("Screen fill\t%f\n", testFillScreen());
	sl_delay(500);

	printf("Text\t%f\n",testText());
	sl_delay(3000);

	printf("Lines\t%f\n",testLines(ILI9341_CYAN));
	sl_delay(500);

	printf("Horiz/Vert Lines\t%f\n", testFastLines(ILI9341_RED, ILI9341_BLUE));
	sl_delay(500);

	printf("Rectangles (outline)\t%f",testRects(ILI9341_GREEN));
	sl_delay(500);

	printf("Rectangles (filled)\t%f\n", testFilledRects(ILI9341_YELLOW, ILI9341_MAGENTA));
	sl_delay(500);

	printf("Circles (filled)\t%f\n", testFilledCircles(10, ILI9341_MAGENTA));

	printf("Circles (outline)\t%f\n", testCircles(10, ILI9341_WHITE));
	sl_delay(500);

	printf("Triangles (outline)\t%f\n", testTriangles());
	sl_delay(500);

	printf("Triangles (filled)\t%f\n", testFilledTriangles());
	sl_delay(500);

	printf("Rounded rects (outline)\t%f\n", testRoundRects());
	sl_delay(500);

	printf("Rounded rects (filled)\t%f\n", testFilledRoundRects());
	sl_delay(500);

	printf("Done!\n");

	for(uint8_t rotation=0; rotation<4; rotation++) 
	{
		tft.setRotation(rotation);
		testText();
		sl_delay(1000);
	}

#ifdef __XENO__
	pthread_set_mode_np(PTHREAD_WARNSW, 0);
#endif

	return 1;
}
