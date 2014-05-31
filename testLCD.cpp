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
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include "GFX.h"
#include "ILI9341.h"

#ifdef __XENO__
#include <sys/mman.h>
#include <rtdk.h>
#define printf rt_printf
#endif

// GPIO 10 (MOSI), GPIO 11 (SCLK), GPIO 9(MISO), GPIO 8(CE0), GPIO 7(CE1)
#define PIN_DC		25
#define PIN_CS		24
#define PIN_MOSI	10
#define PIN_MISO	9
#define PIN_SCLK	11
#define	PIN_CE0		8
#define	PIN_RESET	23
#define	PIN_BL		24


//	ILI9341(int8_t _CS, int8_t _DC, int8_t _RST, int8_t _BL, int8_t _MOSI, int8_t _SCLK, int8_t _MISO);
ILI9341 tft = ILI9341(PIN_CE0, PIN_DC, PIN_RESET, PIN_BL, PIN_MOSI, PIN_SCLK, PIN_MISO);

uint16_t min(uint16_t x, uint16_t y)
{
	if (x<y)
	{
		return x;
	}
	return y;
}

double micros()
{
	return tft.micros();
}

unsigned long testFillScreen() {
  double start = micros();
  tft.fillScreen(ILI9341_BLACK);
  tft.fillScreen(ILI9341_RED);
  tft.fillScreen(ILI9341_GREEN);
  tft.fillScreen(ILI9341_BLUE);
  tft.fillScreen(ILI9341_BLACK);
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  double start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println("1234.56");
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println("0xDEADBEEF");
  tft.println(" ");
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}


unsigned long testLines(uint16_t color) 
{
	double start, t;
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
	double start;
	int x, y, w = tft.width(), h = tft.height();

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
	double start;
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
	double start, t = 0;
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
	double start;
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
	double start;
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
	double start;
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
	double start, t = 0;
	int i, cx = tft.width()  / 2 - 1,
			   cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	start = micros();
	for(i=min(cx,cy); i>10; i-=5) 
	{
		start = micros();
		tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
			tft.color565(0, i, i));
		t += (micros() - start);
		tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
			tft.color565(i, i, 0));
	}

  return t;
}

unsigned long testRoundRects() 
{
	double elapsed;
	timeval t1, t2;
	int           w, i, i2,
				cx = tft.width()  / 2 - 1,
				cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	gettimeofday(&t1, NULL);
	w     = min(tft.width(), tft.height());
	for(i=0; i<w; i+=6) 
	{
		i2 = i / 2;
		tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
	}
	gettimeofday(&t2,NULL);
	// compute and return the elapsed time in uS
    elapsed = (t2.tv_sec - t1.tv_sec) * 1000000.0;      // sec to us
    elapsed += (t2.tv_usec - t1.tv_usec) ;				// us elapsed = 
	return elapsed;
}

unsigned long testFilledRoundRects() 
{
	double elapsed;
	timeval t1, t2;

	int           i, i2,
		cx = tft.width()  / 2 - 1,
		cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	gettimeofday(&t1, NULL);
	for(i=min(tft.width(), tft.height()); i>20; i-=6) 
	{
		i2 = i / 2;
		tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
	}
	gettimeofday(&t2,NULL);
	// compute and return the elapsed time in uS
    elapsed = (t2.tv_sec - t1.tv_sec) * 1000000.0;      // sec to us
    elapsed += (t2.tv_usec - t1.tv_usec) ;				// us elapsed = 
	return elapsed;
}

void print_hex(const uint32_t *buf, int len)
{
   int i;

   for (i = 0; i < len; i++) {
//      if (!(i % 16))
//         puts("");
      printf("%.2X ", buf[i]);
   }
   puts("");
}

void readid()
{
	uint32_t id = 0;//tft.spiread(ILI9341_RDDID, MODE_24bit);
	//print_hex ( id, 32);
	printf("Readid: %0x\n", id);
	id = 0; //tft.spiread(ILI9341_RDDST, MODE_32bit); 
	printf("Read Disp Status: %0x\n", id);
}

void sig_handler(int signo)
{
	if (signo == SIGINT)
	{
		printf("Exiting...\n");
		exit (0);
	}
}

int main(int argc, char *argv[])
{
#ifdef __XENO__
	struct sched_param param = { 99 };
	mlockall(MCL_CURRENT | MCL_FUTURE);
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
#endif


#ifdef __XENO__
	pthread_set_mode_np(0, PTHREAD_WARNSW|PTHREAD_PRIMARY);
#endif

	signal(SIGINT, sig_handler);

	printf("Setting up TFT...\n");
	if (tft.begin())
	{
		perror("Could not map hardware registers");
		exit (1);
	}
	
	int g_HWSPI=0;
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
	printf("Read HW ID...\n");
	readid();
	
	tft.setRotation(2);
	tft.fillScreen(ILI9341_BLACK);
	tft.setCursor(70, 100);
	tft.setTextColor(ILI9341_RED);  
	tft.setTextSize(3);
	tft.println("Hello!");
	tft.setTextSize(2);
	tft.setTextColor(ILI9341_YELLOW);  
	tft.setCursor(0, 160);
	tft.println("This is a graphics\ntest.");
	printf("Displayed message\n");
	tft.delay(5000);

	double elap;
	printf("Benchmark             \tTime (microseconds)\n");

	elap = testFillScreen();
	printf("Screen fill           \t%f\n", elap);
	tft.delay(500);

	tft.setRotation(2) ;
	elap = testText();
	printf("Text                  \t%f\n", elap);
	tft.delay(10000);
	
	elap = testLines(ILI9341_CYAN);
	printf("Lines                 \t%f\n",elap);
	tft.delay(500);
	
	elap = testFastLines(ILI9341_RED, ILI9341_BLUE);
	printf("Horiz/Vert Lines      \t%f\n", elap);
	tft.delay(500);

	elap = testRects(ILI9341_GREEN);
	printf("Rectangles (outline)  \t%f\n", elap);
	tft.delay(500);

	elap = testFilledRects(ILI9341_YELLOW, ILI9341_MAGENTA);
	printf("Rectangles (filled)   \t%f\n", elap);
	tft.delay(500);

	elap = testFilledCircles(10, ILI9341_MAGENTA);
	printf("Circles (filled)      \t%f\n", elap);

	elap = testCircles(10, ILI9341_WHITE);
	printf("Circles (outline)     \t%f\n", elap);
	tft.delay(500);

	elap = testTriangles();
	printf("Triangles (outline)   \t%f\n", elap);
	tft.delay(500);

	elap = testFilledTriangles();
	printf("Triangles (filled)    \t%f\n", elap);
	tft.delay(500);

	elap = testRoundRects();
	printf("Rounded rects(outline)\t%f\n", elap);
	tft.delay(500);

	elap = testFilledRoundRects();
	printf("Rounded rects (filled)\t%f\n", elap);
	tft.delay(500);

	for(uint8_t rotation=0; rotation<4; rotation++) 
	{
		tft.setRotation(rotation);
		testText();
		tft.delay(5000);
	}

	printf("Done!Ctrl+C to exit...");

	time_t t; 
	struct tm * tm;
	char timedisp[255];
	tft.setRotation(2);
	tft.fillScreen(ILI9341_BLACK);
	tft.fillRoundRect(35, 180, 170, 115, 180/8, ILI9341_CYAN);
	tft.drawRoundRect(35, 180, 170, 115, 180/8, ILI9341_RED);
	tft.setTextSize(4);
	tft.setTextColor(ILI9341_RED);
	tft.setCursor(50, 200);
	tft.println("Playya");
	tft.setTextColor(ILI9341_BLUE);
	tft.setTextSize(3);
	tft.setCursor(55, 250);
	tft.println("Kumar K");
	while(true)
	{
		int day,mon,year, hour, min, sec;
		tft.setRotation(2);
		t = time(NULL);
		tm = localtime(&t);
		if ((day != tm->tm_mday) || (mon != tm->tm_mon+1) || (year != tm->tm_year+1900))
		{
			day = tm->tm_mday;
			mon = tm->tm_mon+1;
			year = tm->tm_year+1900;
			tft.fillRect(30, 30, 100, 40, ILI9341_BLACK);
			sprintf(timedisp,"%02d-%02d-%d", tm->tm_mday, tm->tm_mon +1, tm->tm_year+1900);
			tft.setCursor(30, 30);
			tft.setTextColor(ILI9341_YELLOW);
			tft.setTextSize(3);
			tft.println(timedisp);
		}
		if (hour != tm->tm_hour)
		{
			hour = tm->tm_hour;
			tft.fillRect(70, 80, 35, 20, ILI9341_BLACK);
			sprintf(timedisp, "%02d:", hour);
			tft.setCursor(70, 80);
			tft.setTextColor(ILI9341_CYAN);
			tft.setTextSize(2);
			tft.println(timedisp);
		}
		if (min != tm->tm_min || min == 0)
		{
			min = tm->tm_min;
			tft.fillRect(105, 80, 35, 20, ILI9341_BLACK);
			sprintf(timedisp, "%02d:", min);
			tft.setCursor(105, 80);
			tft.setTextColor(ILI9341_CYAN);
			tft.setTextSize(2);
			tft.println(timedisp);
		}
		if (sec != tm->tm_sec)
		{
			sec = tm->tm_sec;
			tft.fillRect(140, 80, 35, 20, ILI9341_BLACK);
			sprintf(timedisp, "%02d", sec);
			tft.setCursor(140, 80);
			tft.setTextColor(ILI9341_CYAN);
			tft.setTextSize(2);
			tft.println(timedisp);
		}
		//sprintf(timedisp,"%02d:%02d:%02d", tm->tm_hour, tm->tm_min, tm->tm_sec);
		//tft.fillRect(100, 80, 100, 20, ILI9341_BLACK);
		//tft.setCursor(100, 80);
		//tft.setTextColor(ILI9341_CYAN);
		//tft.setTextSize(2);
		//tft.println(timedisp);
		sleep(1);
	}


#ifdef __XENO__
	pthread_set_mode_np(PTHREAD_WARNSW, 0);
#endif

	return 1;
}

/*
g++ -o ./graphicstest ./testLCD.cpp ./ILI9341.cpp ./GFX.cpp 
gives filesize=41628 and benchmark:
Setting up TFT...
Setup SW SPI (Bit-Banger) parameters...
Displayed message
Benchmark               Time (microseconds)
Screen fill             2476926.000000
Text                    115884.000000
Lines                   1073453.000000
Horiz/Vert Lines        195891.000000
Rectangles (outline)    124732.000000
Rectangles (filled)     5176136.000000
Circles (filled)        688208.000000
Circles (outline)       482365.000000
Triangles (outline)     341616.000000
Triangles (filled)      1669839.000000
Rounded rects(outline)  227497.000000
Rounded rects (filled)  5579298.000000
Done!

g++ -o ./graphicstest ./testLCD.cpp ./ILI9341.cpp ./GFX.cpp -O3
gives filesize=36831 and benchmark:
Setting up TFT...
Setup SW SPI (Bit-Banger) parameters...
Displayed message
Benchmark               Time (microseconds)
Screen fill             998397.000000
Text                    44388.000000
Lines                   408695.000000
Horiz/Vert Lines        74615.000000
Rectangles (outline)    47921.000000
Rectangles (filled)     1915957.000000
Circles (filled)        262687.000000
Circles (outline)       179901.000000
Triangles (outline)     129693.000000
Triangles (filled)      618841.000000
Rounded rects(outline)  87533.000000
Rounded rects (filled)  2146365.000000
Done!
*/