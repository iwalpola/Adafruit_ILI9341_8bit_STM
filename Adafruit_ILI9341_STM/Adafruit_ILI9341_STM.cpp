/*
See rights and use declaration in License.h
This library has been modified for the Maple Mini.
Includes DMA transfers on DMA1 CH2 and CH3.
*/
#include <Adafruit_ILI9341_STM.h>
#include <avr/pgmspace.h>
#include <limits.h>
#include <libmaple/dma.h>
#include "pins_arduino.h"
#include "wiring_private.h"


// Constructor when using 8080 mode of control.
Adafruit_ILI9341_STM::Adafruit_ILI9341_STM(void)
: Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
  
  pinMode(TFT_CS, OUTPUT);    // Enable outputs
  pinMode(TFT_CD, OUTPUT);
  pinMode(TFT_WR, OUTPUT);
  pinMode(TFT_RD, OUTPUT);
  #if defined(__STM32F1__)
    CS_IDLE; // Set all control bits to HIGH (idle)
    CD_IDLE; // Signals are ACTIVE LOW
    WR_IDLE;
    RD_IDLE;
  #endif
  if(TFT_RST) {
    pinMode(TFT_RST, OUTPUT);
    digitalWrite(TFT_RST, HIGH);
  }
  //probably should set up 8 bit parallel port to write mode.
  setupDataBus();
}

void Adafruit_ILI9341_STM::setupDataBus(void) {
  //set the pins to output mode
  //optimize later with CRL and CRH register
  for (uint8_t i = 0; i <= 7; i++){
    pinMode(DPINS[i], OUTPUT);
  }
}

void Adafruit_ILI9341_STM::write8(uint8_t c) {

  //retain values of A8-A15, and update A0-A7
  CS_ACTIVE;
  TFT_DATA->regs->ODR = ((TFT_DATA->regs->ODR | 0x00FF) | (c));//FF is Binary 0000000011111111
  WR_STROBE;
  CS_IDLE;
}


void Adafruit_ILI9341_STM::writecommand(uint8_t c) {
  CD_COMMAND;

  write8(c);
}


void Adafruit_ILI9341_STM::writedata(uint8_t c) {
  CD_DATA;

  write8(c);
}

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_ILI9341_STM::commandList(uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while (numCommands--) {                // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while (numArgs--) {                  //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if (ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if (ms == 255) ms = 500;    // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


void Adafruit_ILI9341_STM::begin(void) {

  // toggle RST low to reset
  if (TFT_RST > 0) {
    digitalWrite(TFT_RST, HIGH);
    delay(5);
    digitalWrite(TFT_RST, LOW);
    delay(20);
    digitalWrite(TFT_RST, HIGH);
    delay(150);
  }

  /*
  uint8_t x = readcommand8(ILI9341_RDMODE);
  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
  */
  //if(cmdList) commandList(cmdList);

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
  writedata(0x3e); //�Աȶȵ���
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
  delay(120);
  writecommand(ILI9341_DISPON);    //Display on

}


void Adafruit_ILI9341_STM::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
                                        uint16_t y1) {								
  writecommand(ILI9341_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9341_PASET); // Row addr set
  writedata(y0 >> 8);
  writedata(y0);     // YSTART
  writedata(y1 >> 8);
  writedata(y1);     // YEND

  writecommand(ILI9341_RAMWR); // write to RAM

}


void Adafruit_ILI9341_STM::pushColor(uint16_t color) {
  writedata(color >> 8);
  writedata(color);
}

void Adafruit_ILI9341_STM::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x, y, x + 1, y + 1);

  writedata(color >> 8);
  writedata(color);

}


void Adafruit_ILI9341_STM::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                        uint16_t color) {

  // Rudimentary clipping
  if ((x >= _width) || (y >= _height || h < 1)) return;

  if ((y + h - 1) >= _height)
    h = _height - y;
  if (h < 2 ) {
	drawPixel(x, y, color);
	return;
  }

  //  if (hwSPI) spi_begin();
  setAddrWindow(x, y, x, y + h - 1);
  
  uint8_t hi = color >> 8, lo = color;
  while (h--) {
    writedata(hi);
    writedata(lo);
  }

}


void Adafruit_ILI9341_STM::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                        uint16_t color) {

  
  // Rudimentary clipping
  if ((x >= _width) || (y >= _height || w < 1)) return;
  if ((x + w - 1) >= _width)  w = _width - x;
  if (w < 2 ) {
	drawPixel(x, y, color);
	return;
  }

//  if (hwSPI) spi_begin();
  setAddrWindow(x, y, x + w - 1, y);

	uint8_t hi = color >> 8, lo = color;
  while (w--) {
    write8(hi);
    write8(lo);
  }
}

void Adafruit_ILI9341_STM::fillScreen(uint16_t color) {

  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ILI9341_STM::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                   uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if ((x >= _width) || (y >= _height || h < 1 || w < 1)) return;
  if ((x + w - 1) >= _width)  w = _width  - x;
  if ((y + h - 1) >= _height) h = _height - y;
  if (w == 1 && h == 1) {
    drawPixel(x, y, color);
    return;
  }
  
  setAddrWindow(x, y, x + w - 1, y + h - 1);


  uint8_t hi = color >> 8, lo = color;
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--){
      writedata(hi);
      writedata(lo);
    }
  }

}

/*
* Draw lines faster by calculating straight sections and drawing them with fastVline and fastHline.
*/
#if defined (__STM32F1__)
void Adafruit_ILI9341_STM::drawLine(int16_t x0, int16_t y0,int16_t x1, int16_t y1, uint16_t color)
{
	if ((y0 < 0 && y1 <0) || (y0 > _height && y1 > _height)) return;
	if ((x0 < 0 && x1 <0) || (x0 > _width && x1 > _width)) return;
	if (x0 < 0) x0 = 0;
	if (x1 < 0) x1 = 0;
	if (y0 < 0) y0 = 0;
	if (y1 < 0) y1 = 0;

	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		}
		else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		}
		else {
			drawPixel(x0, y0, color);
		}
		return;
	}
	else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		}
		else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	}
	else {
		ystep = -1;
	}

	int16_t xbegin = x0;
	lineBuffer[0] = color;
	//*csport &= ~cspinmask;
	if (steep) {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastVLine (y0, xbegin, len + 1, color);
					//writeVLine_cont_noCS_noFill(y0, xbegin, len + 1);
				}
				else {
					drawPixel(y0, x0, color);
					//writePixel_cont_noCS(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			//writeVLine_cont_noCS_noFill(y0, xbegin, x0 - xbegin);
			drawFastVLine(y0, xbegin, x0 - xbegin, color);
		}

	}
	else {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastHLine(xbegin, y0, len + 1, color);
					//writeHLine_cont_noCS_noFill(xbegin, y0, len + 1);
				}
				else {
					drawPixel(x0, y0, color);
					//writePixel_cont_noCS(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			//writeHLine_cont_noCS_noFill(xbegin, y0, x0 - xbegin);
			drawFastHLine(xbegin, y0, x0 - xbegin, color);
		}
	}
	//*csport |= cspinmask;
}
#endif



// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ILI9341_STM::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_ILI9341_STM::setRotation(uint8_t m) {

  writecommand(ILI9341_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
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


void Adafruit_ILI9341_STM::invertDisplay(boolean i) {
  writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
}


////////// stuff not actively being used, but kept for posterity


// uint8_t Adafruit_ILI9341_STM::spiread(void) {
//   uint8_t r = 0;

//   if (hwSPI) {
// #if defined (__AVR__)
//     uint8_t backupSPCR = SPCR;
//     SPCR = mySPCR;
//     SPDR = 0x00;
//     while (!(SPSR & _BV(SPIF)));
//     r = SPDR;
//     SPCR = backupSPCR;
// #elif defined(TEENSYDUINO)
//     r = SPI.transfer(0x00);
// #elif defined (__STM32F1__)
//     r = SPI.transfer(0x00);
// #elif defined (__arm__)
//     SPI.setClockDivider(11); // 8-ish MHz (full! speed!)
//     SPI.setBitOrder(MSBFIRST);
//     SPI.setDataMode(SPI_MODE0);
//     r = SPI.transfer(0x00);
// #endif
//   } else {

//     for (uint8_t i = 0; i < 8; i++) {
//       digitalWrite(_sclk, LOW);
//       digitalWrite(_sclk, HIGH);
//       r <<= 1;
//       if (digitalRead(_miso))
//         r |= 0x1;
//     }
//   }
//   //Serial.print("read: 0x"); Serial.print(r, HEX);

//   return r;
// }

// uint8_t Adafruit_ILI9341_STM::readdata(void) {
//   digitalWrite(_dc, HIGH);
//   digitalWrite(_cs, LOW);
//   uint8_t r = spiread();
//   digitalWrite(_cs, HIGH);

//   return r;
// }


// uint8_t Adafruit_ILI9341_STM::readcommand8(uint8_t c, uint8_t index) {
//   if (hwSPI) spi_begin();
//   digitalWrite(_dc, LOW); // command
//   digitalWrite(_cs, LOW);
//   write8(0xD9);  // woo sekret command?
//   digitalWrite(_dc, HIGH); // data
//   write8(0x10 + index);
//   digitalWrite(_cs, HIGH);

//   digitalWrite(_dc, LOW);
//   //if(hwSPI) digitalWrite(_sclk, LOW);
//   digitalWrite(_cs, LOW);
//   write8(c);

//   digitalWrite(_dc, HIGH);
//   uint8_t r = spiread();
//   digitalWrite(_cs, HIGH);
//   if (hwSPI) spi_end();
//   return r;
// }



/*

 uint16_t Adafruit_ILI9341_STM::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);

 write8(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);

 pinMode(_sid, OUTPUT); // back to output
 return r;
 }

 uint32_t Adafruit_ILI9341_STM::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 write8(c);
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
