/*
 * Author: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <string>
#include <stdexcept>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "st7735.h"

using namespace upm;

ST7735::ST7735 (uint8_t csLCD, uint8_t cSD, uint8_t rs, uint8_t rst)
    : GFX (ST7735_TFTHEIGHT, ST7735_TFTWIDTH, m_map, font), m_csLCDPinCtx(csLCD), m_cSDPinCtx(cSD),
      m_rSTPinCtx(rst), m_rSPinCtx(rs), m_spi(0) {

    initModule ();
    configModule ();
}

void
ST7735::initModule () {
    mraa::Result error = mraa::SUCCESS;

    // m_height and m_width initialized in configModule() call to setRotation()

    // m_spi.frequency(15 * 1000000); // Do we need to initialize frequence here?

    error = m_csLCDPinCtx.dir(mraa::DIR_OUT);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    error = m_cSDPinCtx.dir(mraa::DIR_OUT);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    error = m_rSTPinCtx.dir(mraa::DIR_OUT);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    error = m_rSPinCtx.dir(mraa::DIR_OUT);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    if (mraa_get_platform_type() == MRAA_INTEL_EDISON_FAB_C) {
        error = m_spi.frequency(25 * 1000000);
    } else {
        error = m_spi.frequency(15 * 1000000);
    }
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    lcdCSOn ();
}

void
ST7735::write (uint8_t value) {
    rsLOW ();
    m_spi.writeByte(value);
}

void
ST7735::data (uint8_t value) {
    rsHIGH ();
    m_spi.writeByte(value);
}

void
ST7735::executeCMDList(const uint8_t *addr) {
    uint8_t  numCommands, numArgs;
    uint16_t ms;

    numCommands = *(addr++);           // Number of commands to follow
    while(numCommands--) {             // For each command...
        write (*(addr++));             // Read, issue command
        numArgs  = *(addr++);          // Number of args to follow
        ms       = numArgs & DELAY;    // If hibit set, delay follows args
        numArgs &= ~DELAY;             // Mask out delay bit
        while(numArgs--) {             // For each argument...
            data (*(addr++));          // Read, issue argument
        }

        if(ms) {
            ms = *(addr++);            // Read post-command delay time (ms)
            if (ms == 255) {
                ms = 500;              // If 255, delay for 500 ms
            }
            usleep (ms * 1000);
        }
    }
}

void
ST7735::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    uint8_t colstart, rowstart;
    colstart  = rowstart = 0;

    write (ST7735_CASET);                       // Column addr set

    rsHIGH ();
    m_spiBuffer[0] = 0x00;
    m_spiBuffer[1] = x0 + colstart;             // XSTART
    m_spiBuffer[2] = 0x00;
    m_spiBuffer[3] = x1 + colstart;             // XEND
    m_spi.write(m_spiBuffer, 4);

    write (ST7735_RASET);                       // Row addr set

    rsHIGH ();
    m_spiBuffer[0] = 0x00;
    m_spiBuffer[1] = y0 + rowstart;             // YSTART
    m_spiBuffer[2] = 0x00;
    m_spiBuffer[3] = y1 + rowstart;             // YEND
    m_spi.write(m_spiBuffer, 4);

    write (ST7735_RAMWR);                       // write to RAM
}

void
ST7735::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (mraa::SUCCESS != setPixel (x, y, color)) {
        return;
    }

    refresh ();
}

void
ST7735::refresh () {
    rsHIGH ();

    // Work around for ioctl() brain freeze on Edison breakout board
    int fragmentDivisor;
    if (mraa_get_platform_type() == MRAA_INTEL_EDISON_FAB_C) {
        fragmentDivisor = 2560;
    } else {
        fragmentDivisor = 20;
    }

    mraa::Result res = mraa::SUCCESS;
    int fragmentSize = m_height * m_width * 2 / fragmentDivisor;
    for (int fragment = 0; fragment < fragmentDivisor; fragment++) {
        // uint8_t* x = mraa_spi_write_buf(m_spi, &m_map[fragment * fragmentSize], fragmentSize);
        // free(x);
        res = m_spi.transfer(&m_map[fragment * fragmentSize], NULL, fragmentSize);
        // m_spi.write(&m_map[fragment * fragmentSize], fragmentSize);
        if (res != mraa::SUCCESS) {
            fprintf (stderr, "%s: Failed mraa_spi_transfer_buf(), mraa_result_t=%d", __FUNCTION__, res);
            break;
        }
    }
}

void
ST7735::configModule() {
    rsHIGH ();
    lcdCSOff ();
    lcdCSOn ();

    m_rSTPinCtx.write(HIGH);
    usleep (500000);
    m_rSTPinCtx.write(LOW);
    usleep (500000);
    m_rSTPinCtx.write(HIGH);
    usleep (500000);

    executeCMDList (Rcmd1);
    executeCMDList (Rcmd2red);
    executeCMDList (Rcmd3);

    setRotation(3); // also sets m_width and m_height

    setAddrWindow (0, 0, m_width - 1, m_height - 1);

    fillScreen (ST7735_BLACK);
    refresh ();
}

mraa::Result
ST7735::lcdCSOn () {
    mraa::Result error = mraa::SUCCESS;

    error = m_csLCDPinCtx.write(LOW);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    error = m_cSDPinCtx.write(HIGH);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    return error;
}

mraa::Result
ST7735::lcdCSOff () {
    mraa::Result error = mraa::SUCCESS;

    error = m_csLCDPinCtx.write(HIGH);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    return error;
}

mraa::Result
ST7735::sdCSOn () {
    mraa::Result error = mraa::SUCCESS;

    error = m_cSDPinCtx.write(LOW);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    error = m_csLCDPinCtx.write(HIGH);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    return error;
}

mraa::Result
ST7735::sdCSOff () {
    mraa::Result error = mraa::SUCCESS;

    error = m_cSDPinCtx.write(HIGH);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    return error;
}

mraa::Result
ST7735::rsHIGH () {
    mraa::Result error = mraa::SUCCESS;

    error = m_rSPinCtx.write(HIGH);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    return error;
}

mraa::Result
ST7735::rsLOW () {
    mraa::Result error = mraa::SUCCESS;

    error = m_rSPinCtx.write(LOW);
    if (error != mraa::SUCCESS) {
        mraa::printError (error);
    }

    return error;
}

// https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7735.cpp
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void
ST7735::setRotation(uint8_t r) {
    uint8_t rotation = r & 3; // 0 to 3 only
    uint8_t param; // MY MX MV ML RGB MH x x

    switch (rotation) {
        case 0:
            // 0 deg
            param = MADCTL_MX | MADCTL_MY | MADCTL_RGB;
            m_height = ST7735_TFTHEIGHT;
            m_width  = ST7735_TFTWIDTH;
            break;
        case 1:
            // 90 deg
            param = MADCTL_MY | MADCTL_MV | MADCTL_RGB;
            m_height = ST7735_TFTWIDTH;
            m_width  = ST7735_TFTHEIGHT;
            break;
        case 2:
            // 180 deg
            param = MADCTL_RGB;
            m_height = ST7735_TFTHEIGHT;
            m_width  = ST7735_TFTWIDTH;
            break;
        case 3:
            // 270 deg
            param = MADCTL_MX | MADCTL_MV | MADCTL_RGB;
            m_height = ST7735_TFTWIDTH;
            m_width  = ST7735_TFTHEIGHT;
            break;
    }

    write (ST7735_MADCTL);
    data(param);
}
