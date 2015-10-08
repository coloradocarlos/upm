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
#include <fstream>
#include <unistd.h>
#include <stdlib.h>

#include "gfx.h"

using namespace upm;

GFX::GFX (int width, int height, uint8_t * screenBuffer, const unsigned char * font) : WIDTH(width), HEIGHT(height) {
    m_height = height;
    m_width  = width;
    m_font   = font;
    m_map    = screenBuffer;
}

GFX::~GFX () {
}

mraa_result_t
GFX::setPixel (int x, int y, uint16_t color) {
    if((x < 0) ||(x >= m_width) || (y < 0) || (y >= m_height)) {
        return MRAA_ERROR_UNSPECIFIED;
    }

    int index = ((y * m_width) + x) * sizeof(uint16_t);
    m_map[index] = (uint8_t) (color >> 8);
    m_map[++index] = (uint8_t)(color);

    return MRAA_SUCCESS;
}

void
GFX::fillScreen (uint16_t color) {
    fillRect(0, 0, m_width, m_height, color);
}

void
GFX::fillRect (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    for (int16_t i=x; i<x+w; i++) {
        drawFastVLine(i, y, h, color);
    }
}

void
GFX::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    drawLine(x, y, x, y+h-1, color);
}

void
GFX::drawLine (int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);

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
    dy = abs (y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (steep) {
            setPixel(y0, x0, color);
        } else {
            setPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

void
GFX::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    drawLine(x0, y0, x1, y1, color);
    drawLine(x1, y1, x2, y2, color);
    drawLine(x2, y2, x0, y0, color);
}

void
GFX::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    setPixel(x0  , y0+r, color);
    setPixel(x0  , y0-r, color);
    setPixel(x0+r, y0  , color);
    setPixel(x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;

        ddF_x += 2;
        f += ddF_x;

        setPixel(x0 + x, y0 + y, color);
        setPixel(x0 - x, y0 + y, color);
        setPixel(x0 + x, y0 - y, color);
        setPixel(x0 - x, y0 - y, color);
        setPixel(x0 + y, y0 + x, color);
        setPixel(x0 - y, y0 + x, color);
        setPixel(x0 + y, y0 - x, color);
        setPixel(x0 - y, y0 - x, color);
    }
}

void
GFX::setCursor (int16_t x, int16_t y) {
    m_cursorX = x;
    m_cursorY = y;
}

void
GFX::setTextColor (uint16_t textColor, uint16_t textBGColor) {
    m_textColor   = textColor;
    m_textBGColor = textBGColor;
}

void
GFX::setTextSize (uint8_t size) {
    m_textSize = (size > 0) ? size : 1;
}

void
GFX::setTextWrap (uint8_t wrap) {
    m_wrap = wrap;
}

void
GFX::drawChar (int16_t x, int16_t y, uint8_t data, uint16_t color, uint16_t bg, uint8_t size) {
    if( (x >= m_width)            || // Clip right
        (y >= m_height)           || // Clip bottom
        ((x + 6 * size - 1) < 0)  || // Clip left
        ((y + 8 * size - 1) < 0))    // Clip top
    return;

    for (int8_t i=0; i<6; i++ ) {
        uint8_t line;
        if (i == 5) {
            line = 0x0;
        } else {
            line = *(m_font+(data * 5)+i);
            for (int8_t j = 0; j<8; j++) {
                if (line & 0x1) {
                    if (size == 1) // default size
                        setPixel (x+i, y+j, color);
                    else {  // big size
                        fillRect (x+(i*size), y+(j*size), size, size, color);
                    }
                } else if (bg != color) {
                    if (size == 1) // default size
                        setPixel (x+i, y+j, bg);
                    else {  // big size
                        fillRect (x+i*size, y+j*size, size, size, bg);
                    }
                }
                line >>= 1;
            }
        }
    }
}

void
GFX::print (std::string msg) {
    int len = msg.length();

    for (int idx = 0; idx < len; idx++) {
        if (msg[idx] == '\n') {
            m_cursorY += m_textSize * 8;
            m_cursorX  = 0;
        } else if (msg[idx] == '\r') {
            // skip em
        } else {
            drawChar(m_cursorX, m_cursorY, msg[idx], m_textColor, m_textBGColor, m_textSize);
            m_cursorX += m_textSize * 6;
            if (m_wrap && (m_textColor > (m_width - m_textSize * 6))) {
                m_cursorY += m_textSize * 8;
                m_cursorX = 0;
            }
        }
    }
}

// See https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/examples/spitftbitmap/spitftbitmap.ino

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 20

void GFX::drawBitmap(const char* fileName, uint8_t x, uint8_t y) {
    uint32_t bmpImageoffset;        // Start of image data in file
    int      bmpWidth, bmpHeight;   // W+H in pixels
    uint8_t  bmpDepth;              // Bit depth (currently must be 24)
    uint32_t rowSize;               // Not always = bmpWidth; may have padding
    bool     flip    = true;        // BMP is stored bottom-to-top
    int      w, h, row, col;
    uint8_t  r, g, b;
    uint32_t pos = 0; //, startTime = millis();
    uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
    uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer

    // Open BMP file
    std::ifstream bmpFile(fileName, std::ios::binary);

    // Check really open
    if (!bmpFile.is_open()) {
        fprintf (stderr, "%s: BMP file not open", __FUNCTION__);
        return;
    }

    // BMP signature
    if (read16(bmpFile) != 0x4D42) {
        fprintf (stderr, "%s: File missing BMP signature", __FUNCTION__);
        bmpFile.close();
        return;
    }

    // Ignore ile size
    read32(bmpFile);

    // Ignore creator bytes
    read32(bmpFile);

    // Start of image data
    bmpImageoffset = read32(bmpFile);

    // Ingore DIB header
    read32(bmpFile);

    // Width / height
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);

    // # planes -- must be '1'
    if (read16(bmpFile) != 1) {
        fprintf (stderr, "%s: Number of planes must be 1", __FUNCTION__);
        bmpFile.close();
        return;
    }

    // bits per pixel
    bmpDepth = read16(bmpFile);
    if (bmpDepth != 24) {
        fprintf (stderr, "%s: Bit depth must be 24", __FUNCTION__);
        bmpFile.close();
        return;
    }

    // 0 = uncompressed
    if (read32(bmpFile) != 0) {
        fprintf (stderr, "%s: Bitmap not uncompressed", __FUNCTION__);
        bmpFile.close();
        return;
    }

    // BMP rows are padded (if needed) to 4-byte boundary
    rowSize = (bmpWidth * 3 + 3) & ~3;

    // If bmpHeight is negative, image is in top-down order.
    // This is not canon but has been observed in the wild.
    if (bmpHeight < 0) {
        bmpHeight = -bmpHeight;
        flip      = false;
    }

    // Crop area to be loaded
    w = bmpWidth;
    h = bmpHeight;
    if ((x+w-1) >= m_width)  w = m_width  - x;
    if ((y+h-1) >= m_height) h = m_height - y;

    // Set TFT address window to clipped image bounds
    setAddrWindow(x, y, x+w-1, y+h-1);

    // For each scanline...
    for (row=0; row<h; row++) {
        // Seek to start of scan line.  It might seem labor-
        // intensive to be doing this on every line, but this
        // method covers a lot of gritty details like cropping
        // and scanline padding.  Also, the seek only takes
        // place if the file position actually needs to change
        // (avoids a lot of cluster math in SD library).
        if (flip) {
            // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
        } else {
            // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
        }

        // Need seek
        if (bmpFile.tellg() != pos) {
            bmpFile.clear();
            bmpFile.seekg(pos, std::ios::beg);
            buffidx = sizeof(sdbuffer); // Force buffer reload
        }

        for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
                bmpFile.read((char *)&sdbuffer, sizeof(sdbuffer));
                buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            setPixel(col, row, toColor565(r,g,b));
        } // end pixel
    } // end scanline

    bmpFile.close();
    return;
}

uint16_t GFX::read16(std::ifstream &f) {
    uint16_t result;
    f.read(((char *)&result), 2);
    return result;
}

uint32_t GFX::read32(std::ifstream &f) {
    uint32_t result;
    f.read(((char *)&result), 4);
    return result;
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t GFX::toColor565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
