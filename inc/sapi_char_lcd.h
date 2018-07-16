/* Copyright 2018, Eric Pernia & Ariel Berardi.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _SAPI_CHAR_LCD_H_
#define _SAPI_CHAR_LCD_H_

/*==================[inclusions]=============================================*/

#include "sapi.h"
#include "FreeRTOS.h"
#include "task.h"

/*==================[c++]====================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

// LCD delay Times
#define CHAR_LCD_EN_PULSE_WAIT_US        25    // 25 us
#define CHAR_LCD_LOW_WAIT_US             25    // 25 us
#define CHAR_LCD_HIGH_WAIT_US            100   // 100 us

#define CHAR_LCD_CMD_WAIT_US             45    // Wait time for every command 45 us, except:
#define CHAR_LCD_CLR_DISP_WAIT_MS        3     // - Clear Display 1.52 ms
#define CHAR_LCD_RET_HOME_WAIT_MS        3     // - Return Home  1.52 ms
                                               // - Read Busy flag and address 0 us
#define CHAR_LCD_STARTUP_WAIT_MS         3000  // 3000 ms

// LCD delay HAL
#define charLcdDelay_ms(duration)        vTaskDelay(duration / portTICK_RATE_MS)
#define charLcdDelay_us(duration)        vTaskDelay(duration / portTICK_RATE_MS / 1000) //delayUs(duration)
#define charLcdCommandDelay()            charLcdDelay_us(CHAR_LCD_CMD_WAIT_US)
#define charLcdConfigPinAsOutput(pin)    gpioConfig( (pin), GPIO_OUTPUT );
#define charLcdPinWrite( pin, value )    gpioWrite( (pin), (value) )

#define CHAR_LCD_GPIO_DECL( lcdName, lcdLineWidth, lcdAmountOfLines )   \
charLcdPin_t lcdName##Pins = {.rs = LCDRS,                              \
                              .en = LCDEN,                              \
                              .rw = 0,                                  \
                              .d0 = 0,                                  \
                              .d1 = 0,                                  \
                              .d2 = 0,                                  \
                              .d3 = 0,                                  \
                              .d4 = LCD1,                               \
                              .d5 = LCD2,                               \
                              .d6 = LCD3,                               \
                              .d7 = LCD4,                               \
                              .bl = GPIO0                               \
};                                                                      \
charLcd_t lcdName = { .mode    = CHAR_LCD_GPIO,                         \
                      .lineWidth     = lcdLineWidth,                    \
                      .amountOfLines = lcdAmountOfLines,                \
                      .gpios         = &(lcdName##Pins),                \
                      .dataBitSize   = CHAR_LCD_4_BITS,                 \
                      .backlight     = CHAR_LCD_BACKLIGHT_ON,           \
                      .cursorType    = CHAR_LCD_CURSOR_OFF,             \
                      .displayStatus = CHAR_LCD_DISPLAY_ON              \
} 

#define CHAR_LCD_I2C_DECL( lcdName, lcdLineWidth, lcdAmountOfLines,     \
                           lcdAddress)                                  \
charLcd_t lcdName = {.mode          = CHAR_LCD_I2C,                     \
                     .lineWidth     = lcdLineWidth,                     \
                     .amountOfLines = lcdAmountOfLines,                 \
                     .dataBitSize   = CHAR_LCD_4_BITS,                  \
                     .i2cAddress    = lcdAddress,                       \
                     .backlight     = CHAR_LCD_BACKLIGHT_ON,            \
                     .cursorType    = CHAR_LCD_CURSOR_OFF,              \
                     .displayStatus = CHAR_LCD_DISPLAY_ON               \
}

/*==================[typedef]================================================*/

// Enumeration defining the type of LCD
typedef enum{
   CHAR_LCD_GPIO,
   CHAR_LCD_I2C
}charLcdMode_t;

// Enumeration defining bits of GPIO LCD
typedef enum{
   CHAR_LCD_4_BITS,
   CHAR_LCD_8_BITS
}charLcdBitSize_t;

// Enumeration defining backlight status
typedef enum{
   CHAR_LCD_BACKLIGHT_OFF,
   CHAR_LCD_BACKLIGHT_ON
}charLcdBacklight_t;

// Enumeration defining display status
typedef enum{
   CHAR_LCD_DISPLAY_OFF = 0x00,
   CHAR_LCD_DISPLAY_ON  = 0x04
}charLcdDisplay_t;

// Enumerations diferents cursor types
typedef enum{
   CHAR_LCD_CURSOR_OFF     = 0x00,
   CHAR_LCD_CURSOR_ON      = 0x02,
   CHAR_LCD_CURSOR_BLINK   = 0x01
}charLcdCursor_t;

typedef struct{
   uint32_t rs;   // RS = 0 to select command register, RS = 1 to select data register
   uint32_t en;   // Enable
   uint32_t rw;   // R/W = 0 for write, R/W = 1 for read
   uint32_t d0;
   uint32_t d1;
   uint32_t d2;
   uint32_t d3;
   uint32_t d4;
   uint32_t d5;
   uint32_t d6;
   uint32_t d7;
   uint32_t bl;   // Backlight LED
}charLcdPin_t;

typedef struct{
   // Mode I2C or GPIO
   uint8_t mode;
   // Lines
   uint8_t lineWidth;
   uint8_t amountOfLines;
   // Char geometry
   uint8_t charWidth;
   uint8_t charHeight;
   // Configurations
   charLcdBacklight_t backlight;
   charLcdCursor_t cursorType;
   charLcdDisplay_t displayStatus; 
   // GPIO
   charLcdPin_t* gpios;
   charLcdBitSize_t dataBitSize; // 4 or 8 bits
   // I2C
   uint8_t i2cAddress;
}charLcd_t;

/*==================[external data declaration]==============================*/
/*==================[external functions declaration]=========================*/

// LCD low level functions
void charLcdCommand( charLcd_t lcd, uint8_t cmd );
void charLcdData( charLcd_t lcd, uint8_t data );

// LCD high level functions
void charLcdInit( charLcd_t lcd );
void charLcdWrite( charLcd_t lcd, char *str );   
void charLcdCreateChar( charLcd_t lcd, uint8_t charnum, const char *chardata );
void charLcdGoToXY( charLcd_t lcd, uint8_t x, uint8_t y );
void charLcdClear( charLcd_t lcd );
void charLcdReturnHome( charLcd_t lcd );
void charLcdCursorSet( charLcd_t *lcd, charLcdCursor_t value );
void charLcdDisplaySet( charLcd_t *lcd, charLcdDisplay_t value );
void charLcdBacklightSet( charLcd_t *lcd, charLcdBacklight_t value );

/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_CHAR_LCD_H_ */
