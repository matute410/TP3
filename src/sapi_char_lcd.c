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
 
/*==================[inclusions]==============================================*/

#include "sapi_char_lcd.h"

/*==================[macros and definitions]==================================*/
/*==================[internal data declaration]===============================*/
/*==================[internal functions declaration]==========================*/ 

// GPIO LCD low level functions
static void charLcdGpioInit( charLcd_t lcd );
static void charLcdGpioCommand( charLcd_t lcd, uint8_t cmd );
static void charLcdGpioData( charLcd_t lcd, uint8_t data );
static void charLcdGpioEnablePulse( charLcd_t lcd );
static void charLcdGpioSendNibble( charLcd_t lcd, uint8_t nibble );

// I2C LCD low level functions
static void charLcdi2cInit( charLcd_t lcd );
static void charLcdi2cCommand( charLcd_t lcd, uint8_t cmd );
static void charLcdi2cData( charLcd_t lcd, uint8_t data );

/*==================[external data declaration]===============================*/
/*==================[internal functions definition]===========================*/

static void charLcdGpioInit( charLcd_t lcd ){
   
   // Configure LCD Pins as Outputs
                 
   charLcdConfigPinAsOutput( lcd.gpios->rs );
   charLcdConfigPinAsOutput( lcd.gpios->rw );
   charLcdConfigPinAsOutput( lcd.gpios->en );
   
   charLcdConfigPinAsOutput( lcd.gpios->d4 );
   charLcdConfigPinAsOutput( lcd.gpios->d5 );
   charLcdConfigPinAsOutput( lcd.gpios->d6 );
   charLcdConfigPinAsOutput( lcd.gpios->d7 );

   // Configure LCD for 4-bit mode
                 
   charLcdPinWrite( lcd.gpios->rw, OFF );   // RW = 0
   charLcdPinWrite( lcd.gpios->rs, OFF );   // RS = 0
   charLcdPinWrite( lcd.gpios->en, OFF );   // EN = 0

   charLcdDelay_ms( CHAR_LCD_STARTUP_WAIT_MS ); // Wait for stable power
   
   charLcdGpioCommand( lcd, 0x33 );      // Command 0x33 for 4-bit mode
   charLcdCommandDelay();                // Wait
                 
   charLcdGpioCommand( lcd, 0x32 );      // Command 0x32 for 4-bit mode
   charLcdCommandDelay();                // Wait
                 
   charLcdGpioCommand( lcd, 0x20 );      // Command 0x20 for 4-bit mode and 1 line
   charLcdCommandDelay();                // Wait
   
   if( lcd.amountOfLines > 1 ){          // Configure for 2 lines or more
      charLcdGpioCommand( lcd, 0x28 );   // Command 0x28 for 4-bit mode and 2 lines
      charLcdCommandDelay();             // Wait
   }
   
   // Initialize LCD
   charLcdGpioCommand( lcd, 0x0C );      // Command 0x0E for display on, cursor off
   charLcdCommandDelay();                // Wait
   
   charLcdClear( lcd );                   // Command for clear LCD

   charLcdGpioCommand( lcd, 0x06 );      // Command 0x06 for Shift cursor right
   charLcdCommandDelay();                // Wait

   charLcdDelay_ms( 1 );                 // Wait*/
}

static void charLcdGpioCommand( charLcd_t lcd, uint8_t cmd ){
   
   charLcdGpioSendNibble( lcd, cmd & 0xF0 );     // Send high nibble to D7-D4
   
   charLcdPinWrite( lcd.gpios->rs, OFF );      // RS = 0 for command
   charLcdPinWrite( lcd.gpios->rw, OFF );      // RW = 0 for write

   charLcdGpioEnablePulse( lcd );
   charLcdDelay_us( CHAR_LCD_LOW_WAIT_US );  // Wait
   
   charLcdGpioSendNibble( lcd, cmd << 4 );       // Send low nibble to D7-D4
   charLcdGpioEnablePulse( lcd );
}

static void charLcdGpioData( charLcd_t lcd, uint8_t data ){
   
   charLcdGpioSendNibble( lcd, data & 0xF0 ); // Send high nibble to D7-D4
   
   charLcdPinWrite( lcd.gpios->rs, ON );    // RS = 1 for data
   charLcdPinWrite( lcd.gpios->rw, OFF );   // RW = 0 for write

   charLcdGpioEnablePulse( lcd );
   
   charLcdGpioSendNibble( lcd, data << 4 );   // Send low nibble to D7-D4
   charLcdGpioEnablePulse( lcd );
}

static void charLcdGpioEnablePulse( charLcd_t lcd ){
   
   charLcdPinWrite( lcd.gpios->en, ON );             // EN = 1 for H-to-L pulse
   charLcdDelay_us( CHAR_LCD_EN_PULSE_WAIT_US );   // Wait to make EN wider
   charLcdPinWrite( lcd.gpios->en, OFF );            // EN = 0 for H-to-L pulse
}

static void charLcdGpioSendNibble( charLcd_t lcd, uint8_t nibble ){
   
   charLcdPinWrite( lcd.gpios->d7, ( nibble & 0x80 ) );
   charLcdPinWrite( lcd.gpios->d6, ( nibble & 0x40 ) );
   charLcdPinWrite( lcd.gpios->d5, ( nibble & 0x20 ) );
   charLcdPinWrite( lcd.gpios->d4, ( nibble & 0x10 ) );
}

static void charLcdi2cInit( charLcd_t lcd ){
   // Config i2c port
   i2cConfig( I2C0, 10000 );
   
   // Config display
   charLcdi2cCommand( lcd, 0x02 );     // Command 0x02 set 4 bit mode for start
   charLcdi2cCommand( lcd, 0x20 );     // Command 0x20 set 4 bit mode and 1 lines
   
   // Config for 2 lines or more
   if( lcd.amountOfLines > 1 ){        
      charLcdi2cCommand( lcd, 0x28 );  // Command 0x28 set 4 bit mode and 2 lines
   }
   
   charLcdi2cCommand( lcd, 0x0C );     // Command 0x0c set display on, cursor off
   charLcdi2cCommand( lcd, 0x06 );     // Command 0x06 set default entry mode set
   charLcdi2cCommand( lcd, 0x80 );     // Command 0x80 set default character type
}

static void charLcdi2cCommand( charLcd_t lcd, uint8_t cmd ){
   
   uint8_t cmdHigh, cmdLow;
	uint8_t cmdNibble[4];
   uint8_t backlight = 0x00;
   
   // Set backlight status
   if(lcd.backlight == CHAR_LCD_BACKLIGHT_ON){
      backlight = 0x08;
   }  
	
   // Split byte in high an low 4bits
   cmdHigh = cmd & 0xf0;
	cmdLow = (cmd << 4) & 0xf0;
   
   // Format packet to send
	cmdNibble[0] = cmdHigh | 0x04 | backlight;    
	cmdNibble[1] = cmdHigh | backlight;      
	cmdNibble[2] = cmdLow  | 0x04 | backlight;    
	cmdNibble[3] = cmdLow  | backlight;      
   
   // 0x04 means EN=1 and RS=0
   // Always has to be send backlight state

   i2cWrite(I2C0, lcd.i2cAddress, (uint8_t *)cmdNibble, 4, TRUE);
}

static void charLcdi2cData( charLcd_t lcd, uint8_t data ){
   
   uint8_t dataHigh, dataLow;
	uint8_t dataNibble[4];
	uint8_t backlight = 0x00;
   
   // Set backlight status
   if(lcd.backlight == CHAR_LCD_BACKLIGHT_ON){
      backlight = 0x08;
   }  
	
   // Split byte in high an low 4bits
   dataHigh = data & 0xf0;
	dataLow = (data << 4) & 0xf0;
   
   // Format packet to send
	dataNibble[0] = dataHigh | 0x05 | backlight;    
	dataNibble[1] = dataHigh | 0x01 | backlight;      
	dataNibble[2] = dataLow  | 0x05 | backlight;    
	dataNibble[3] = dataLow  | 0x01 | backlight;      
   
   // 0x05 means EN=1 and RS=1
   // Always has to be send backlight state
   
   i2cWrite(I2C0, lcd.i2cAddress, (uint8_t *)dataNibble, 4, TRUE); 
}

/*==================[external functions definition]===========================*/

void charLcdInit( charLcd_t lcd ){
   
   if( lcd.mode == CHAR_LCD_I2C ){
      charLcdi2cInit( lcd );
   }
   else{
      charLcdGpioInit( lcd );
   }
}

void charLcdCommand( charLcd_t lcd, uint8_t cmd ){
   
   if( lcd.mode == CHAR_LCD_I2C ){
      charLcdi2cCommand( lcd, cmd );
   }
   else{
      charLcdGpioCommand( lcd, cmd );
   }
}

void charLcdData( charLcd_t lcd, uint8_t data ){
   
   if( lcd.mode == CHAR_LCD_I2C ){
      charLcdi2cData( lcd, data );
   }
   else{
      charLcdGpioData( lcd, data );
   }   
}

void charLcdWrite( charLcd_t lcd, char *str ){
   
   uint8_t currentColumn = 1;
   uint8_t currentRow = 1;
   
   // Reading chars
   while( *str ){
      
      // Auto-scroll lines
      if( currentColumn > lcd.lineWidth ){
         currentColumn = 1;
         currentRow++;
         
         if( currentRow > lcd.amountOfLines ){
            currentRow = 1;
         }
         
         charLcdGoToXY( lcd, currentColumn, currentRow );
      }
      
      // Send chars
      charLcdData( lcd, *str );
      str++;
      currentColumn++;
   }
}

void charLcdCreateChar( charLcd_t lcd, uint8_t charnum, const char *chardata ){
   charnum &= 0x07;
   
   charLcdCommand( lcd, 0x40 | (charnum << 3) );
   
   for( uint8_t i=0; i < 8; i++ ){
      charLcdData( lcd, chardata[i] );
   }
   
   // Return home for avoid errors
   charLcdReturnHome( lcd );
}

void charLcdGoToXY( charLcd_t lcd, uint8_t x, uint8_t y ){
	int i;
   uint8_t firstCharAdress[] = { 0x80, 0xC0, 0x94, 0xD4 };   // See table 12-5
   
   charLcdCommand( lcd, firstCharAdress[ y - 1 ] + x - 1 );
   for(i = 100; i > 0; i--);

   //charLcdDelay_us( CHAR_LCD_HIGH_WAIT_US ); // Wait
}

void charLcdClear( charLcd_t lcd ){
   
   charLcdCommand( lcd, 0x01 );                    // Command 0x01 for clear LCD
   charLcdDelay_ms( CHAR_LCD_CLR_DISP_WAIT_MS );   // Wait
}

void charLcdReturnHome( charLcd_t lcd ){

   charLcdCommand( lcd, 0x02 );                    // Command 0x02 for return cursor 
   charLcdDelay_ms( CHAR_LCD_RET_HOME_WAIT_MS );   // Wait
}

void charLcdCursorSet( charLcd_t *lcd, charLcdCursor_t value ){
   
   lcd->cursorType = value;
   charLcdCommand( *lcd, 0x08 | lcd->cursorType | lcd->displayStatus );
}

void charLcdDisplaySet( charLcd_t *lcd, charLcdDisplay_t value ){

   lcd->displayStatus = value;
   charLcdCommand( *lcd, 0x08 | lcd->cursorType | lcd->displayStatus );
}

void charLcdBacklightSet( charLcd_t *lcd, charLcdBacklight_t value ){
   
   /* This functions only works for i2c LCD */   
   if( lcd->mode == CHAR_LCD_GPIO ){
      return;
   }
   
   if( value == TRUE ){
      lcd->backlight = CHAR_LCD_BACKLIGHT_ON;
   }
   else{
      lcd->backlight = CHAR_LCD_BACKLIGHT_OFF;
   }
   
   // Only send backlight status with no data
   charLcdCommand( *lcd, 0 );
}

/*==================[end of file]=============================================*/
