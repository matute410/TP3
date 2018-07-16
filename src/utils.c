/*
 * utils.c
 *
 *  Created on: 20 de jun. de 2018
 *      Author: Matute
 */

#include "utils.h"
#include "board.h"
#include "sapi.h"


void initHardware(void)
{
   /* Inicializar la placa */
   boardConfig();

   /* Inicializar Uart */
   DEBUG_PRINT_ENABLE;
   debugPrintConfigUart( UART_USB, 115200 );

   /* Inicializo los sensores */
   dht11Init(GPIO1);

   adcConfig(ADC_ENABLE);
}


void vPrintString( char * string)
{
   uartWriteString( UART_USB, string );
}


void vPrintNumber( int32_t number)
{
   uint8_t uartBuff[10];
   /* Conversión de number entero a ascii con base decimal */
   itoa( number, uartBuff, 10 ); /* 10 significa decimal */
   /* Enviar number */
   uartWriteString( UART_USB, uartBuff );
}


void vPrintStringAndNumber( char * string, int32_t number)
{
   vPrintString( string );
   vPrintNumber( number );
   vPrintString( "\r\n" );
}


void format(float valor, char *dst, uint8_t pos)
{
	uint16_t val;
	val = 10 * valor;
	val = val % 1000;
	dst[pos] = (val / 100) + '0';
	pos++;
	dst[pos] = (val % 100) / 10 + '0';
	pos++;
	dst[pos] = '.';
	pos++;
	dst[pos] = (val % 10)  + '0';
	pos++;
	dst[pos] = '\0';
}

