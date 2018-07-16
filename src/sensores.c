/*
 * sensores.c
 *
 *  Created on: 19 de jun. de 2018
 *      Author: Matute
 */

#include "sensores.h"
#include "sapi_dht11.h"
#include "utils.h"
#include "portmacro.h"


void inicializar_sensor(sensor_t * sen, char * nom, int umb, leer_sensor_t pf1, escribir_sensor_t pf2, funcion_control_t pf3)
{
	sen->nombre = nom;
	sen->umbral = umb;
	sen->leer_sensor = pf1;
	sen->escribir_sensor = pf2;
	sen->funcion_control = pf3;
}

bool_t leer_temperatura(sensor_t *sensor)
{
	static float med_temp;
	static int med_hum;

    dht11Read(&med_hum, &med_temp);

    sensor->medicion = &med_temp;

    return 0;
}

void escribir_temperatura(sensor_t *temp, charLcd_t *lcd)
{
	char med[5];
	char buff[16];

	charLcdGoToXY(*lcd, 1, 1);
	charLcdWrite(*lcd, temp->nombre);
    charLcdGoToXY(*lcd, 3, 2);
    format(*(tipo_temperatura_t *)temp->medicion, med, 0);
    sprintf(buff, "T:%s%cC", med, '\337');
	charLcdWrite(*lcd, buff);
}

bool_t leer_humedad(sensor_t *sensor)
{
	// TODO optimizar funcion para leer una sola vez y despues usar el otro valor.
	static float med_temp;
	static int med_hum;

    dht11Read(&med_hum, &med_temp);

    sensor->medicion = &med_hum;

    return 0;
}

void escribir_humedad(sensor_t *hum, charLcd_t *lcd)
{
	char buff[10];

	charLcdGoToXY(*lcd, 1, 1);
	charLcdWrite(*lcd, hum->nombre);
    charLcdGoToXY(*lcd, 3, 2);
	sprintf(buff, "H:%d%% ", *(tipo_humedad_ambiente_t *)hum->medicion);
	charLcdWrite(*lcd, buff);
}

bool_t leer_humedad_suelo(sensor_t *sensor)
{
	static int med_hum_sue;

	med_hum_sue = adcRead( CH1 );
	med_hum_sue = (1-((float)med_hum_sue/1024))*100;

    sensor->medicion = &med_hum_sue;

    return 0;
}

void escribir_humedad_suelo(sensor_t *hum_sue, charLcd_t *lcd)
{
	char buff[8];

	charLcdGoToXY(*lcd, 1, 1);
	charLcdWrite(*lcd, hum_sue->nombre);
    charLcdGoToXY(*lcd, 3, 2);
	sprintf(buff, "H:%d%% ", *(tipo_humedad_suelo_t *)hum_sue->medicion);
	charLcdWrite(*lcd, buff);
}

void funcion_control_temperatura(sensor_t *temp, charLcd_t *lcd)
{
	charLcdClear(*lcd);
	charLcdGoToXY(*lcd, 1, 1);
	charLcdWrite(*lcd, "Activando Sist.");
	charLcdGoToXY(*lcd, 1, 2);
	charLcdWrite(*lcd, "control temp.");
	gpioWrite(LEDR, 1);
	vTaskDelay(1000/portTICK_RATE_MS);
	gpioWrite(LEDR, 0);
	charLcdClear(*lcd);
	charLcdGoToXY(*lcd, 1, 1);
}

void funcion_control_humedad_ambiente(sensor_t * hum_amb, charLcd_t *lcd)
{
	charLcdClear(*lcd);
	charLcdGoToXY(*lcd, 1, 1);
	charLcdWrite(*lcd, "Activando Sist.");
	charLcdGoToXY(*lcd, 1, 2);
	charLcdWrite(*lcd, "control hum amb.");
	gpioWrite(LEDR, 1);
	vTaskDelay(1000/portTICK_RATE_MS);
	gpioWrite(LEDR, 0);
	charLcdClear(*lcd);
	charLcdGoToXY(*lcd, 1, 1);
}

void funcion_control_humedad_suelo(sensor_t * hum_sue, charLcd_t *lcd)
{
	charLcdClear(*lcd);
	charLcdGoToXY(*lcd, 1, 1);
	charLcdWrite(*lcd, "Activando Sist.");
	charLcdGoToXY(*lcd, 1, 2);
	charLcdWrite(*lcd, "control hum sue.");
	gpioWrite(GPIO5, 1);
	gpioWrite(LEDR, 1);
	vTaskDelay(2000/portTICK_RATE_MS);
	gpioWrite(LEDR, 0);
	gpioWrite(GPIO5, 0);
	charLcdClear(*lcd);
	charLcdGoToXY(*lcd, 1, 1);
}
