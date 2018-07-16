/*
 * maquina_estados.c
 *
 *  Created on: 21 de jun. de 2018
 *      Author: Matute
 */

#include "maquina_estados.h"
#include "sapi_peripheral_map.h"
#include "sapi_datatypes.h"
#include "sapi_gpio.h"
#include "queue.h"

void fsm_maquina_pantalla_lectura_tecla_pantalla(estado_pantalla_t *estado, int estado_siguiente)
{
	// TODO resolver que pasa si el usuario mantiene presionado el boton

	if (gpioRead(TECLA_ENCENDIDO_PANTALLA) == FALSE)
	{
		*estado = estado_siguiente;
	}
}

void fsm_maquina_pantalla_apagar_pantalla(estado_pantalla_t *estado, pantalla_IO_t *pantalla)
{
	*estado = PANTALLA_APAGADA;
	charLcdBacklightSet(&(pantalla->lcd), CHAR_LCD_BACKLIGHT_OFF);
	charLcdDisplaySet(&(pantalla->lcd), CHAR_LCD_DISPLAY_OFF);
}

void fsm_maquina_pantalla_prender_pantalla(estado_pantalla_t *estado, pantalla_IO_t *pantalla)
{
	*estado = PANTALLA_ENCENDIDA;
	charLcdDisplaySet(&(pantalla->lcd), CHAR_LCD_DISPLAY_ON);
	charLcdBacklightSet(&(pantalla->lcd), CHAR_LCD_BACKLIGHT_ON);
}

void fsm_maquina_pantalla_control_pantalla(pantalla_IO_t *pantalla)
{
	static estado_pantalla_t estado = PANTALLA_ENCENDIDA;
	static int tiempo_pantalla_ON = 0;

	if (tiempo_pantalla_ON >= TIEMPO_PANTALLA_ON)
	{
		estado = APAGAR_PANTALLA;
		tiempo_pantalla_ON = 0;
	}

	switch (estado)
	{
		case PANTALLA_ENCENDIDA:
			fsm_maquina_pantalla_lectura_tecla_pantalla(&estado, APAGAR_PANTALLA);
			tiempo_pantalla_ON++;
			break;
		case APAGAR_PANTALLA:
			fsm_maquina_pantalla_apagar_pantalla(&estado, pantalla);
			pantalla->estado_pantalla = PANTALLA_APAGADA;
			tiempo_pantalla_ON = 0;
			break;
		case PANTALLA_APAGADA:
			fsm_maquina_pantalla_lectura_tecla_pantalla(&estado, ENCENDER_PANTALLA);
			break;
		case ENCENDER_PANTALLA:
			fsm_maquina_pantalla_prender_pantalla(&estado, pantalla);
			pantalla->estado_pantalla = PANTALLA_ENCENDIDA;
			break;
		default:
			estado = PANTALLA_ENCENDIDA;
			break;
	}
}

void fsm_maquina_sensores_leer_sensores(pantalla_IO_t *pantalla)
{
	static estado_leer_sensor_t estado_leer = LEER_IDLE;

	switch (estado_leer)
	{
		case LEER_IDLE:
			if(pantalla->contador_lectura_sensores++ >= TIEMPO_LECTURA)
			{
				estado_leer = LEER_TEMPERATURA;
				pantalla->contador_lectura_sensores = 0;
			}
			break;
		case LEER_TEMPERATURA:
			pantalla->sensores[SENSOR_TEMPERATURA].leer_sensor(&(pantalla->sensores[SENSOR_TEMPERATURA]));
			estado_leer = LEER_HUMEDAD_AMBIENTE;
			break;
		case LEER_HUMEDAD_AMBIENTE:
			pantalla->sensores[SENSOR_HUMEDAD_AMBIENTE].leer_sensor(&(pantalla->sensores[SENSOR_HUMEDAD_AMBIENTE]));
			estado_leer = LEER_HUMEDAD_SUELO;
			break;
		case LEER_HUMEDAD_SUELO:
			pantalla->sensores[SENSOR_HUMEDAD_SUELO].leer_sensor(&(pantalla->sensores[SENSOR_HUMEDAD_SUELO]));
			pantalla->dato_nuevo = TRUE;
			estado_leer = LEER_IDLE;
			break;
		default:
			estado_leer = LEER_IDLE;
			break;
	}
}

void fsm_imprimir_sensores_pantalla(pantalla_IO_t *pantalla)
{
	static estado_imprimir_sensor_t estado_imprimir = IMPRIMIR_TEMPERATURA;

	if (pantalla->estado_pantalla == PANTALLA_APAGADA)
	{
		return;
	}
	switch (estado_imprimir)
	{
		case IMPRIMIR_TEMPERATURA:
			fsm_imprimir_lectura_tecla(&estado_imprimir, IMPRIMIR_HUMEDAD_AMBIENTE, pantalla);
			break;
		case IMPRIMIR_HUMEDAD_AMBIENTE:
			fsm_imprimir_lectura_tecla(&estado_imprimir, IMPRIMIR_HUMEDAD_SUELO, pantalla);
			break;
		case IMPRIMIR_HUMEDAD_SUELO:
			fsm_imprimir_lectura_tecla(&estado_imprimir, IMPRIMIR_TEMPERATURA, pantalla);
			break;
		default:
			estado_imprimir = IMPRIMIR_TEMPERATURA; // Caso por defecto
			break;
	}
	pantalla->sensor_imprimir = estado_imprimir;
}

void fsm_imprimir_lectura_tecla(estado_imprimir_sensor_t *estado, estado_imprimir_sensor_t estado_siguiente, pantalla_IO_t *pantalla)
{
	if(gpioRead(TECLA_CAMBIO_LECTURA_SENSOR) == FALSE)
	{
			*estado = estado_siguiente;
			if(pantalla->estado_pantalla == PANTALLA_ENCENDIDA)
			{
				charLcdClear(pantalla->lcd);
				vTaskDelay(10/portTICK_RATE_MS);
			}
	}
}

void fsm_funcion_imprimir_pantalla(pantalla_IO_t * pantalla)
{
	if (pantalla->estado_pantalla == PANTALLA_APAGADA || pantalla->dato_nuevo == FALSE)
	{
		return;
	}
	switch(pantalla->sensor_imprimir)
	{
		case IMPRIMIR_TEMPERATURA:
			pantalla->sensores[SENSOR_TEMPERATURA].escribir_sensor(&(pantalla->sensores[SENSOR_TEMPERATURA]), &(pantalla->lcd));
			pantalla->dato_nuevo = FALSE;
			break;
		case IMPRIMIR_HUMEDAD_AMBIENTE:
			pantalla->sensores[SENSOR_HUMEDAD_AMBIENTE].escribir_sensor(&(pantalla->sensores[SENSOR_HUMEDAD_AMBIENTE]), &(pantalla->lcd));
			pantalla->dato_nuevo = FALSE;
			break;
		case IMPRIMIR_HUMEDAD_SUELO:
			pantalla->sensores[SENSOR_HUMEDAD_SUELO].escribir_sensor(&(pantalla->sensores[SENSOR_HUMEDAD_SUELO]), &(pantalla->lcd));
			pantalla->dato_nuevo = FALSE;
			break;
		default:
			pantalla->sensor_imprimir = IMPRIMIR_TEMPERATURA;
			break;
	}
}

void fsm_maquina_control_sensores(pantalla_IO_t *pantalla)
{
	static estado_control_sensores_t estado = CONTROL_IDLE;

	switch (estado)
	{
		case CONTROL_IDLE:
			if (pantalla->dato_nuevo == TRUE)
			{
				estado = CONTROL_SENSORES;
			}
			break;
		case CONTROL_SENSORES:
			fsm_maquina_control_sensor_temperatura(pantalla);
			fsm_maquina_control_sensor_humedad_ambiente(pantalla);
			fsm_maquina_control_sensor_humedad_suelo(pantalla);
			estado = CONTROL_IDLE;
			break;
		default:
			estado = CONTROL_IDLE;
			break;
	}
}

void fsm_maquina_control_sensor_temperatura(pantalla_IO_t * pantalla)
{
	int aux = *(tipo_temperatura_t *)(pantalla->sensores[SENSOR_TEMPERATURA].medicion);
	if (aux >= UMBRAL_TEMPERATURA)
	{
		pantalla->sistema_control = SISTEMA_CONTROL_TEMPERATURA;
	}
}

void fsm_maquina_control_sensor_humedad_ambiente(pantalla_IO_t * pantalla)
{
	if (*(tipo_humedad_ambiente_t *)pantalla->sensores[SENSOR_HUMEDAD_AMBIENTE].medicion >= UMBRAL_HUMEDAD_AMBIENTE)
	{
		pantalla->sistema_control = SISTEMA_CONTROL_HUMEDAD_AMBIENTE;
	}
}

void fsm_maquina_control_sensor_humedad_suelo(pantalla_IO_t * pantalla)
{
	if (*(tipo_humedad_suelo_t *)pantalla->sensores[SENSOR_HUMEDAD_SUELO].medicion <= UMBRAL_HUMEDAD_SUELO)
	{
		pantalla->sistema_control = SISTEMA_CONTROL_HUMEDAD_SUELO;
	}
}

void fsm_funcion_control_sensores(pantalla_IO_t *pantalla)
{
	switch (pantalla->sistema_control)
	{
		case OK:
			break;
		case SISTEMA_CONTROL_TEMPERATURA:
			pantalla->sensores[SENSOR_TEMPERATURA].funcion_control(&(pantalla->sensores[SENSOR_TEMPERATURA]), &(pantalla->lcd));
			pantalla->contador_lectura_sensores = TIEMPO_LECTURA;
			pantalla->sistema_control = OK;
			break;
		case SISTEMA_CONTROL_HUMEDAD_AMBIENTE:
			pantalla->sensores[SENSOR_HUMEDAD_AMBIENTE].funcion_control(&(pantalla->sensores[SENSOR_HUMEDAD_AMBIENTE]), &(pantalla->lcd));
			pantalla->contador_lectura_sensores = TIEMPO_LECTURA;
			pantalla->sistema_control = OK;
			break;
		case SISTEMA_CONTROL_HUMEDAD_SUELO:
			pantalla->sensores[SENSOR_HUMEDAD_SUELO].funcion_control(&(pantalla->sensores[SENSOR_HUMEDAD_SUELO]), &(pantalla->lcd));
			pantalla->contador_lectura_sensores = TIEMPO_LECTURA;
			pantalla->sistema_control = OK;
			break;
		default:
			pantalla->sistema_control = OK;
			break;
	}
}
