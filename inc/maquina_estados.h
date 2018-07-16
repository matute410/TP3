/*
 * maquina_estados.h
 *
 *  Created on: 21 de jun. de 2018
 *      Author: Matute
 */

#ifndef _INC_MAQUINA_ESTADOS_H_
#define _INC_MAQUINA_ESTADOS_H_

#include "maquina_estados.h"
#include "sapi_char_lcd.h"
#include "sensores.h"

#define TIEMPO_LECTURA 20
#define TIEMPO_PANTALLA_ON 250

#define CANTIDAD_SENSORES 3

#define TECLA_ENCENDIDO_PANTALLA TEC1
#define TECLA_CAMBIO_LECTURA_SENSOR TEC2

typedef enum
{
	PANTALLA_ENCENDIDA,
	APAGAR_PANTALLA,
	PANTALLA_APAGADA,
	ENCENDER_PANTALLA
}estado_pantalla_t;

typedef enum
{
	LEER_IDLE,
	LEER_TEMPERATURA,
	LEER_HUMEDAD_AMBIENTE,
	LEER_HUMEDAD_SUELO
}estado_leer_sensor_t;

typedef enum
{
	IMPRIMIR_TEMPERATURA,
	IMPRIMIR_HUMEDAD_AMBIENTE,
	IMPRIMIR_HUMEDAD_SUELO
}estado_imprimir_sensor_t;

typedef enum
{
	CONTROL_IDLE,
	CONTROL_SENSORES
}estado_control_sensores_t;

typedef enum
{
	OK,
	SISTEMA_CONTROL_TEMPERATURA,
	SISTEMA_CONTROL_HUMEDAD_AMBIENTE,
	SISTEMA_CONTROL_HUMEDAD_SUELO
}activar_sistema_control_t;

typedef struct
{
	sensor_t sensores[CANTIDAD_SENSORES];
	charLcd_t lcd;
	estado_pantalla_t estado_pantalla;
	activar_sistema_control_t sistema_control;
	bool_t dato_nuevo;
	estado_imprimir_sensor_t sensor_imprimir;
	int contador_lectura_sensores;
}pantalla_IO_t;

void fsm_maquina_pantalla_control_pantalla(pantalla_IO_t *);
void fsm_maquina_pantalla_prender_pantalla(estado_pantalla_t *, pantalla_IO_t *);
void fsm_maquina_pantalla_apagar_pantalla(estado_pantalla_t *, pantalla_IO_t *);
void fsm_maquina_pantalla_lectura_tecla_pantalla(estado_pantalla_t *, int);
void fsm_maquina_sensores_leer_sensores(pantalla_IO_t *);
void fsm_imprimir_sensores_pantalla(pantalla_IO_t *);
void fsm_imprimir_pantalla_temperatura(pantalla_IO_t *);
void fsm_imprimir_lectura_tecla(estado_imprimir_sensor_t *, estado_imprimir_sensor_t, pantalla_IO_t *);
void fsm_funcion_imprimir_pantalla(pantalla_IO_t *);
void fsm_maquina_control_sensores(pantalla_IO_t *);
void fsm_maquina_control_sensor_temperatura(pantalla_IO_t *);
void fsm_maquina_control_sensor_humedad_ambiente(pantalla_IO_t *);
void fsm_maquina_control_sensor_humedad_suelo(pantalla_IO_t *);
void fsm_funcion_control_sensores(pantalla_IO_t *);

#endif /* _INC_MAQUINA_ESTADOS_H_ */
