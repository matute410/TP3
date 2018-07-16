/*
 * sensores.h
 *
 *  Created on: 12 de jun. de 2018
 *      Author: Matute
 */

#ifndef SENSORES_H
#define SENSORES_H

#include "sapi_char_lcd.h"
#include "sapi_datatypes.h"


#define TEXTO_SENSOR_TEMPERATURA "Temperatura"
#define TEXTO_SENSOR_HUMEDAD_AMBIENTE "Humedad amb."
#define TEXTO_SENSOR_HUMEDAD_SUELO "Humedad suelo"

#define UMBRAL_TEMPERATURA 25
#define UMBRAL_HUMEDAD_AMBIENTE 80
#define UMBRAL_HUMEDAD_SUELO 50

#define SENSOR_TEMPERATURA 0
#define SENSOR_HUMEDAD_AMBIENTE 1
#define SENSOR_HUMEDAD_SUELO 2

typedef float tipo_temperatura_t;
typedef int tipo_humedad_ambiente_t;
typedef uint16_t tipo_humedad_suelo_t;

typedef bool_t (*leer_sensor_t)(void *);
typedef void (*escribir_sensor_t)(void *, charLcd_t *);
typedef void (*funcion_control_t)(void*, charLcd_t *);

typedef struct sensor_st
{
    char *nombre;
    void *medicion;
    int umbral;
    leer_sensor_t leer_sensor;
    escribir_sensor_t escribir_sensor;
    funcion_control_t funcion_control;
}sensor_t;


// Prototipos

void inicializar_sensor(sensor_t *, char *, int, leer_sensor_t, escribir_sensor_t, funcion_control_t);
bool_t leer_temperatura(sensor_t *);
void escribir_temperatura(sensor_t *, charLcd_t *);
void funcion_control_temperatura(sensor_t *, charLcd_t*);
bool_t leer_humedad(sensor_t *);
void escribir_humedad(sensor_t *, charLcd_t *);
void funcion_control_humedad_ambiente(sensor_t *, charLcd_t*);
bool_t leer_humedad_suelo(sensor_t *);
void escribir_humedad_suelo(sensor_t *, charLcd_t *);
void funcion_control_humedad_suelo(sensor_t *, charLcd_t*);


#endif //SENSORES_H
