#include "board.h"
#include "sapi.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "sapi_char_lcd.h"

#include "main.h"
#include "sensores.h"
#include "utils.h"
#include "maquina_estados.h"

static void vPrincipalTask(void * pvParameters);

sensor_t temp;
sensor_t hum_amb;
sensor_t hum_suelo;


int main( void )
{
	char * nombre_temperatura = TEXTO_SENSOR_TEMPERATURA;
	char * nombre_humedad_ambiente = TEXTO_SENSOR_HUMEDAD_AMBIENTE;
	char * nombre_humedad_suelo = TEXTO_SENSOR_HUMEDAD_SUELO;

	initHardware();

	inicializar_sensor(&temp, nombre_temperatura, UMBRAL_TEMPERATURA, (leer_sensor_t) leer_temperatura, (escribir_sensor_t) escribir_temperatura, (funcion_control_t)funcion_control_temperatura);
	inicializar_sensor(&hum_amb, nombre_humedad_ambiente, UMBRAL_HUMEDAD_AMBIENTE, (leer_sensor_t) leer_humedad, (escribir_sensor_t) escribir_humedad, (funcion_control_t)funcion_control_humedad_ambiente);
	inicializar_sensor(&hum_suelo, nombre_humedad_suelo, UMBRAL_HUMEDAD_SUELO, (leer_sensor_t) leer_humedad_suelo, (escribir_sensor_t) escribir_humedad_suelo, (funcion_control_t)funcion_control_humedad_suelo);

    xTaskCreate(vPrincipalTask, "programaPpal", 240, NULL, 1, NULL);

    vTaskStartScheduler();

	for(;;)

	return 0;
}


static void vPrincipalTask(void * pvParameters)
{
	TickType_t xLastWakeTime;

	CHAR_LCD_I2C_DECL( lcd, 16, 2, 0x3F );
	charLcdInit(lcd);
	charLcdClear(lcd);
    charLcdGoToXY(lcd, 1, 1);
    vTaskDelay(50/portTICK_RATE_MS);

    gpioWrite(LEDG, 1);

    pantalla_IO_t pantalla;
    pantalla.sensores[SENSOR_TEMPERATURA] = temp;
    pantalla.sensores[SENSOR_HUMEDAD_AMBIENTE] = hum_amb;
    pantalla.sensores[SENSOR_HUMEDAD_SUELO] = hum_suelo;
    pantalla.dato_nuevo = FALSE;
    pantalla.lcd = lcd;
    pantalla.contador_lectura_sensores = 0;

    const TickType_t xDelay100ms = 100UL/portTICK_RATE_MS;
    xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		fsm_maquina_sensores_leer_sensores(&pantalla);  // TODO armar estructura / arreglo con los sensores (usar la cola de freertos)
		fsm_maquina_pantalla_control_pantalla(&pantalla);
		fsm_imprimir_sensores_pantalla(&pantalla);
		fsm_maquina_control_sensores(&pantalla);

		fsm_funcion_imprimir_pantalla(&pantalla);
		fsm_funcion_control_sensores(&pantalla);

		vTaskDelayUntil(&xLastWakeTime, xDelay100ms);
	}
}
