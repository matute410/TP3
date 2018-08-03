//#include "board.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "semphr.h"
//#include <stdlib.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "sapi.h"

#define TP3_4 (1)
#define TP3_5 (2)
#define TP3_6 (3)

#define TEST (TP3_6)


DEBUG_PRINT_ENABLE;
#define	vPrintString(str) debugPrintString(str)

//macros relacionados con el uso de interrupcion por software
#define mainSW_INTERRUPT_ID		(0)
#define mainTRIGGER_INTERRUPT()	NVIC_SetPendingIRQ(mainSW_INTERRUPT_ID)
#define mainCLEAR_INTERRUPT()	NVIC_ClearPendingIRQ(mainSW_INTERRUPT_ID)
#define mainSOFTWARE_INTERRUPT_PRIORITY	(5)
#define vSoftwareInterruptHandler (DAC_IRQHandler)

#define PERIODIC_TASK_DELAY_MS 500
#define PERIODIC_TASK_DELAY PERIODIC_TASK_DELAY_MS / portTICK_PERIOD_MS

//inicializacion de la interrupcion por software
static void prvSetupSoftwareInterrupt()
{
	NVIC_SetPriority(mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY);

	NVIC_EnableIRQ(mainSW_INTERRUPT_ID);
}

//inicializacion del hardware
static void prvSetupHardware(void)
{
	// Inicializo el hardware y el puerto serie
	SystemCoreClockUpdate();
	Board_Init();
	prvSetupSoftwareInterrupt();

	debugPrintConfigUart( UART_USB, 115200 );

	// inicializo los LED's de la placa en 1 para saber que esta vivo
	gpioWrite( LED1, ON );
	gpioWrite( LED2, ON );
	gpioWrite( LED3, ON );
}

#if (TEST == TP3_4)

const char *pcTextForMain = "\r\n Ejercicio 4\r\n";

//definicion de cola y semaforo
xSemaphoreHandle xBinarySemaphore;
xQueueHandle xQueue;

//callback de la interrupcion por software
void vSoftwareInterruptHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	vPrintString("Interrupción - Entrego el semáforo.\r\n");

	//produzco un semaforo en la interrupcion
    xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);

    //limpio la interrupcion
    mainCLEAR_INTERRUPT();

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

//tarea periodica que genera la interrupcion por software
static void vTaskPeriodic(void *pvParameters)
{
	TickType_t xLastWakeTime;

	while (1) {
		xLastWakeTime = xTaskGetTickCount();

		// Cambio el estado del Led, imprimo mensaje por serie y activo la interrupcion
		gpioToggle(LED1);

		vPrintString("Tarea 1 - Periodica -  Activo interrupcion\r\n");
		mainTRIGGER_INTERRUPT();
		vPrintString("Tarea 1 - Periodica -  Interrupción terminada\r\n");

		// Genero un delay considerando el momento de inicio de la tarea
		vTaskDelayUntil( &xLastWakeTime, PERIODIC_TASK_DELAY );
	}
}

// Tarea que toma el samoforo y agrega un valor a la cola
static void vTaskHandler(void *pvParameters)
{
	int sendValue = 0;
	char str[200];

	while (1) {
		//suspendo la tarea hasta que llega el semaforo
		xSemaphoreTake(xBinarySemaphore,portMAX_DELAY);

//		sprintf(str, "Tarea 2 - Handler - Semaforo recibido. Enviando dato = %d\r\n", sendValue);
//		vPrintString(str);
		vPrintString("Tarea 2 - Handler - Semaforo recibido. Enviando dato");

		//envio el valor actual a la cola
		xQueueSendToBack(xQueue, &sendValue, 0);

		// Parpadeo LED para indicar envio de dato a la cola
		gpioToggle(LED3);

		//actualizo el valor a mandar
		sendValue++;
		if(sendValue > 50){
			sendValue=0;
		}
    }
}

// Tarea que recibe de la cola
static void vTaskReceiver(void *pvParameters)
{
	int receivedValue;
	//char str[200];

	while (1) {
		//suspendo la tarea hasta que se reciba un valor en la cola
		xQueueReceive(xQueue,&receivedValue,portMAX_DELAY);

		// Parpadeo LED para indicar que recibio dato
		gpioToggle(LED2);

		vPrintString("Tarea 3 - Receiver - Dato recibido");

		// Por alguna razon, el Debug deja de funcionar cuando quiero imprimir los numeros, por eso dejo las lineas comentadas
//		sprintf(str, "Tarea 3 - Receiver - Dato recibido = %d\r\n", receivedValue);
//		vPrintString(str);
    }
}

int main(void)
{
	//inicializo hardware
	prvSetupHardware();

	// Imprimo inicializacion del programa
	vPrintString(pcTextForMain);

	//inicializo recursos y tareas
    xBinarySemaphore = xSemaphoreCreateBinary();
   	xQueue = xQueueCreate(10, sizeof(int));

   	// Creo las 3 tareas a utilizar
    xTaskCreate(vTaskPeriodic, (char *) "vTask1", configMINIMAL_STACK_SIZE, NULL,
           			(tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

    xTaskCreate(vTaskHandler, (char *) "vTask2", configMINIMAL_STACK_SIZE, NULL,
               			(tskIDLE_PRIORITY + 2UL), (xTaskHandle *) NULL);

    xTaskCreate(vTaskReceiver, (char *) "vTask3", configMINIMAL_STACK_SIZE, NULL,
                   			(tskIDLE_PRIORITY + 3UL), (xTaskHandle *) NULL);

    //inicio scheduler
	vTaskStartScheduler();

	for(;;);

	return 0;
}

#endif // TEST == TP4_4


#if (TEST == TP3_5)

const char *pcTextForMain = "\r\n Ejercicio 5\r\n";

// Declaro recursos
xSemaphoreHandle xBinarySemaphore;
xQueueHandle xIntegerQueue;

// Callback de interrupcion por software
void vSoftwareInterruptHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// Bloqueo hasta recibir un valor de la cola o que pase 1mS
	int recivedValue = -1;
	if(xQueueReceiveFromISR(xIntegerQueue,&recivedValue,&xHigherPriorityTaskWoken) == pdTRUE){
		vPrintString("Interrupcion - Recibido: \r\n");
		// Imprimir valor recibido
	} else {
		vPrintString("Interrupcion - Tiempo limite alcanzado.\r\n");
	}

	//limpio interrupcion
    mainCLEAR_INTERRUPT();

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

//tarea periodica que triggerea interrupcion
static void vTaskPeriodic(void *pvParameters)
{
	TickType_t xLastWakeTime;

	while (1) {

		//levanto tiempo actual
		xLastWakeTime = xTaskGetTickCount();

		//cambio led y trigereo interrupcion por software
		gpioToggle(LED1);
		vPrintString("Tarea 1 - periodica - Activando interrupcion\r\n");
		mainTRIGGER_INTERRUPT();
		vPrintString("Tarea 1 - periodica - interrupcion finalizada\r\n");

		//delay desde el momento de inicio de tarea
		vTaskDelayUntil( &xLastWakeTime, PERIODIC_TASK_DELAY );

	}
}

//tarea que toma el semaforo y genera un valor en la cola
static void vTaskHandler(void *pvParameters)
{
	int sendValue = 0;
	while (1) {
		//bloqueo hasta tomar el semaforo
		xSemaphoreTake(xBinarySemaphore,portMAX_DELAY);
		vPrintString("Tarea 2 - Handler - Semaforo recibido, enviando dato\r\n");
		// Imprimir valor enviado

		//envio en valor a la cola
		xQueueSendToBack(xIntegerQueue, &sendValue, 0);

		//actualizo el valor a mandar
		sendValue++;
		if(sendValue > 100){
			sendValue=0;
		}
    }
}

//tarea que genera el semaforo cuando la cola esta vacia
static void vTaskReceiver(void *pvParameters)
{
	while (1) {
	    vTaskDelay(100 / portTICK_PERIOD_MS);
	    //reviso si la cola esta vacia
	    if(uxQueueMessagesWaiting(xIntegerQueue)==0){
			vPrintString("Tarea 3 - Receiver - Devuelve semaforo\r\n");
			xSemaphoreGive(xBinarySemaphore);
	    }
    }
}

int main(void)
{
	//inicializo el hardware
	prvSetupHardware();

	// Imprimo inicializacion del programa
	vPrintString(pcTextForMain);

   	//inicializo recursos y tareas
    xBinarySemaphore = xSemaphoreCreateBinary();
   	xIntegerQueue = xQueueCreate(10, sizeof(int));

    xTaskCreate(vTaskPeriodic, (char *) "vTask1", configMINIMAL_STACK_SIZE, NULL,
           			(tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

    xTaskCreate(vTaskHandler, (char *) "vTask2", configMINIMAL_STACK_SIZE, NULL,
               			(tskIDLE_PRIORITY + 2UL), (xTaskHandle *) NULL);

    xTaskCreate(vTaskReceiver, (char *) "vTask3", configMINIMAL_STACK_SIZE, NULL,
                   			(tskIDLE_PRIORITY + 3UL), (xTaskHandle *) NULL);

    //inicio scheduler
	vTaskStartScheduler();

	while (1);

	return ((int) NULL);
}

#endif // TEST == TP4_5

#if (TEST == TP3_6)

#define LED_SEQUENCE_LENGTH 4

//defino mutex
xSemaphoreHandle xMutex;

//tarea que recibe como argumento la secuencia de leds
static void vTaskLed(void *pvParameters)
{
	TickType_t xLastWakeTime;
	//casteo el parametro
	bool *sequence = (bool *)pvParameters;
	char nombre[100];

	while (1) {

		size_t i = 0;

		//intento tomar el mutex que representa el uso del recurso led
		xSemaphoreTake(xMutex, portMAX_DELAY);
		//sprintf("empieza secuencia de tarea %s\r\n",pcTaskGetTaskName(xTaskGetCurrentTaskHandle()));
		vPrintString("empieza secuencia de tarea ");
		vPrintString(pcTaskGetTaskName(xTaskGetCurrentTaskHandle()));
		vPrintString("\r\n");

		//itero sobre la secuencia
		for (i = 0; i < LED_SEQUENCE_LENGTH; ++i) {
			xLastWakeTime = xTaskGetTickCount();
			if(sequence[i]){
				//si la secuencia tiene un uno prendo el primer led y apago el segundo
				gpioWrite( LED1, ON );
				gpioWrite( LED2, OFF);
			}else{
				//si la secuencia tiene un uno prendo el segundo led y apago el primero
				gpioWrite( LED2, ON );
				gpioWrite( LED3, OFF);
			}
			//delay para poder ver la secuencia
			vTaskDelayUntil( &xLastWakeTime, PERIODIC_TASK_DELAY );
		}
		//cuando termino devuelvo el semaforo
		xSemaphoreGive(xMutex);
		//genero un delay para dar tiempo a las otras tareas
		vTaskDelay(PERIODIC_TASK_DELAY*10);
	}
}

int main(void)
{
	//inicializo hardware
	prvSetupHardware();

	//inicializo mutex
   	xMutex = xSemaphoreCreateMutex();

   	//defino las secuencias de cada tarea
   	bool sequenceTask1[] = {true,true,true,true};
   	bool sequenceTask2[] = {false,false,false,false};
   	bool sequenceTask3[] = {false,true,false,true};

   	//creo las tareas
    xTaskCreate(vTaskLed, (char *) "vTaskLed1", configMINIMAL_STACK_SIZE, sequenceTask1,
           				(tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

    xTaskCreate(vTaskLed, (char *) "vTaskLed2", configMINIMAL_STACK_SIZE, sequenceTask2,
               			(tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

    xTaskCreate(vTaskLed, (char *) "vTaskLed3", configMINIMAL_STACK_SIZE, sequenceTask3,
						(tskIDLE_PRIORITY + 1UL), (xTaskHandle *) NULL);

    //inicio el scheduler
	vTaskStartScheduler();

	while (1);

	return ((int) NULL);
}



#endif // TEST == TP3_6
