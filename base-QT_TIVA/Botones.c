//*****************************************************************************
//
// Codigo de partida comunicacion TIVA-QT (Marzo2015)
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//  Estructura de aplicación básica para el desarrollo de aplicaciones genericas
//  basada en la TIVA, en las que existe un intercambio de comandos con un interfaz
//  gráfico (GUI) Qt.
//  La aplicacion se basa en un intercambio de comandos con órdenes e información, a través  de la
//  configuración de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
//  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
//   En el ejemplo basico de partida se implementará la recepcion de un comando
//  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
//  comando enviado desde la placa al GUI, para mostrar el estado de los botones.
//  Implementacion del comando de ejemplo Botones
//*****************************************************************************

#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"

#include "drivers/rgb.h"
#include "usb_dev_serial.h"
#include "protocol.h"


#define LED1TASKPRIO 1
#define LED1TASKSTACKSIZE 128

//Globales

uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;

extern void vUARTTask( void *pvParameters );


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t ui8Count = 0;

	if (++ui8Count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		ui8Count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

SemaphoreHandle_t mutexUSB;

xQueueHandle cola_freertos;

static portTASK_FUNCTION( ButtonsTask, pvParameters ){
	uint8_t pui8Frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
	PARAM_COMANDO_BUTTONS parametro;
	int16_t i16Numdatos;
	int32_t i32Status;
	//
	// Loop forever.
	//
	while(1)
	{
		if (xQueueReceive(cola_freertos,&i32Status,portMAX_DELAY)==pdTRUE)
		{
			parametro.ui8Buttons=0;
			if((i32Status & LEFT_BUTTON) == 0)
				parametro.button.fLeft = 1;
			if((i32Status & RIGHT_BUTTON) == 0)
				parametro.button.fRight = 1;
			i16Numdatos=create_frame(pui8Frame,COMANDO_BUTTONS,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
			if (i16Numdatos>=0)
			{
				xSemaphoreTake(mutexUSB,portMAX_DELAY);
				send_frame(pui8Frame,i16Numdatos);
				xSemaphoreGive(mutexUSB);
			}
		}
	}
}



// El codigo de esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos del interprete a traves
// del terminal serie (puTTY)
//Aqui solo la declaramos para poderla referenciar en la funcion main
extern void vUARTTask( void *pvParameters );

// Codigo para procesar los comandos recibidos a traves del canal USB
static portTASK_FUNCTION( CommandProcessingTask, pvParameters ){


	uint8_t pui8Frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
	int16_t i16Numdatos;
	uint16_t ui16Errors=0;
	uint8_t ui8Command;

	/* The parameters are not used. */
	( void ) pvParameters;

	for(;;)
	{
		i16Numdatos=receive_frame(pui8Frame,MAX_FRAME_SIZE);
		if (i16Numdatos>0)
		{	//Si no hay error, proceso la trama que ha llegado.
			i16Numdatos=destuff_and_check_checksum(pui8Frame,i16Numdatos);
			if (i16Numdatos<0)
			{
				//Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
				ui16Errors++;
				// Procesamiento del error (TODO)
			}
			else
			{
				//El paquete esta bien, luego procedo a tratarlo.
				ui8Command=decode_command_type(pui8Frame,0);

				switch(ui8Command)
				{
				case COMANDO_PING :
					//A un comando de ping se responde con el propio comando
					i16Numdatos=create_frame(pui8Frame,ui8Command,0,0,MAX_FRAME_SIZE);
					if (i16Numdatos>=0)
					{
						xSemaphoreTake(mutexUSB,portMAX_DELAY);
						send_frame(pui8Frame,i16Numdatos);
						xSemaphoreGive(mutexUSB);
					}else{
						//Error de creacion de trama: determinar el error y abortar operacion
						ui16Errors++;
						// Procesamiento del error (TODO)
						// Esto de aqui abajo podria ir en una funcion "createFrameError(numdatos)  para evitar
						// tener que copiar y pegar todo en cada operacion de creacion de paquete
						switch(i16Numdatos){
						case PROT_ERROR_NOMEM:
							// Procesamiento del error NO MEMORY (TODO)
							break;
						case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
							// Procesamiento del error STUFFED_FRAME_TOO_LONG (TODO)
							break;
						case PROT_ERROR_COMMAND_TOO_LONG:
							// Procesamiento del error COMMAND TOO LONG (TODO)
							break;
						}
					}
					break;
				case COMANDO_LEDS:
				{
					PARAM_COMANDO_LEDS parametro;
					uint8_t ui8LED=0;
					uint32_t g_pui32Colors[3] = { 0x0000, 0x0000, 0x0000 };
					if (check_command_param_size(i16Numdatos,sizeof(parametro)))
					{
						extract_packet_command_param(pui8Frame,sizeof(parametro),&parametro);
						g_pui32Colors[0]= parametro.leds.fRed ? 0x8000 : 0x0000;
						if(g_pui32Colors[0]!=0)
							ui8LED=ui8LED+2;
						g_pui32Colors[1]=parametro.leds.fGreen ? 0x8000 : 0x0000;
						if(g_pui32Colors[1]!=0)
							ui8LED=ui8LED+8;
						g_pui32Colors[2]= parametro.leds.fBlue ? 0x8000 : 0x0000;
						if(g_pui32Colors[2]!=0)
							ui8LED=ui8LED+4;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ui8LED);

					}
				}
				break;
				case COMANDO_BRILLO:
				{
					PARAM_COMANDO_BRILLO parametro;
					float Intensity = 0.0f;
					if (check_command_param_size(i16Numdatos,sizeof(parametro)))
					{
						extract_packet_command_param(pui8Frame,sizeof(parametro),&parametro);
						Intensity= parametro.rIntensity;
						RGBIntensitySet(Intensity);
					}
				}
				break;
				case COMANDO_COLOR:
						{
							PARAM_COMANDO_COLOR parametro;

							uint32_t g_pui32Colors[3] = { 0x0000, 0x0000, 0x0000 }; /*lo comento ya
							 * que creo que para que los demas led no se vean afectados no es necesario
							 * resetearlos todos cada vez que se ajuste uno
							 */
							if (check_command_param_size(i16Numdatos,sizeof(parametro)))
							{
								extract_packet_command_param(pui8Frame,sizeof(parametro),&parametro);

								g_pui32Colors[0]=parametro.leds.fRed ? 0x8000 : 0x0000;
								g_pui32Colors[1]=parametro.leds.fGreen ? 0x8000 : 0x0000;
								g_pui32Colors[2]= parametro.leds.fBlue ? 0x8000 : 0x0000;

								RGBColorSet(g_pui32Colors);
							}
						}
						break;

				case COMANDO_CAMBIO_MODO:
						{
							PARAM_COMANDO_MODO parametro;


							if (check_command_param_size(i16Numdatos,sizeof(parametro)))
							{
								extract_packet_command_param(pui8Frame,sizeof(parametro),&parametro);

								if(parametro.modo==1){

									// Configura pines PF1, PF2, y PF3 como salidas (control de LEDs)
									GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
								}
								if(parametro.modo==0){
									RGBInit(1);
								}
							}
						}
						break;

		    	case COMANDO_SONDEO:
				{
					PARAM_COMANDO_SONDEO parametro;


					if (check_command_param_size(i16Numdatos,sizeof(parametro)))
					{
						extract_packet_command_param(pui8Frame,sizeof(parametro),&parametro);

						if(parametro.sondeo==1){
							GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
							uint8_t pui8Frame[MAX_FRAME_SIZE];
							PARAM_COMANDO_BUTTONS parametro1;
							int16_t i16Numdatos;
							int32_t i32Status = ROM_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);
							parametro1.ui8Buttons=0;
							if((i32Status & LEFT_BUTTON) == 0)
								parametro1.button.fLeft = 1;
							if((i32Status & RIGHT_BUTTON) == 0)
								parametro1.button.fRight = 1;
							i16Numdatos=create_frame(pui8Frame,COMANDO_BUTTONS,&parametro1,sizeof(parametro1),MAX_FRAME_SIZE);
								if (i16Numdatos>=0)
									{
									xSemaphoreTake(mutexUSB,portMAX_DELAY);
									send_frame(pui8Frame,i16Numdatos);
									xSemaphoreGive(mutexUSB);
								}
							GPIOIntDisable(GPIO_PORTF_BASE,ALL_BUTTONS);
						}

					  }
					}
				break;

		    	case COMANDO_MODO_SONDEO:
		    		{
		    			PARAM_COMANDO_SONDEO parametro;

		    			if (check_command_param_size(i16Numdatos,sizeof(parametro)))
		    			{
		    				extract_packet_command_param(pui8Frame,sizeof(parametro),&parametro);

		    				if(parametro.sondeo==0){//tarea de botones activada
		    					GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
		    				}
		    				if(parametro.sondeo==1){//tarea de actualizacion de botones constante desactivada
		    					GPIOIntDisable(GPIO_PORTF_BASE,ALL_BUTTONS);
		    				}

		    			  }
		    			}
		    		break;


				default:
				{
					PARAM_COMANDO_NO_IMPLEMENTADO parametro;
					parametro.command=ui8Command;
					//El comando esta bien pero no esta implementado
					i16Numdatos=create_frame(pui8Frame,COMANDO_NO_IMPLEMENTADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
					if (i16Numdatos>=0)
					{
						xSemaphoreTake(mutexUSB,portMAX_DELAY);
						send_frame(pui8Frame,i16Numdatos);
						xSemaphoreGive(mutexUSB);
					}
					break;
				}
				}// switch
			}
		}else{ // if (i16Numdatos >0)
			//Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
			ui16Errors++;
			// Procesamiento del error (TODO)
		}
	}
}

//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 50 MHz (200 Mhz del Pll dividido por 4)


	// Get the system clock speed.
	g_ui32SystemClock = SysCtlClockGet();



	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	ROM_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

	//
	// Inicializa la UARTy la configura a 115.200 bps, 8-N-1 .
	//se usa para mandar y recibir mensajes y comandos por el puerto serie
	// Mediante un programa terminal como gtkterm, putty, cutecom, etc...
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0,115200,SysCtlClockGet());

	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);	//La UART tiene que seguir funcionando aunque el micro este dormido
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);	//La UART tiene que seguir funcionando aunque el micro este dormido

	//Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1 (eliminar si no se usa finalmente)
	RGBInit(1);
	SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
	SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
	SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);	//Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo

	//Inicializa los botones (tambien en el puerto F) y habilita sus interrupciones
	ButtonsInit();
	ROM_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
	ROM_IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);// Misma prioridad que configMAX_SYSCALL_INTERRUPT_PRIORITY
	// Una prioridad menor (mayor numero) podria dar problemas si la interrupcion
	// ejecuta llamadas a funciones de FreeRTOS
	GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
	ROM_IntEnable(INT_GPIOF);

	//
	// Mensaje de bienvenida inicial.
	//
	UARTprintf("\n\nBienvenido a la aplicacion XXXXX (curso 2014/15)!\n");
	UARTprintf("\nAutores: XXXXXX y XXXXX ");

	// Cola para el envío del estado del puertoF (botones)
	cola_freertos=xQueueCreate(3,sizeof(int32_t));	//espacio para 3items de tamaño ulong
	if (NULL==cola_freertos)
		while(1);

	// Mutex para proteccion del canal USB
    mutexUSB=xSemaphoreCreateMutex();
    if(mutexUSB==NULL) // Error de creacion de semaforos
    	while(1);

	/**                                              Creacion de tareas 												**/

	// Crea la tarea que gestiona los comandos UART (definida en el fichero commands.c)
	//
	if((xTaskCreate(vUARTTask, "Uart", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

	UsbSerialInit(32,32);	//Inicializo el  sistema USB
	//
	// Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
	//
	if(xTaskCreate(CommandProcessingTask, "usbser",512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdTRUE)
	{
		while(1);
	}

	//
	// Crea la tarea que envia el estado de los botones
	//
	if(xTaskCreate(ButtonsTask, "Botones",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
		{
			while(1);
		}

	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

void GPIOFIntHandler(void)
{
	signed portBASE_TYPE higherPriorityTaskWoken=pdFALSE;	//Hay que inicializarlo a False!!
	int32_t i32Status = ROM_GPIOPinRead(GPIO_PORTF_BASE,ALL_BUTTONS);
	xQueueSendFromISR (cola_freertos,&i32Status,&higherPriorityTaskWoken);
	GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);              //limpiamos flags
	//Cesion de control de CPU si se ha despertado una tarea de mayor prioridad
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}