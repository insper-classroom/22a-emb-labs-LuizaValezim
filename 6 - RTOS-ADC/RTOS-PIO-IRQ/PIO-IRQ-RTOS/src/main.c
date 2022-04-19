#include "conf_board.h"
#include <asf.h>

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/

#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

// Configuracoes do BOTAO 1 - OLED
#define BUT_PIO1			  PIOD
#define BUT_PIO1_ID        ID_PIOD
#define BUT_PIO1_IDX	      28
#define BUT_PIO1_IDX_MASK (1u << BUT_PIO1_IDX)

#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_PIO_IDX 8
#define LED_IDX_MASK (1 << LED_PIO_IDX)

#define USART_COM_ID ID_USART1
#define USART_COM USART1

/************************************************************************/
/* RTOS                                                                */
/************************************************************************/

#define TASK_LED_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_BUT_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_BUT_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Semaforo a ser usado pela task led */
SemaphoreHandle_t xSemaphoreBut;
SemaphoreHandle_t xSemaphoreBut1;

/** Queue for msg log send data */
QueueHandle_t xQueueLedFreq;
QueueHandle_t xQueueBut;

//Variaveis

int delay = 2000;

/************************************************************************/
/* prototypes local                                                     */
/************************************************************************/

void but_callback(void);
void but1_oled_callback(void); // -- Botão 1 da placa oled
static void BUT_init(void);
static void BUT1_OLED_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
static void USART1_init(void);
void LED_init(int estado);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
* \brief Called if stack overflow during execution
*/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/**
* \brief This function is called by FreeRTOS idle task
*/
extern void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }

/**
* \brief This function is called by FreeRTOS each tick
*/
extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBut, &xHigherPriorityTaskWoken);
	
	int delay = 100;
	xQueueSendFromISR(xQueueBut, (void *)&delay,  &xHigherPriorityTaskWoken);
	

	
}

void but1_oled_callback(void) {
	printf("Entrou no callback oled but1");
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreBut, &xHigherPriorityTaskWoken);
	int delay = -100;
	xQueueSendFromISR(xQueueBut, (void *)&delay,  &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_led(void *pvParameters) {

	LED_init(1);

	uint32_t msg = 0;
	uint32_t delayMs = 2000;

	/* tarefas de um RTOS não devem retornar */
	for (;;) {
		/* verifica se chegou algum dado na queue, e espera por 0 ticks */
		if (xQueueReceive(xQueueLedFreq, &msg, (TickType_t)0)) {
			/* chegou novo valor, atualiza delay ! */
			/* aqui eu poderia verificar se msg faz sentido (se esta no range certo)
			*/
			/* converte ms -> ticks */
			delayMs = msg / portTICK_PERIOD_MS;
			printf("delay = %d \n", delayMs);
		}

		/* pisca LED */
		pin_toggle(LED_PIO, LED_IDX_MASK);

		/* suspende por delayMs */
		vTaskDelay(delayMs);
	}
}

static void task_but(void *pvParameters) {

	/* iniciliza botao */
	BUT_init();
	BUT1_OLED_init();

	int32_t delayTicks = 0;
	int delay = 1000;

	for (;;) {
		/* aguarda por tempo inderteminado até a liberacao do semaforo */
		if (xQueueReceive(xQueueBut, &delayTicks, (TickType_t)0)) {

			delay = delay + delayTicks;
			/* envia nova frequencia para a task_led */
			xQueueSend(xQueueLedFreq, (void *)&delayTicks, 10);
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if (pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio, mask);
}

void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_IDX_MASK, estado, 0, 0);
};


static void BUT_init(void) {
	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK,
	PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE,
	but_callback);

	pio_get_interrupt_status(BUT_PIO);
	
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

}

static void BUT1_OLED_init(void) {
	/* conf botão como entrada */
	pio_configure(BUT_PIO1, PIO_INPUT, BUT_PIO1_IDX_MASK,
	PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO1, BUT_PIO1_IDX_MASK, 60);
	pio_enable_interrupt(BUT_PIO1, BUT_PIO1_IDX_MASK);
	pio_handler_set(BUT_PIO1, BUT_PIO1_ID, BUT_PIO1_IDX_MASK, PIO_IT_FALL_EDGE,
	but1_oled_callback);

	pio_get_interrupt_status(BUT_PIO1);
	
	/* configura prioridade */
	NVIC_EnableIRQ(BUT_PIO1_ID);
	NVIC_SetPriority(BUT_PIO1_ID, 4);

}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	configure_console();

	/* Attempt to create a semaphore. */
	xSemaphoreBut = xSemaphoreCreateBinary();
	if (xSemaphoreBut == NULL)
	printf("falha em criar o semaforo do botão da placa \n");
	
	xSemaphoreBut1 = xSemaphoreCreateBinary();
	if (xSemaphoreBut1 == NULL)
	printf("falha em criar o semaforo do botão 1 da placa \n");

	/* cria queue com 32 "espacos" */
	/* cada espaço possui o tamanho de um inteiro*/
	xQueueLedFreq = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueLedFreq == NULL)
	printf("falha em criar a queue do led\n");
	
	xQueueBut = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueBut == NULL)
	printf("falha em criar a queue do botão\n");

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Create task to monitor processor activity */
	if (xTaskCreate(task_but, "BUT", TASK_BUT_STACK_SIZE, NULL,
	TASK_BUT_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create UartTx task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while (1) {
	}

	return 0;
}