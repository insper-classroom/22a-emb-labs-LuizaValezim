// Laboratório 5 - Computação Embarcada
// Aluno: Luiza Valezim Augusto Pinto
// Data: 23/03/2022

#include <asf.h>
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LED
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Botõess
#define BUT1_PIO          PIOD
#define BUT1_PIO_ID       ID_PIOD
#define BUT1_PIO_IDX      28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

// Echo e Trigger
#define ECHO_PIO     PIOD
#define ECHO_PIO_ID  ID_PIOC
#define ECHO_IDX     30
#define ECHO_IDX_MASK (1 << ECHO_IDX)

#define TRIGGER_PIO     PIOA
#define TRIGGER_PIO_ID  ID_PIOA
#define TRIGGER_IDX     6
#define TRIGGER_IDX_MASK (1 << TRIGGER_IDX)

volatile char measure_flag = 0;
volatile double dist_min = 0.02;      // [m]
volatile double dist_max = 4.00;      // [m]
volatile double time_min = 0.000058;  // [s]
volatile double time_max = 0.011764;  // [s]

void io_init(void);
void BUT_callback(void);
void trigger(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* funções                                                              */
/************************************************************************/

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void BUT_callback(void)
{
	if (!pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		measure_flag = 1;
	}
	else {
		measure_flag = 0;
		// PINO == 0 --> Borda de descida
	}
}

// Inicializa botao SW0 do kit com interrupcao
void io_init(void)
{
	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_1, LED_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT1_PIO_ID);
	
	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: BUT2_callback()
	pio_handler_set(BUT1_PIO,
					BUT1_PIO_ID,
					BUT1_PIO_IDX_MASK,
					PIO_IT_EDGE,
					BUT_callback);
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 5); // Prioridade 4
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		pin_toggle(LED_PIO, LED_PIO_IDX_MASK);    // BLINK Led
	}
}

void trigger(void) {
	pio_set(TRIGGER_PIO, TRIGGER_IDX_MASK);
	delay_us(10);
	pio_clear(TRIGGER_PIO, TRIGGER_IDX_MASK);
	delay_us(10);
}

int main (void) {
	board_init();
	sysclk_init();
	delay_init();
	io_init();
	
	// Init OLED
	gfx_mono_ssd1306_init();
	
	// sprintf(str, "%d", t);
	// gfx_mono_draw_string(str, 80, 16, &sysfont);

	while(1) {
		if (measure_flag) {
			gfx_mono_draw_string("teste", 0, 0, &sysfont);
			RTT_init(0.25, 0, RTT_MR_RTTINCIEN);  
		}
	}
}