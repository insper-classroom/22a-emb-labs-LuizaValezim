// Laboratório 3 - Computação Embarcada
// Aluno: Luiza Valezim Augusto Pinto
// Data: 09/03/2022

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

#define BUT2_PIO          PIOC
#define BUT2_PIO_ID       ID_PIOC
#define BUT2_PIO_IDX      31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO          PIOA
#define BUT3_PIO_ID       ID_PIOA
#define BUT3_PIO_IDX      19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

volatile char BUT1_flag = 0;
volatile char BUT2_flag = 0;
volatile char BUT3_flag = 0;

void pisca_led(int n, int t);
void io_init(void);
void BUT1_callback(void);
void BUT2_callback(void);
void BUT3_callback(void);


/************************************************************************/
/* funções                                                              */
/************************************************************************/

void BUT1_callback(void)
{
	if (!pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		BUT1_flag = 1;
	}
	else {
		BUT1_flag = 0;
		// PINO == 0 --> Borda de descida
	}
}

void BUT2_callback(void)
{
	if (!pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		BUT2_flag = 1;
		} 
	else {
		BUT2_flag = 0;
		// PINO == 0 --> Borda de descida
	}
}

void BUT3_callback(void)
{
	if (!pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		BUT3_flag = 1;
	}
	else {
		BUT3_flag = 0;
		// PINO == 0 --> Borda de descida
	}
}

void pisca_led(int n, int t){
	
	int progresso = 10;
	
	if (BUT2_flag) {
		for (int i = 0; i < n; i++)	{
			pio_clear(LED_PIO, LED_IDX_MASK);
			delay_ms(t);
			pio_set(LED_PIO, LED_IDX_MASK);
			delay_ms(t);
			
			if (BUT1_flag) {
				t = t - 10;
				char str[4];
				sprintf(str, "%d", t);
				gfx_mono_draw_string(str, 80, 16, &sysfont);
			}
		
			if (BUT3_flag) {
				t = t + 10;
				char str[4];
				sprintf(str, "%d", t);
				gfx_mono_draw_string(str, 80, 16, &sysfont);
			}
			
			gfx_mono_draw_string("|", progresso, 0, &sysfont);
			progresso += 2;
			
			if (BUT2_flag) {
				pio_set(LED_PIO, LED_IDX_MASK);
				break;
			}
		}
		
		BUT2_flag = 0;
	}
	
	else {
		pio_set(LED_PIO, LED_IDX_MASK);
	}
}


// Inicializa botao SW0 do kit com interrupcao
void io_init(void)
{
	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: BUT2_callback()
	pio_handler_set(BUT1_PIO,
					BUT1_PIO_ID,
					BUT1_PIO_IDX_MASK,
					PIO_IT_EDGE,
					BUT1_callback);
	
	pio_handler_set(BUT2_PIO,
					BUT2_PIO_ID,
					BUT2_PIO_IDX_MASK,
					PIO_IT_EDGE,
					BUT2_callback);
					
	pio_handler_set(BUT3_PIO,
					BUT3_PIO_ID,
					BUT3_PIO_IDX_MASK,
					PIO_IT_EDGE,
					BUT3_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 5); // Prioridade 4
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 5); // Prioridade 4
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 5); // Prioridade 4
}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	io_init();
	
	// Init OLED
	gfx_mono_ssd1306_init();
  
	// Escreve na tela um circulo e um texto
	// gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	gfx_mono_draw_string("Delay", 20,16, &sysfont);
	
	gfx_mono_draw_string("|", 5, 0, &sysfont);
	gfx_mono_draw_string("|", 123, 0, &sysfont);
	
	
	int t = 200;
	char str[4];
	sprintf(str, "%d", t);
	gfx_mono_draw_string(str, 80, 16, &sysfont);

	/* Insert application code here, after the board has been initialized. */
	while(1) {
		pisca_led(55, t);
	}
}