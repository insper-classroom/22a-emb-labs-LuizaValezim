#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define BUTTON1_PIO PIOD
#define BUTTON1_PIO_ID ID_PIOD
#define BUTTON1_IDX 28
#define BUTTON1_IDX_MASK (1u << BUTTON1_IDX)

#define BUTTON2_PIO PIOC
#define BUTTON2_PIO_ID ID_PIOC
#define BUTTON2_IDX 31
#define BUTTON2_IDX_MASK (1u << BUTTON2_IDX)

#define LED1_PIO           PIOA
#define LED1_PIO_ID        ID_PIOA
#define LED1_PIO_IDX       0
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)

#define LED2_PIO           PIOC
#define LED2_PIO_ID        ID_PIOC
#define LED2_PIO_IDX       30
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)

#define LED3_PIO           PIOB
#define LED3_PIO_ID        ID_PIOB
#define LED3_PIO_IDX       2
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);


volatile int cont = 0;

/**
* @Brief Inverte o valor do pino 0->1/ 1->0
*/
void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

/**
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado) {
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, estado, 0, 0);
};

void io_init(void){
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_1, LED2_PIO_IDX_MASK, PIO_DEFAULT);
}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
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


/**
*  Interrupt handler for TC1 interrupt.
*/
void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);  
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC6_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC2, 0);
	
	cont += 1;	  
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC3_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 0);
	
	cont += 1;	  
}

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);    // BLINK Led
	}
}

void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		// o código para irq de segundo vem aqui
	}
	
// 	/* Time or date alarm */
// 	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
// 		// o código para irq de alame vem aqui
// 		flag_rtc_alarm = 1;
// 	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}


void init(){
	
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(BUTTON1_PIO_ID);
	
	pio_configure(BUTTON1_PIO, PIO_INPUT, BUTTON1_IDX_MASK, PIO_PULLUP);
	
	pio_enable_interrupt(BUTTON1_PIO, BUTTON1_IDX_MASK);
	pio_get_interrupt_status(BUTTON1_PIO);
	
	NVIC_EnableIRQ(BUTTON1_PIO_ID);
	NVIC_SetPriority(BUTTON1_PIO_ID, 4);
}

int main (void)
{	 char time[20];
	init();
	
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	io_init();
	gfx_mono_ssd1306_init();

	/* Configura Leds *
	LED_init(0);

	/** Configura timer TC0, canal 1 */
	TC_init(TC0, ID_TC1, 1, 4);
	tc_start(TC0, 1);
	
	RTT_init(0.25, 0, RTT_MR_RTTINCIEN);   
	
	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	
	/* Leitura do valor atual do RTC */
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	
	// RTC
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);
	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
	
	sprintf(time, "%d:%d:%d  ",current_hour,current_min,current_sec);
	gfx_mono_draw_string(time, 25,16, &sysfont);
	
	while (1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		if(!pio_get(BUTTON1_PIO, PIO_INPUT, BUTTON1_IDX_MASK)) {
			TC_init(TC2, ID_TC6, 0, 1);
			tc_start(TC2, 0);
		}
		
		if (cont >= 20) {
			pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
			delay_ms(500);
			pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
			
			cont = 0;
			tc_stop(TC2,0);
		}
		
		
// 		TC_init(TC1, ID_TC3, 0, 1);
// 		tc_start(TC1, 0);
		
		rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
 		rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
 		
 		sprintf(time, "%d:%d:%d  ",current_hour,current_min,current_sec);
 		gfx_mono_draw_string(time, 25,16, &sysfont);
	}
}
