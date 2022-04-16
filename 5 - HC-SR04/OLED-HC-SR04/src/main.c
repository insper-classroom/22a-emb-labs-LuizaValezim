#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define time_min 58*1e-6
#define time_max 11764*1e-6
#define vel 340 
#define height_max 32
#define length_max 120

#define LED_PIO             PIOC   
#define LED1_PIO			PIOA
#define LED2_PIO			PIOC
#define LED3_PIO			PIOB

#define LED_PIO_ID			ID_PIOC
#define LED1_PIO_ID			ID_PIOA
#define LED2_PIO_ID			ID_PIOC
#define LED3_PIO_ID			ID_PIOB

#define LED_PIO_IDX			8	
#define LED1_PIO_IDX		0
#define LED2_PIO_IDX		30
#define LED3_PIO_IDX		2

#define LED_PIO_IDX_MASK    (1 << LED_PIO_IDX)
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)

#define BUT_PIO				PIOA
#define BUT1_PIO			PIOD
#define BUT2_PIO			PIOC
#define BUT3_PIO			PIOA

#define BUT_PIO_ID			ID_PIOA
#define BUT1_PIO_ID			ID_PIOD
#define BUT2_PIO_ID			ID_PIOC
#define BUT3_PIO_ID			ID_PIOA

#define BUT_PIO_IDX			11
#define BUT1_PIO_IDX		28
#define BUT2_PIO_IDX		31
#define BUT3_PIO_IDX		19

#define BUT_PIO_IDX_MASK (1 << BUT_PIO_IDX) 
#define BUT1_PIO_IDX_MASK  (1 << BUT1_PIO_IDX)
#define BUT2_PIO_IDX_MASK  (1 << BUT2_PIO_IDX)
#define BUT3_PIO_IDX_MASK  (1 << BUT3_PIO_IDX)

#define TRIG_PIO				PIOA 
#define ECHO_PIO				PIOA 

#define ECHO_PIO_ID			    ID_PIOA 
#define TRIG_PIO_ID				ID_PIOA 

#define ECHO_PIO_IDX			13 
#define TRIG_PIO_IDX			3

#define ECHO_PIO_IDX_MASK		(1 << ECHO_PIO_IDX)
#define TRIG_PIO_IDX_MASK		(1 << TRIG_PIO_IDX)

volatile char trig_flag = 0;
volatile char echo_flag_fall = 0;
volatile char echo_flag_rise = 0;
volatile int undefined_time_counter = 0;
volatile char begin_flag = 0;
volatile char flag_rtc_alarm ;
volatile double time_rtt;

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

void pin_toggle(Pio *pio, uint32_t mask);

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void TC1_Handler(void) {
	volatile uint32_t status = tc_get_status(TC0, 1);
	undefined_time_counter++;
}

void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {}
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) { flag_rtc_alarm = 1; }

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}


void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) { }

}

void trig_callback() {
	trig_flag = 1;
	begin_flag = 1;
}

void echo_callback() {
	if(!pio_get(ECHO_PIO,PIO_INPUT,ECHO_PIO_IDX_MASK)) {
		echo_flag_fall = 1 ;
		echo_flag_rise = 0;
	} 
	
	else if (pio_get(ECHO_PIO,PIO_INPUT,ECHO_PIO_IDX_MASK)) {
		echo_flag_rise = 1;
		echo_flag_fall = 0;	
	}
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
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

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
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

void io_init(void)
{
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED_PIO,LED_PIO_IDX_MASK);
	
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED1_PIO,LED1_PIO_IDX_MASK);
	
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED2_PIO,LED2_PIO_IDX_MASK);
	
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set(LED3_PIO,LED3_PIO_IDX_MASK);
	
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pio_configure(TRIG_PIO, PIO_OUTPUT_0, TRIG_PIO_IDX_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_handler_set(BUT1_PIO,BUT1_PIO_ID,BUT1_PIO_IDX_MASK,PIO_IT_FALL_EDGE,trig_callback);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 2);
	
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK, PIO_DEFAULT);
	pio_handler_set(ECHO_PIO,ECHO_PIO_ID,ECHO_PIO_IDX_MASK,PIO_IT_EDGE,echo_callback);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_IDX_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4); 
}

int calculate_graphic (float measure) {
	
	int bar_raw = 20 - ( (int) measure*height_max)/400;
	if (bar_raw < 0) { bar_raw = 4; }
	else {
		bar_raw = abs(bar_raw) + 2;
	}
	
	return bar_raw;
}

void draw_graphic(float measure[]) {
	
	int bar_raw_0 = calculate_graphic(measure[0]);
	int bar_raw_1 = calculate_graphic(measure[1]);
	int bar_raw_2 = calculate_graphic(measure[2]);
	int bar_raw_3 = calculate_graphic(measure[3]);
	
	gfx_mono_draw_string("_",  (int) 2* length_max/3 + 0*10,bar_raw_0,&sysfont);
	gfx_mono_draw_string("_",  (int) 2* length_max/3 + 1*10,bar_raw_1,&sysfont);
	gfx_mono_draw_string("_",  (int) 2* length_max/3 + 2*10,bar_raw_2,&sysfont);
	gfx_mono_draw_string("_",  (int) 2* length_max/3 + 3*10,bar_raw_3,&sysfont);
};

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	io_init();
	
	WDT->WDT_MR = WDT_MR_WDDIS;

	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Let´s measure", 0,16, &sysfont);
	
	
	char dist_str[64];
	int ticks = 0;
	float time = 0.0;
	char oled_print = 0;
	char undefined_time = 1;
  
	float measurements[] = {0,0,0,0};
	char i = 0;
  
	TC_init(TC0, ID_TC1,1,1);
  
	while(1) {
		if (begin_flag) {
		
			if (trig_flag) {	
				ticks = 0;
				pio_set(TRIG_PIO,TRIG_PIO_IDX_MASK);
				delay_us(10);
				pio_clear(TRIG_PIO,TRIG_PIO_IDX_MASK);
				trig_flag = 0;
			}  
			
			if (echo_flag_rise && !trig_flag) {
				RTT_init(1.0/(2*time_min),0,0);
				echo_flag_rise = 0;
				tc_start(TC0,1);
			} 
			
			if (echo_flag_fall && !trig_flag) {	
				ticks = rtt_read_timer_value(RTT); //ticks
				echo_flag_fall = 0;
				oled_print = 1;
				tc_stop(TC0,1);
			
			} 
			
			if (undefined_time_counter >= 3) {
				if (undefined_time) {
					gfx_mono_ssd1306_init();
					gfx_mono_draw_string("SENSOR ERROR", 0,16, &sysfont);
					undefined_time = 0;
				}
			}
			
			if (oled_print) {
				time = ticks *time_min;
				float dist_met = 100*(vel*time)/2; //cm
				gfx_mono_ssd1306_init();
				
				if (dist_met > 400) {
					gfx_mono_draw_string("OUT OF RANGE", 0,16, &sysfont);
				}
				
				else {
					if (i > 4) {i = 0;}
					sprintf(dist_str,"%0.1lf",dist_met);
					gfx_mono_draw_string("[cm]", 0,0, &sysfont);
					gfx_mono_draw_string(dist_str, 0,16, &sysfont);
					measurements[i] = dist_met;
					draw_graphic(measurements);
					i++;
				}
				
				oled_print = 0;
				undefined_time = 1;
				undefined_time_counter = 0;
			}
		}
			
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}