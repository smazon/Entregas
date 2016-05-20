#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "smc.h"

/************************************************************************/
/* ADC                                                                     */
/************************************************************************/

/** Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE     (100)
/** Reference voltage for ADC,in mv. */
#define VOLT_REF        (3300)
/* Tracking Time*/
#define TRACKING_TIME    1
/* Transfer Period */
#define TRANSFER_PERIOD  1
/* Startup Time*/
#define STARTUP_TIME ADC_STARTUP_TIME_4

/** The maximal digital value */
#define MAX_DIGITAL     (4095)

/* Redefinir isso */
#define ADC_POT_CHANNEL 1

/************************************************************************/
/* LCD                                                                  */
/************************************************************************/
/** Chip select number to be set */
#define ILI93XX_LCD_CS      1

struct ili93xx_opt_t g_ili93xx_display_opt;

/************************************************************************/
/* prototype                                                            */
/************************************************************************/

void configure_LCD();
void configure_ADC();

/************************************************************************/
/* Configs                                                              */
/************************************************************************/
void configure_LCD(){
	/** Enable peripheral clock */
	pmc_enable_periph_clk(ID_SMC);

	/** Configure SMC interface for Lcd */
	smc_set_setup_timing(SMC, ILI93XX_LCD_CS, SMC_SETUP_NWE_SETUP(2)
	| SMC_SETUP_NCS_WR_SETUP(2)
	| SMC_SETUP_NRD_SETUP(2)
	| SMC_SETUP_NCS_RD_SETUP(2));
	smc_set_pulse_timing(SMC, ILI93XX_LCD_CS, SMC_PULSE_NWE_PULSE(4)
	| SMC_PULSE_NCS_WR_PULSE(4)
	| SMC_PULSE_NRD_PULSE(10)
	| SMC_PULSE_NCS_RD_PULSE(10));
	smc_set_cycle_timing(SMC, ILI93XX_LCD_CS, SMC_CYCLE_NWE_CYCLE(10)
	| SMC_CYCLE_NRD_CYCLE(22));
	#if ((!defined(SAM4S)) && (!defined(SAM4E)))
	smc_set_mode(SMC, ILI93XX_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE
	| SMC_MODE_DBW_8_BIT);
	#else
	smc_set_mode(SMC, ILI93XX_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE);
	#endif
	/** Initialize display parameter */
	g_ili93xx_display_opt.ul_width = ILI93XX_LCD_WIDTH;
	g_ili93xx_display_opt.ul_height = ILI93XX_LCD_HEIGHT;
	g_ili93xx_display_opt.foreground_color = COLOR_BLACK;
	g_ili93xx_display_opt.background_color = COLOR_WHITE;

	/** Switch off backlight */
	aat31xx_disable_backlight();

	/** Initialize LCD */
	ili93xx_init(&g_ili93xx_display_opt);

	/** Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(0, 0, ILI93XX_LCD_WIDTH,
	ILI93XX_LCD_HEIGHT);
	/** Turn on LCD */
	ili93xx_display_on();
	ili93xx_set_cursor_position(0, 0);
}

void configure_ADC(void){
	
	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_ADC);
	
	/* Initialize ADC. */
	/*
	 * Formula: ADCClock = MCK / ( (PRESCAL+1) * 2 )
	 * For example, MCK = 64MHZ, PRESCAL = 4, then:
	 * ADCClock = 64 / ((4+1) * 2) = 6.4MHz;
	 */
	/* Formula:
	 *     Startup  Time = startup value / ADCClock
	 *     Startup time = 64 / 6.4MHz = 10 us
	 */
	adc_init(ADC, sysclk_get_cpu_hz(), 6400000, STARTUP_TIME);
	
	/* Formula:
	 *     Transfer Time = (TRANSFER * 2 + 3) / ADCClock
	 *     Tracking Time = (TRACKTIM + 1) / ADCClock
	 *     Settling Time = settling value / ADCClock
	 *
	 *     Transfer Time = (1 * 2 + 3) / 6.4MHz = 781 ns
	 *     Tracking Time = (1 + 1) / 6.4MHz = 312 ns
	 *     Settling Time = 3 / 6.4MHz = 469 ns
	 */
	adc_configure_timing(ADC, TRACKING_TIME	, ADC_SETTLING_TIME_3, TRANSFER_PERIOD);

	/*
	* Configura trigger por software
	*/ 
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);

	/*
	* Checa se configuração 
	*/
	adc_check(ADC, sysclk_get_cpu_hz());

	/* Enable channel for potentiometer. */
	adc_enable_channel(ADC, ADC_TEMPERATURE_SENSOR);

	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);

	/* Start conversion. */
	adc_start(ADC);

	/* Enable PDC channel interrupt. */
	adc_enable_interrupt(ADC, ADC_ISR_RXBUFF);
}


/************************************************************************/
/* Interruptions                                                        */
/************************************************************************/

/**
 * \brief Timmer handler (100ms) starts a new conversion.
 */
void TC0_Handler(void)
{
	if (adc_get_status(ADC) & (1 << ADC_POT_CHANNEL)) {
		adc_start(ADC);
	}
}

/**
 * \brief ADC interrupt handler.
 * Entramos aqui quando a conversao for concluida.
 */
void ADC_Handler(void)
{
	uint32_t ul_counter;
	int32_t l_vol;
	float f_temp;
	uint32_t ul_value = 0;
	uint32_t ul_temp_value = 0;

	if ((adc_get_status(ADC) & ADC_ISR_RXBUFF) == ADC_ISR_RXBUFF) {

		adc_get_channel_value(ADC, ADC_POT_CHANNEL);
	}
}


/**
 * \brief Application entry point for smc_lcd example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	sysclk_init();
	board_init();

	configure_LCD();
	configure_ADC();

	
	while (1) {
	}
}

