#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "smc.h"
#include "img.h"
#include "arm_math.h"

/** Chip select number to be set */
#define ILI93XX_LCD_CS      1

struct ili93xx_opt_t g_ili93xx_display_opt;

//! DAC channel used for test
#define DACC_CHANNEL        1 // (PB14)

#define ADC_POT_CHANNEL 5
//! DAC register base for test
#define DACC_BASE           DACC
//! DAC ID for test
#define DACC_ID             ID_DACC

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



/** Analog control value */
#define DACC_ANALOG_CONTROL (DACC_ACR_IBCTLCH0(0x02) \
| DACC_ACR_IBCTLCH1(0x02) \
| DACC_ACR_IBCTLDACCORE(0x01))

/** The analog voltage reference **/
#define VADREF    (float) 3.3
/** The maximal data value (no sign) */
#define MAX_DIGITAL   ((1 << DACC_RESOLUTION) - 1)
/** The maximal (peak-peak) amplitude value */
#define MAX_AMPLITUDE_ANAG (float) 5/6*VADREF
/** The minimal (peak-peak) amplitude value */
#define MIN_AMPLITUDE_ANAG (float) 1/6*VADREF

/**
 * global variables
 */

int namostra = 0 ;
float32_t ySeno;
int resolucao	;
float deltaTeta ;
float valorMedio;
int amplitude;
float rad;
int nleituraADC = 0;
int max_digital;

/**
*  Interrupt handler for TC0 interrupt.
*/
void ADC_Handler(void)
{
	uint32_t tmp;
	uint32_t status ;

	status = adc_get_status(ADC);
	max_digital = adc_get_channel_value(ADC, ADC_POT_CHANNEL);
}
	
void TC0_Handler(void){

  volatile uint32_t ul_dummy, status;
  uint32_t valorDAC = 1024;
  ul_dummy = tc_get_status(TC0,0);
  UNUSED(ul_dummy);
  

  /************************************************************************/
  /* ADC                                                                     */
  /************************************************************************/
  if(nleituraADC >= 500){
	 adc_start(ADC);
	 nleituraADC = 0;
   }
   nleituraADC++;

  /************************************************************************/
  /* Escreve um novo valor no DAC                                         */
  /************************************************************************/
  status = dacc_get_interrupt_status(DACC_BASE);

  /* namostra > 2*pi  ?? */
  if(namostra > resolucao)
	namostra = 0;

  ySeno = sin(deltaTeta*namostra)*((float)max_digital)/MAX_AMPLITUDE_ANAG;
  dacc_write_conversion_data(DACC_BASE, ySeno);

  namostra++;

 
}

static void configure_tc(int freq)
{
	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	
	// [main_tc_configure]
	/*
	* Aqui atualizamos o clock da cpu que foi configurado em sysclk init
	*
	* O valor atual est'a em : 120_000_000 Hz (120Mhz)
	*/
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	/****************************************************************
	* Ativa o clock do periférico TC 0
	*****************************************************************
	*
	* Parametros :
	*  1 - ID do periferico
	*
	*
	*****************************************************************/
	pmc_enable_periph_clk(ID_TC0);

	/*****************************************************************
	* Configura TC para operar no modo de comparação e trigger RC
	*****************************************************************
	*
	* Configura TC para operar no modo de comparação e trigger RC
	* devemos nos preocupar com o clock em que o TC irá operar !
	*
	* Cada TC possui 3 canais, escolher um para utilizar.
	*
	* No nosso caso :
	
	*
	*	MCK		= 120_000_000
	*	SLCK	= 32_768		(rtc)
	*
	* Uma opção para achar o valor do divisor é utilizar a funcao, como ela
	* funciona ?
	* tc_find_mck_divisor()
	*
	*
	* Parametros
	*   1 - TC a ser configurado (TC0,TC1, ...)
	*   2 - Canal a ser configurado (0,1,2)
	*   3 - Configurações do TC :
	*
	*	* Configurações de modo de operação :
	*	    TC_CMR_ABETRG  : TIOA or TIOB External Trigger Selection
	*	    TC_CMR_CPCTRG  : RC Compare Trigger Enable
	*	    TC_CMR_WAVE    : Waveform Mode
	*
	*     Configurações de clock :
	*	    TC_CMR_TCCLKS_TIMER_CLOCK1 : Clock selected: internal MCK/2 clock signal
	*	    TC_CMR_TCCLKS_TIMER_CLOCK2 : Clock selected: internal MCK/8 clock signal
	*	    TC_CMR_TCCLKS_TIMER_CLOCK3 : Clock selected: internal MCK/32 clock signal
	*	    TC_CMR_TCCLKS_TIMER_CLOCK4 : Clock selected: internal MCK/128 clock signal
	*	    TC_CMR_TCCLKS_TIMER_CLOCK5 : Clock selected: internal SLCK clock signal
	*
	*****************************************************************/
	tc_init(TC0, 0, TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK5);
	
	/*****************************************************************
	* Configura valor trigger RC
	*****************************************************************
	*
	* Aqui devemos configurar o valor do RC que vai trigar o reinicio da contagem
	* devemos levar em conta a frequência que queremos que o TC gere as interrupções
	* e tambem a frequencia com que o TC está operando.
	*
	* Devemos configurar o RC para o mesmo canal escolhido anteriormente.
	*
	*   ^
	*	|	Contador (incrementado na frequencia escolhida do clock)
	*   |
	*	|	 	Interrupcao
	*	|------#----------- RC
	*	|	  /
	*	|   /
	*	| /
	*	|-----------------> t
	*
	* Parametros :
	*   1 - TC a ser configurado (TC0,TC1, ...)
	*   2 - Canal a ser configurado (0,1,2)
	*   3 - Valor para trigger do contador (RC)
	*****************************************************************/
	tc_write_rc(TC0,0,32768/freq);
	
	/*****************************************************************
	* Configura interrupção no TC
	*****************************************************************
	* Parametros :
	*   1 - TC a ser configurado
	*   2 - Canal
	*   3 - Configurações das interrupções
	*
	*        Essas configurações estão definidas no head : tc.h
	*
	*	        TC_IER_COVFS : 	Counter Overflow
	*	        TC_IER_LOVRS : 	Load Overrun
	*	        TC_IER_CPAS  : 	RA Compare
	*	        TC_IER_CPBS  : 	RB Compare
	*	        TC_IER_CPCS  : 	RC Compare
	*	        TC_IER_LDRAS : 	RA Loading
	*	        TC_IER_LDRBS : 	RB Loading
	*	        TC_IER_ETRGS : 	External Trigger
	*****************************************************************/
	tc_enable_interrupt(TC0,0,TC_IER_CPCS);
	
	/*****************************************************************
	* Ativar interrupção no NVIC*/
	
	
	NVIC_EnableIRQ(ID_TC0);
	
	/*****************************************************************
	* Inicializa o timer
	*****************************************************************
	*
	* Parametros :
	*   1 - TC
	*   2 - Canal
	*****************************************************************/
	tc_start(TC0,0);


	/************************************************************************/
	/* Main Code
	*/
	/************************************************************************/
}

void configure_LCD(void){
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
};


void configure_adc(void)
{
	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, sysclk_get_cpu_hz(), 6400000, STARTUP_TIME);
	adc_configure_timing(ADC, TRACKING_TIME	, ADC_SETTLING_TIME_3, TRANSFER_PERIOD);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
	/* Enable chnnel for potentiometer. */
	adc_enable_channel(ADC, ADC_POT_CHANNEL);
	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);
	/* Start conversion. */
	adc_start(ADC);
	adc_enable_interrupt(ADC, ADC_ISR_EOC5);
}

void configure_dac(void){
  /************************************************************************/
  /* DAC                                                                  */
  /************************************************************************/
  
  /* Enable clock for DACC */
  sysclk_enable_peripheral_clock(DACC_ID);
  
  /* Reset DACC registers */
  dacc_reset(DACC_BASE);

  /* Half word transfer mode */
  dacc_set_transfer_mode(DACC_BASE, 0);

  /* selects channel */
  dacc_set_channel_selection(DACC_BASE, DACC_CHANNEL);

  /* Enable output channel DACC_CHANNEL */
  dacc_enable_channel(DACC_BASE, DACC_CHANNEL);

  /* Set up analog current */
  dacc_set_analog_control(DACC_BASE, DACC_ANALOG_CONTROL);
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

  uint16_t i ;

  configure_LCD();
  configure_dac();
  configure_tc(1000);
  configure_adc();

  resolucao	 = 100;
  deltaTeta  = 2.0*PI/((float)resolucao);
  valorMedio = MAX_AMPLITUDE_ANAG/2.0;


  ili93xx_draw_pixmap(0,
            ILI93XX_LCD_HEIGHT-100-1,
            240-1,
            100-1,
            &image_data_maua[0]);
  

  while (1) {
     
  }
}
