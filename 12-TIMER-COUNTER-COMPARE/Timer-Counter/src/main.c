/************************************************************************
* Timer Counter
*
* Exemplo de utilização do modo RC Compare
* Nesse exemplo o uC entra em modo de stand-by e aguarda por três diferentes
* interrupcoes :
*
* 1 - Timer
*	Muda o estado do led em uma frequencia pré definida (inicial 4Hz)
*
* 2 - Botao 1
*	Altera a frequencia do led em 10% para cima
*	
* 3 - Botao 2
*	Altera a frequencia do led em 10% para baixo
*
************************************************************************/

#include "asf.h"


#define PIN_PUSHBUTTON_1_MASK	PIO_PB3
#define PIN_PUSHBUTTON_1_PIO	PIOB
#define PIN_PUSHBUTTON_1_ID		ID_PIOB
#define PIN_PUSHBUTTON_1_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_1_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE

#define PIN_PUSHBUTTON_2_MASK	PIO_PC12
#define PIN_PUSHBUTTON_2_PIO	PIOC
#define PIN_PUSHBUTTON_2_ID		ID_PIOC
#define PIN_PUSHBUTTON_2_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_2_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_FALL_EDGE

/** IRQ priority for PIO (The lower the value, the greater the priority) */
#define IRQ_PRIOR_PIO    0

#define Freq_Init_Blink 4	//Hz

/**mascaras*/
#define PIN_LED_BLUE	19
#define PIN_LED_RED		20
#define PIN_LED_GREEN	20
#define PIN_BUTTON		3
#define PIN_BUTTON_1	12	
#define time			100

/** 
 * Definição dos ports
 * Ports referentes a cada pino
 */
#define PORT_LED_BLUE	PIOA
#define PORT_LED_GREEN	PIOA
#define PORT_LED_RED	PIOC
#define PORT_BUT_2		PIOB
#define PORT_BUT_1		PIOC

/**
 * Define os IDs dos periféricos associados aos pinos
 */
#define ID_LED_BLUE		ID_PIOA
#define ID_LED_GREEN	ID_PIOA
#define ID_LED_RED		ID_PIOC
#define ID_BUT_2		ID_PIOB
#define ID_BUT_1		ID_PIOC

/**
 *	Define as masks utilziadas
 */
#define MASK_LED_BLUE	(1u << PIN_LED_BLUE)
#define MASK_LED_GREEN	(1u << PIN_LED_GREEN)
#define MASK_LED_RED	(1u << PIN_LED_RED)
#define MASK_BUT_2		(1u << PIN_BUTTON)
#define MASK_BUT_1		(1u << PIN_BUTTON_1)


/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{	
	tc_write_rc(TC0,0,tc_read_rc(TC0,0)*0.9);	
}

/**
 *  Handle Interrupcao botao 2.
 */
 
static void Button2_Handler(uint32_t id, uint32_t mask)
{	
	tc_write_rc(TC0,0,tc_read_rc(TC0,0)*1.1);	
}

/**
 *  Interrupt handler for TC0 interrupt. 
 */
 
void TC0_Handler(void)
{

	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0,0);

	if (PORT_LED_GREEN->PIO_ODSR & MASK_LED_GREEN)
		pio_clear(PIOA, MASK_LED_GREEN);
	else
		pio_set(PIOA, MASK_LED_GREEN);
    
}



/**
 *  \brief Configure the Pushbuttons
 *
 *  Configure the PIO as inputs and generate corresponding interrupt when
 *  pressed or released.
 */
static void configure_buttons(void)
{	
	pmc_enable_periph_clk(ID_BUT_2);
	pmc_enable_periph_clk(ID_BUT_1);
	pio_set_input(PORT_BUT_2, MASK_BUT_2, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(PORT_BUT_1, MASK_BUT_1, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(PORT_BUT_2, MASK_BUT_2, 20);
	pio_set_debounce_filter(PORT_BUT_1, MASK_BUT_1, 20);
	pio_handler_set(PORT_BUT_2, ID_BUT_2, MASK_BUT_2, PIO_IT_FALL_EDGE , Button1_Handler );
	pio_handler_set(PORT_BUT_1, ID_BUT_1, MASK_BUT_1, PIO_IT_FALL_EDGE , Button2_Handler );
	pio_enable_interrupt(PORT_BUT_2, MASK_BUT_2);
	pio_enable_interrupt(PORT_BUT_1, MASK_BUT_1);
	NVIC_SetPriority((IRQn_Type) ID_PIOB, 2);
	NVIC_SetPriority((IRQn_Type) ID_PIOC, 2);
	NVIC_EnableIRQ(ID_BUT_2);
	NVIC_EnableIRQ(ID_BUT_1);	
		
}


static void configure_leds(void)
{	
	pmc_enable_periph_clk(ID_LED_BLUE);
	pio_set_output(PORT_LED_GREEN , MASK_LED_GREEN  ,1,0,0);
}



static void configure_tc(void)
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
    * 	* Configurações de modo de operação :
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
    tc_write_rc(TC0,0,32768/4);
	
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

int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/** Configura o timer */
	configure_tc();
	
	/* Configura os botões */
	configure_buttons();

    /* Configura Leds */
    configure_leds();

    
	while (1) {
		/* Entra em modo sleep */
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
