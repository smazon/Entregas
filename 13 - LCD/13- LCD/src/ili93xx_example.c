/**
 * \file
 *
 * \brief lcd controller ili93xx example.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage lcd controller ili93xx Example
 *
 * \section Purpose
 *
 * This example demonstrates how to configure lcd controller ili9325 or
 * ili9341 to control the LCD on the board.
 *
 * \section Requirements
 *
 * This package can be used with SAM4E-EK evaluation kits.
 *
 * \section Description
 *
 * This example first configure ili93xx for access the LCD controller,
 * then initialize the LCD, finally draw some text, image, basic shapes (line,
 * rectangle, circle) on LCD.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board.
 * -# Some text, image and basic shapes should be displayed on the LCD.
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "smc.h"

/** Chip select number to be set */
#define ILI93XX_LCD_CS      1

struct ili93xx_opt_t g_ili93xx_display_opt;

/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};

	/** Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

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

#define PIN_BUTTON		3
#define PIN_BUTTON_1	12	
#define time			100

/** 
 * Definição dos ports
 * Ports referentes a cada pino
 */

#define PORT_BUT_2		PIOB
#define PORT_BUT_1		PIOC

/**
 * Define os IDs dos periféricos associados aos pinos
 */

#define ID_BUT_2		ID_PIOB
#define ID_BUT_1		ID_PIOC

/**
 *	Define as masks utilziadas
 */

#define MASK_BUT_2		(1u << PIN_BUTTON)
#define MASK_BUT_1		(1u << PIN_BUTTON_1)
#define a 32
int num = 0;
uint8_t vet[a];

/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{	
	sprintf(vet,"Contador: %d", ++num);
	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(10, 119, ILI93XX_LCD_WIDTH,150);
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(10, 120, vet);
}

/**
 *  Handle Interrupcao botao 2.
 */
 
static void Button2_Handler(uint32_t id, uint32_t mask)
{	
	sprintf(vet, "Contador: %d", --num);
	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(10, 119, ILI93XX_LCD_WIDTH,150);
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(10, 120, vet);
}

/**
 *  Interrupt handler for TC0 interrupt. 
 */
 
void TC0_Handler(void)
{		static b = 0;
		volatile uint32_t ul_dummy;
		ul_dummy = tc_get_status(TC0,0);
		ili93xx_set_foreground_color(COLOR_BLACK);
		ili93xx_draw_string(10,120, (uint8_t *)"Contador:");
		sprintf(vet, "Timer: %d", b);
		ili93xx_set_foreground_color(COLOR_WHITE);
		ili93xx_draw_filled_rectangle(119, 299, ILI93XX_LCD_WIDTH,319);
		ili93xx_set_foreground_color(COLOR_BLACK);
		ili93xx_draw_string(120, 300, vet);
		b++;
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
    tc_write_rc(TC0,0,32768);
	
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


/**
 * \brief Application entry point for smc_lcd example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	sysclk_init();
	board_init();

	/** Initialize debug console */
	configure_console();
	
	/** Configura o timer */
	configure_tc();
	
	/* Configura os botões */
	configure_buttons();

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

	/** Draw text, image and basic shapes on the LCD */
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(80, 10, (uint8_t *)"13 - LCD");
	ili93xx_draw_string(10, 40, (uint8_t *)"Andre");
	ili93xx_draw_string(10, 60, (uint8_t *)"Thales");
	ili93xx_draw_string(10, 80, (uint8_t *)"Tete");
	ili93xx_set_foreground_color(COLOR_ORANGE);
	ili93xx_draw_filled_rectangle(0, 100, ILI93XX_LCD_WIDTH,105);
	ili93xx_set_foreground_color(COLOR_GREEN);
	ili93xx_draw_filled_rectangle(0, 106, ILI93XX_LCD_WIDTH,111);
	
	while (1) {
		/* Entra em modo sleep */
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}

