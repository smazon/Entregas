/**
 * IMT - Rafael Corsi
 * 
 * PIO - 07
 *  Configura o PIO do SAM4S (Banco A, pino 19) para operar em
 *  modo de output. Esse pino está conectado a um LED, que em 
 *  lógica alta apaga e lógica baixa acende.
*/

#include <asf.h>

/*
 * Prototypes
 */

/** 
 * Definição dos pinos
 * Pinos do uC referente aos LEDS.
 *
 * O número referente ao pino (PIOAxx), refere-se ao
 * bit que deve ser configurado no registrador para alterar
 * o estado desse bit específico.
 *
 * exe : O pino PIOA_19 é configurado nos registradores pelo bit
 * 19. O registrador PIO_SODR configura se os pinos serão nível alto.
 * Nesse caso o bit 19 desse registrador é referente ao pino PIOA_19
 *
 * ----------------------------------
 * | BIT 19  | BIT 18  | ... |BIT 0 |
 * ----------------------------------
 * | PIOA_19 | PIOA_18 | ... |PIOA_0|
 * ----------------------------------
 */
#define PIN_LED_BLUE 19
#define PIN_LED_GREEN 20
#define PIN_LED_RED 20
#define n=30



void function(int id, int pin, Pio * port);
/**
 * Main function
 * 1. configura o clock do sistema
 * 2. desabilita wathdog
 * 3. ativa o clock para o PIOA
 * 4. ativa o controle do pino ao PIO
 * 5. desabilita a proteção contra gravações do registradores
 * 6. ativa a o pino como modo output
 * 7. coloca o HIGH no pino
 */

int main (void)
{

	/**
	* Inicializando o clock do uP
	*/
	sysclk_init();
	
	/** 
	*  Desabilitando o WathDog do uP
	*/
	WDT->WDT_MR = WDT_MR_WDDIS;
		
	
	/*
	PMC->PMC_PCER0 |= ID_PIOA | ID_PIOC; // initializing the peripherals

	 //31.6.1 PIO Enable Register
	// 1: Enables the PIO to control the corresponding pin (disables peripheral control of the pin).	
	PIOA->PIO_PER |= (1 << PIN_LED_BLUE );
	PIOA ->PIO_PER |= (1<< PIN_LED_GREEN);
	PIOC ->PIO_PER |= (1<< PIN_LED_RED);

	// 31.6.46 PIO Write Protection Mode Register
	// 0: Disables the write protection if WPKEY corresponds to 0x50494F (PIO in ASCII). 
	//OS REGISTRADORES PODEM SER ALTERADOS
	PIOA->PIO_WPMR = 0;
	PIOC->PIO_WPMR = 0;
	// 31.6.4 PIO Output Enable Register
	// value =
	//	 	1 : Enables the output on the I/O line.
	//	 	0 : do nothing
	PIOA->PIO_OER |=  (1 << PIN_LED_BLUE );
	PIOA->PIO_OER |=  (1 << PIN_LED_GREEN );
	PIOC->PIO_OER |= (1 << PIN_LED_RED);
	// 31.6.10 PIO Set Output Data Register
	// value = 
	// 		1 : Sets the data to be driven on the I/O line.
	// 		0 : do nothing
	
    */
	
	
	/**
	*	Loop infinito
	*/
	function(ID_PIOA, PIN_LED_BLUE, PIOA);
	function(ID_PIOA, PIN_LED_GREEN, PIOA);
	function(ID_PIOC, PIN_LED_RED, PIOC);

		while(1){

            /*
             * Utilize a função delay_ms para fazer o led piscar na frequência
             * escolhida por você.
             */
			PIOC->PIO_CODR = (1 << PIN_LED_RED );// sets the output to 1 and then the red led is turned off
			delay_ms(500);
			PIOA->PIO_SODR = (1 << PIN_LED_BLUE );// sets the output to 1 and then the blue led is turned off
			delay_ms(500);
			PIOA->PIO_SODR = (1 << PIN_LED_GREEN);// sets the output to 1 and then the green led is turned off
            delay_ms(300);		
			PIOC->PIO_SODR = (1 << PIN_LED_RED );// sets the output to 0 and then the red led is turned on
			delay_ms(500);
			PIOA->PIO_CODR = (1 << PIN_LED_BLUE ); // sets the output to 0 and then the blue led is turned on
			delay_ms(500);
			PIOA->PIO_CODR = (1 << PIN_LED_GREEN);// sets the output to 0 and then the green led is turned on
			delay_ms(300);
	}
}
void function(int id, int pin, Pio * port) {
	// 29.17.4 PMC Peripheral Clock Enable Register 0
	// 1: Enables the corresponding peripheral clock.
	// ID_PIOA = 11 - TAB 11-1
	
	PMC->PMC_PCER0 = id;
	port-> PIO_PER = (1 << pin );
	port->PIO_WPMR = 0;
	port->PIO_OER |=  (1 << pin );
}


