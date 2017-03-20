/************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* 10-PIO-INTERRUPCAO
*
* [ref] http://www.atmel.com/Images/Atmel-42142-SAM-AT03258-Using-Low-Power-Mode-in-SAM4E-Microcontroller_Application-Note.pdf
* [ref] https://www.eecs.umich.edu/courses/eecs373/labs/refs/M3%20Guide.pdf
************************************************************************/


#include "asf.h"
#include "conf_clock.h"



/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs
 */
#define LED_PIO_ID		 ID_PIOC
#define LED_PIO          PIOC
#define LED_PIN		     8
#define LED_PIN_MASK     (1<<LED_PIN)

#define LED1_PIO_ID		 ID_PIOA
#define LED1_PIO         PIOA	
#define LED1_PIN		 0
#define LED1_PIN_MASK    (1<<LED1_PIN)

#define LED2_PIO_ID		 ID_PIOC
#define LED2_PIO         PIOC
#define LED2_PIN		 30
#define LED2_PIN_MASK    (1<<LED2_PIN)

#define LED3_PIO_ID		 ID_PIOB
#define LED3_PIO         PIOB
#define LED3_PIN		 2
#define LED3_PIN_MASK    (1<<LED3_PIN)

/**
 * Botão
 */
#define BUT1_PIO_ID      ID_PIOD
#define BUT1_PIO         PIOD
#define BUT1_PIN		 28
#define BUT1_PIN_MASK    (1 << BUT1_PIN)

#define BUT2_PIO_ID      ID_PIOC
#define BUT2_PIO         PIOC
#define BUT2_PIN		 31
#define BUT2_PIN_MASK    (1 << BUT2_PIN)

#define BUT3_PIO_ID      ID_PIOA
#define BUT3_PIO         PIOA
#define BUT3_PIN		 19
#define BUT3_PIN_MASK    (1 << BUT3_PIN)

/************************************************************************/
/* prototype                                                             */
/************************************************************************/
void led_init(int estado);
void but_init(void);
void but_Handler1();
void but_Handler2();
void but_Handler3();

/************************************************************************/
/* Interrupçcões                                                        */
/************************************************************************/

void but_Handler1(){
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED1_PIO, LED1_PIN_MASK))
    pio_clear(LED1_PIO, LED1_PIN_MASK);
   else
    pio_set(LED1_PIO,LED1_PIN_MASK);

    
}

void but_Handler2(){
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED2_PIO, LED2_PIN_MASK))
    pio_clear(LED2_PIO, LED2_PIN_MASK);
   else
    pio_set(LED2_PIO,LED2_PIN_MASK);

    
}


void but_Handler3(){
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED3_PIO, LED3_PIN_MASK))
    pio_clear(LED3_PIO, LED3_PIN_MASK);
   else
    pio_set(LED3_PIO,LED3_PIN_MASK);

    
}


/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void led_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, 1, 0, 0 );

    pmc_enable_periph_clk(LED1_PIO_ID);
    pio_set_output(LED1_PIO, LED1_PIN_MASK, 1, 0, 0 );

    pmc_enable_periph_clk(LED2_PIO_ID);
    pio_set_output(LED2_PIO, LED2_PIN_MASK, 1, 0, 0 );

    pmc_enable_periph_clk(LED3_PIO_ID);
    pio_set_output(LED3_PIO, LED3_PIN_MASK, 1, 0, 0 );
};

/**
 * @Brief Inicializa o pino do BUT
 *  config. botao em modo entrada enquanto 
 *  ativa e configura sua interrupcao.
 */
void but_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT1_PIO_ID);
    pio_set_input(BUT1_PIO, BUT1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_set_input(BUT2_PIO, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_set_input(BUT3_PIO, BUT3_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
    pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);
    pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, but_Handler1);

	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIN_MASK, PIO_IT_RISE_EDGE, but_Handler2);

	pio_enable_interrupt(BUT3_PIO, BUT3_PIN_MASK);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIN_MASK, PIO_IT_RE_OR_HL, but_Handler3);

    
    /* habilita interrupçcão do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT1_PIO_ID);
    NVIC_SetPriority(BUT1_PIO_ID, 1);

	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 1);
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 1);
};

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{
	/************************************************************************/
	/* Inicialização básica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;

	/************************************************************************/
	/* Inicializao I/OS                                                     */
	/************************************************************************/
	led_init(1);
    but_init();

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
       /* entra em modo sleep */
       pmc_sleep(SLEEPMGR_SLEEP_WFI);
				for (int i = 0; i < 1125000; i++){
				    pio_clear(LED_PIO, LED_PIN_MASK);
				}
				for (int i = 0; i < 1125000; i++){
					pio_set(LED_PIO, LED_PIN_MASK);
				}
				for (int i = 0; i < 1125000; i++){
					pio_clear(LED_PIO, LED_PIN_MASK);
				}
				for (int i = 0; i < 1125000; i++){
					pio_set(LED_PIO, LED_PIN_MASK);
				}
	};
}


