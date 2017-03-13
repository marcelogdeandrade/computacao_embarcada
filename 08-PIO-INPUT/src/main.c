/**************************************************************************
* Rafael Corsi   - Insper 
* rafael.corsi@insper.edu.br        
*
* Computa��o Embarcada
*
* 08-PIO-ENTRADA
************************************************************************/


#include "asf.h"
#include "conf_clock.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs
 */

#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN			8
#define LED_PIN_MASK	(1<<LED_PIN) 

/**
 * Bot�o
 */ 
#define BUT_PIO_ID		
#define BUT_PIO         
#define BUT_PIN			
#define BUT_PIN_MASK	

/************************************************************************/
/* Prototipa��o                                                        */
/************************************************************************/
void ledConfig();

/************************************************************************/
/* Fun��es	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(){
	PMC->PMC_PCER0 = (1<<LED_PIO_ID);
	PIOC->PIO_OER = (1 << 8);
	PIOC->PIO_PER = (1 << 8);
	PIOC->PIO_CODR =  (1 << 8);
};

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{

	/************************************************************************/
	/* Inicializa��o b�sica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo sa�da
	ledConfig();

	// Configura botao
	PMC->PMC_PCER0= (1<<10);

	PIOA->PIO_PER = (1<<11);
	PIOA->PIO_ODR = (1<<11);
	PIOA->PIO_PUER= (1<<11);
	PIOA->PIO_IFER= (1<<11);

	// Configura botoes ext
	//PMC->PMC_PCER0= (1<<12);

	//PIOC->PIO_PER = (1<<31);
	//PIOC->PIO_ODR = (1<<31);
	//PIOC->PIO_PUER= (1<<31);
	//PIOC->PIO_IFER= (1<<31);

	PIOA->PIO_PER = (1<<19);
	PIOA->PIO_ODR = (1<<19);
	PIOA->PIO_PUER= (1<<19);
	PIOA->PIO_IFER= (1<<19);

	PMC->PMC_PCER0= (1<<16);

	PIOD->PIO_PER = (1<<28);
	PIOD->PIO_ODR = (1<<28);
	PIOD->PIO_PUER= (1<<28);
	PIOD->PIO_IFER= (1<<28);


	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
	  int freq = 3000000;
	  if (!(PIOA->PIO_PDSR & (1<<19)) )
		freq = 1500000;
	  if (!(PIOD->PIO_PDSR & (1<<28)) )
		freq = 6000000;
	  if (PIOA->PIO_PDSR & (1<<11) )
	    PIOC->PIO_SODR = (1<<8);
	  else
		PIOC->PIO_CODR =  (1 << 8);
		for (int i = 0; i < freq; i++){
		}
		PIOC->PIO_SODR =  (1 << 8);
		for (int i = 0; i < freq; i++){
		}
		

		/*
	  if( PIOA->PIO_PDSR & (1<<11) ){
		PIOC->PIO_SODR = (1<<8);
	  } else {
		// Piscar
		
		PIOC->PIO_CODR =  (1 << 8);
		for (int i = 0; i < freq; i++){
		}
		PIOC->PIO_SODR =  (1 << 8);
		for (int i = 0; i < freq; i++){
		*/
		
	  
	}
}


