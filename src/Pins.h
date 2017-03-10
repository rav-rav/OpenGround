/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
//*******************
//***   Pinouts   ***
//*******************
//#define gpio_set(GPIO, PIN)
//#define gpio_clear(GPIO, PIN)

/*
#define	PE1_gpio			  0 //PB4								//PE1
#define	PE1_pin			  0 //PB4								//PE1
#define	PE2_gpio			  0 //PB4								//PE1
#define	PE2_pin			  0 //PB5								//PE2

//CS pins
#define	CC25_CSN_pin	0 //PB6								//CC2500
#define	NRF_CSN_pin		0 //PB7								//NRF24L01
#define	CYRF_RST_pin	0 //PB8								//CYRF RESET
#define	A7105_CSN_pin	0 //PB9								//A7105
#define	CYRF_CSN_pin	0 //PB12							//CYRF CSN

//SPI pins
#define	SCK_pin			  0 //PB13							//SCK
#define	SDO_pin			  0 //PB14							//MISO
#define	SDI_pin			  0 //PB15							//MOSI			

#define HIGH 1
#define LOW 0

//
#define	PE1_on  		gpio_set(PE1_gpio, PE1_pin)
#define	PE1_off		 	gpio_clear(PE1_gpio, PE1_pin)
//
#define	PE2_on  		digitalWrite(PE2_pin,HIGH)
#define	PE2_off 		digitalWrite(PE2_pin,LOW)

#define	A7105_CSN_on	digitalWrite(A7105_CSN_pin,HIGH)			
#define	A7105_CSN_off	digitalWrite(A7105_CSN_pin,LOW)		

#define NRF_CE_on
#define	NRF_CE_off

#define	SCK_on			digitalWrite(SCK_pin,HIGH)			
#define	SCK_off			digitalWrite(SCK_pin,LOW)		

#define	SDI_on			digitalWrite(SDI_pin,HIGH)			
#define	SDI_off			digitalWrite(SDI_pin,LOW)		

#define	SDI_1			(digitalRead(SDI_pin)==HIGH)	
#define	SDI_0			(digitalRead(SDI_pin)==LOW)			

#define	CC25_CSN_on		digitalWrite(CC25_CSN_pin,HIGH)		
#define	CC25_CSN_off	digitalWrite(CC25_CSN_pin,LOW)	

#define	NRF_CSN_on		digitalWrite(NRF_CSN_pin,HIGH)		
#define	NRF_CSN_off		digitalWrite(NRF_CSN_pin,LOW)	

#define	CYRF_CSN_on		digitalWrite(CYRF_CSN_pin,HIGH)		
#define	CYRF_CSN_off	digitalWrite(CYRF_CSN_pin,LOW)

#define	CYRF_RST_HI		digitalWrite(CYRF_RST_pin,HIGH)	//reset cyrf 
#define	CYRF_RST_LO		digitalWrite(CYRF_RST_pin,LOW)	//

#define	SDO_1			(digitalRead(SDO_pin)==HIGH)		
#define	SDO_0			(digitalRead(SDO_pin)==LOW)	

#define	TX_INV_on		digitalWrite(TX_INV_pin,HIGH)		
#define	TX_INV_off		digitalWrite(TX_INV_pin,LOW)

#define	RX_INV_on		digitalWrite(RX_INV_pin,HIGH)		
#define	RX_INV_off		digitalWrite(RX_INV_pin,LOW)
*/


// cc2500 module connection
// SI = SDIO
// SCK = SCK
// MISO = GD1
// GDO2 = GDO2
// GDO0 = RF1
// CSN = SCS
// PA_EN = TX = TXW
// LNA_EN = RX = RXW

#define MULTI4IN1_SPI_GPIO          GPIOE
                                    
// LABELED SCK                      
#define MULTI4IN1_SPI_SCK_PIN       GPIO13
// LABELED SPIO                     
#define MULTI4IN1_SPI_MOSI_PIN      GPIO15
// LABELED GIO1                     
#define MULTI4IN1_SPI_MISO_PIN      GPIO14
                                    
                                    
// LABELED RF1                      
#define MULTI4IN1_PE1_GPIO          GPIOE
#define MULTI4IN1_PE1_PIN           GPIO11
                                    
// LABELED RF2                      
#define MULTI4IN1_PE2_GPIO          GPIOE
#define MULTI4IN1_PE2_PIN           GPIO12

// LABELED TX_W
#define MULTI4IN1_A7105_CSN_GPIO    GPIOE
#define MULTI4IN1_A7105_CSN_PIN     GPIO8

// LABELED RX-W
#define MULTI4IN1_CC25_CSN_GPIO     GPIOE
#define MULTI4IN1_CC25_CSN_PIN      GPIO9

//TODO: rav - define these
#define MULTI4IN1_CYRF_CSN_GPIO     GPIOB
#define MULTI4IN1_CYRF_CSN_PIN      GPIO2

//TODO: rav - define these
#define MULTI4IN1_CYRF_RST_GPIO     GPIOB
#define MULTI4IN1_CYRF_RST_PIN      GPIO2

//TODO: rav - define these
#define MULTI4IN1_NRF_CSN_GPIO      GPIOB
#define MULTI4IN1_NRF_CSN_PIN       GPIO2

// LABELED GIO2
#define MULTI4IN1_NRF_CE_GPIO       GPIOB
#define MULTI4IN1_NRF_CE_PIN        GPIO2


/*
#define MULTI4IN1_SPI                  SPI1
#define MULTI4IN1_SPI                  SPI1
#define MULTI4IN1_SPI_DR               SPI1_DR
#define MULTI4IN1_SPI_CLK              RCC_SPI1
#define MULTI4IN1_SPI_DMA_CLOCK        RCC_DMA1
#define MULTI4IN1_SPI_TX_DMA_CHANNEL   DMA_CHANNEL3
#define MULTI4IN1_SPI_TX_DMA_TC_FLAG   DMA1_FLAG_TC3
#define MULTI4IN1_SPI_RX_DMA_CHANNEL   DMA_CHANNEL2
#define MULTI4IN1_SPI_RX_DMA_TC_FLAG   DMA1_FLAG_TC2
*/


#define SCK_on          gpio_set(MULTI4IN1_SCK_GPIO, MULTI4IN1_SCK_PIN)         
#define SCK_off         gpio_clear(MULTI4IN1_SCK_GPIO, MULTI4IN1_SCK_PIN)     

#define SDI_on          gpio_set(MULTI4IN1_SDI_GPIO, MULTI4IN1_SDI_PIN)          
#define SDI_off         gpio_clear(MULTI4IN1_SDI_GPIO, MULTI4IN1_SDI_PIN)      

#define PE1_on          gpio_set(MULTI4IN1_PE1_GPIO, MULTI4IN1_PE1_PIN)
#define PE1_off         gpio_clear(MULTI4IN1_PE1_GPIO, MULTI4IN1_PE1_PIN)
//
#define PE2_on          gpio_set(MULTI4IN1_PE2_GPIO, MULTI4IN1_PE2_PIN)
#define PE2_off         gpio_clear(MULTI4IN1_PE2_GPIO, MULTI4IN1_PE2_PIN)

#define A7105_CSN_on    gpio_set(MULTI4IN1_A7105_CSN_GPIO, MULTI4IN1_A7105_CSN_PIN)            
#define A7105_CSN_off   gpio_clear(MULTI4IN1_A7105_CSN_GPIO, MULTI4IN1_A7105_CSN_PIN)    

#define CC25_CSN_on     gpio_set(MULTI4IN1_CC25_CSN_GPIO, MULTI4IN1_CC25_CSN_PIN)     
#define CC25_CSN_off    gpio_clear(MULTI4IN1_CC25_CSN_GPIO, MULTI4IN1_CC25_CSN_PIN) 

#define CYRF_CSN_on     gpio_set(MULTI4IN1_CYRF_CSN_GPIO, MULTI4IN1_CYRF_CSN_PIN)    
#define CYRF_CSN_off    gpio_clear(MULTI4IN1_CYRF_CSN_GPIO, MULTI4IN1_CYRF_CSN_PIN)

#define CYRF_RST_HI     gpio_set(MULTI4IN1_CYRF_RST_GPIO, MULTI4IN1_CYRF_RST_PIN)
#define CYRF_RST_LO     gpio_clear(MULTI4IN1_CYRF_RST_GPIO, MULTI4IN1_CYRF_RST_PIN)

#define NRF_CSN_on      gpio_set(MULTI4IN1_NRF_CSN_GPIO, MULTI4IN1_NRF_CSN_PIN)     
#define NRF_CSN_off     gpio_clear(MULTI4IN1_NRF_CSN_GPIO, MULTI4IN1_NRF_CSN_PIN) 

#define NRF_CE_on       gpio_set(MULTI4IN1_NRF_CE_GPIO, MULTI4IN1_NRF_CE_PIN)  
#define NRF_CE_off      gpio_clear(MULTI4IN1_NRF_CE_GPIO, MULTI4IN1_NRF_CE_PIN)
