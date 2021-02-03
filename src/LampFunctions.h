#ifndef LampFunctions

#include "stm32f0xx.h"

#define LampFunctions

#define UART_TX_Pin 9
#define UART_TX_Port GPIOA
#define UART_RX_Pin 10
#define UART_RX_Port GPIOA

#define I2C_SCL_Pin 8
#define I2C_SCL_Port GPIOB
#define I2C_SDA_Pin 9
#define I2C_SDA_Port GPIOB
#define I2C_SLAVE_ADDR 0x04
#define I2C_BYTECOUNT 4

void setupPortAndPin_AF(GPIO_TypeDef *port, int pinNumber){
	if (port == GPIOA) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	else if (port == GPIOB) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	else if (port == GPIOC) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	else if (port == GPIOD) RCC->AHBENR |= RCC_AHBENR_GPIODEN;
	else if (port == GPIOF) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

	//because each pin has two places that need to be assigned, the register goes from
	//0-31, and pins go from 0-15, so to access the MSB of the pin, we do pinNumber*2+1
	//and for the LSB we do pinNumber*2
	port->MODER |= (1 << ((pinNumber*2)+1)); //set MSB to 1 and LSB to 0 for Alternate Function
	port->MODER &= ~(1 << pinNumber*2);    //

	port->OTYPER |= (1<<pinNumber); //set type to open drain (not push/pull)

	port->OSPEEDR &= ~(1<<pinNumber*2); //set to low speed

	port->PUPDR &= ~(1<<pinNumber); //not using PUPD

	if(pinNumber>7){ //set AFR, assuming we are using the first alternate function available
		port->AFR[1] |= (1<<(pinNumber%8)*4);
	} else{
		port->AFR[0] |= (1<<pinNumber*4);
	}
}

void setupPortAndPin_Output(GPIO_TypeDef *port, int pinNumber){
	if (port == GPIOA) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	else if (port == GPIOB) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	else if (port == GPIOC) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	else if (port == GPIOD) RCC->AHBENR |= RCC_AHBENR_GPIODEN;
	else if (port == GPIOF) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
	//because each pin has two places that need to be assigned, the register goes from
	//0-31, and pins go from 0-15, so to access the MSB of the pin, we do pinNumber*2+1
	//and for the LSB we do pinNumber*2
	port->MODER &= ~(1 << ((pinNumber*2)+1)); //set MSB to 0 and LSB to 1 for General output
	port->MODER |= (1 << pinNumber*2);    //

	port->OTYPER &= ~(1<<pinNumber); //set type to push/pull (not open drain)

	port->OSPEEDR |= (1 << ((pinNumber*2)+1));//set MSB to 1 and LSB to 1 for High Speed
	port->OSPEEDR |= (1 << pinNumber*2);  // (can be combined into the same line since both ORs)

	port->PUPDR &= ~(1<<pinNumber); //not using PUPD
}

void setupUART(USART_TypeDef *USARTx){
	setupPortAndPin_AF(UART_TX_Port, UART_TX_Pin);
	setupPortAndPin_AF(UART_RX_Port, UART_RX_Pin);

	if(USARTx == USART1){
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //enable USART1
	} else if (USARTx == USART2){
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //enable USART2
	}

	USARTx->BRR = 0x0341; //for 9600 Baud @ 8MHz clk
	//Enable         TX Done      |Receive not empty | Transmit empty  |      TX      |      RX      |    UART1
//	USARTx->CR1 |= USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
	//Enable       Receive not empty|      RX      |    UART1
	USARTx->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;

}

void setupI2C(I2C_TypeDef *I2Cx){
	setupPortAndPin_AF(I2C_SCL_Port, I2C_SCL_Pin);
	setupPortAndPin_AF(I2C_SDA_Port, I2C_SDA_Pin);

	if(I2Cx == I2C1){
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //enable I2C1
	} else if (I2Cx == I2C2){
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; //enable I2C2
	}

	I2C1->TIMINGR = (uint32_t)0x00201D2B; //master mode, std mode, 100KHz @ 8MHz Clk
	I2C1->CR1 |= I2C_CR1_PE; //enable I2C
	I2C1->CR2 |= I2C_SLAVE_ADDR << 1; //slave address 4
	I2C1->CR2 &= ~I2C_CR2_RD_WRN; //write mode
	I2C1->CR2 |= (I2C_BYTECOUNT<<16); //1 byte(s)
}

void I2CStartTransmission(){
	I2C1->CR2 |= I2C_CR2_START; //start
	while (I2C1->CR2 & I2C_CR2_START); //wait for start bit to be cleared by HW
}

void I2CStopTransmission(){
	I2C1->CR2 |= I2C_CR2_STOP; //stop
	while (I2C1->CR2 & I2C_CR2_STOP); //wait for stop bit to be cleared by HW
}

void I2CTransmitChar(char character){ //transmits a character and only a character
	I2CStartTransmission();
	I2C1->TXDR = character; //send character into transmit register
	while(!(I2C1->ISR & I2C_ISR_TXE)); //wait for transmit register to be empty (has been transmitted)
	I2CStopTransmission();
}

void I2CTransmitCharNSS(char character){ //transmits a character but is used for transmitting multiple chars at a time
	I2C1->TXDR = character; //send character into transmit register
	while(!(I2C1->ISR & I2C_ISR_TXE)); //wait for transmit register to be empty (has been transmitted)
}

void I2CTransmitString(char* str, int strlength){ //transmits a string str of length strlength
	int i;
	I2CStartTransmission();
	for (i=0;i<strlength;i++){

		I2CTransmitCharNSS(str[i]);
	}
	I2CStopTransmission();
}


#endif
