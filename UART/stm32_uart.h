/*
 * stm32_uart.h
 *
 *  Created on: May 20, 2023
 *      Author: LorisRT
 */

#ifndef STM32_UART_H_
#define STM32_UART_H_

#define RCC_BASE_ADDR		0x40023800U
#define GPIOC_BASE_ADDR		0x40020800U
#define UART4_BASE_ADDR		0x40004c00U
#define SYST_BASE_ADDR		0xe000e010U
#define NVIC_ISR_BASE_ADDR	0xe000e100U
#define NVIC_ICR_BASE_ADDR	0Xe000e180U
#define NVIC_ISR1		(NVIC_ISR_BASE_ADDR + 0x04)
#define NVIC_ICR1		(NVIC_ICR_BASE_ADDR + 0x04)

#define HSE_FLAG_LIM		100000U
#define UART_FLAG_LIM		1000000U
#define UART_BUFFER_SIZE	200

#define IRQ_NUMBER_UART4	52

#define FRAME_TERMINATOR	'\r'

#define GET_RCC_HSERDY_FLAG		(1U << 17)
#define GET_UART_TXE_FLAG		(1U << 7)
#define GET_UART_RXNE_FLAG		(1U << 5)


/* ************************************ */
/* *** Enumeration field definition *** */
/* ************************************ */
typedef enum {
	STM_OK,
	STM_INIT_ERROR,
	STM_CONFIG_ERROR,
	STM_UNKNOWN_ERRROR
} STM_STATUS_e;

typedef enum {
	UART_OK,
	UART_ERROR_TX,
	UART_ERROR_RX,
	UART_ERROR_UNKNOWN,
	UART_RX_DISCARD,
	UART_RX_FRAME,
	UART_RX_EMPTY
} STM_UART_e;



/* ********************************** */
/* *** Structure field definition *** */
/* ********************************** */
typedef struct {
	volatile uint32_t CR;
	uint32_t RESERVED0;
	volatile uint32_t CFGR;
	uint32_t RESERVED1[9];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
} __attribute__((packed)) RCC_t;

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
} __attribute__((packed)) GPIO_t;

typedef struct {
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} __attribute__((packed)) UART_t;

typedef struct {
	volatile char data[UART_BUFFER_SIZE];
	volatile uint32_t size;
	volatile uint8_t flag_frame;
	volatile uint8_t flag_discard;
} UART_Buffer_t;



/* ************************** */
/* *** Function prototype *** */
/* ************************** */
void configPeripheralClock(void);
void configGPIO4UART(void);
void configUART4(void);
STM_STATUS_e enableClock(void);
STM_UART_e UART_writeBurst(const char *s);
STM_UART_e UART_read(char *c);



#endif /* STM32_UART_H_ */
