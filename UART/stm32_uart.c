/*
 * stm32_uart.c
 *
 *  Created on: May 20, 2023
 *      Author: LorisRT
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32_uart.h"



/* ************************************************* */
/* *** Register structure and pointer definition *** */
/* ************************************************* */
static volatile RCC_t * const RCC = (volatile RCC_t *) RCC_BASE_ADDR;
static volatile GPIO_t * const GPIO_C = (volatile GPIO_t *) GPIOC_BASE_ADDR;
static volatile UART_t * const UART4 = (volatile UART_t *) UART4_BASE_ADDR;
static volatile uint32_t * const ptr_NVIC_ISR1 = (volatile uint32_t *) NVIC_ISR1;
static volatile UART_Buffer_t buffer;



/* ********************************* */
/* *** Static function prototype *** */
/* ********************************* */
static void flushBuffer(void);
static void buffer_memcpy(char *, const volatile char *, uint32_t);



/* *************************** */
/* *** Function definition *** */
/* *************************** */
/**
 * @brief UART Interrupt Service Routine (ISR)
 * @author LorisRT
 */
void UART4_IRQHandler(void)
{
	char dummy_read;

	/* Received byte processing */
	if (UART4->SR & GET_UART_RXNE_FLAG)
	{
		dummy_read = (char) UART4->DR;
		if (buffer.size < UART_BUFFER_SIZE)
		{
			if (dummy_read == FRAME_TERMINATOR)
			{
				(void) dummy_read;
				buffer.flag_frame = 1;
			}
			else
			{
				buffer.data[buffer.size++] = dummy_read;
			}
		}
		else
		{
			(void) dummy_read;
			buffer.flag_discard = 1;
		}
	}
}

/**
 * @brief
 * @author LorisRT
 */
STM_UART_e UART_read(char *c)
{
	if (buffer.flag_discard)
	{
		flushBuffer();
		return UART_RX_DISCARD;
	}

	if (buffer.flag_frame)
	{
		buffer_memcpy(c, buffer.data, buffer.size);
		/* Add CR + LN for putty terminal */
		*(c + buffer.size) = '\r';
		*(c + (buffer.size + 1)) = '\n';
		flushBuffer();
		return UART_RX_FRAME;
	}

	return UART_RX_EMPTY;
}

/**
 * @brief Flush data array in UART buffer, clear flags and
 * 		  set size variable to 0
 * @author LorisRT
 */
static void flushBuffer(void)
{
	while (buffer.size > 0U)
	{
		buffer.data[buffer.size--] = '\0';
	}
	buffer.flag_discard = 0U;
	buffer.flag_frame = 0U;
}

/**
 *
 */
static void buffer_memcpy(char *c, const volatile char *b, uint32_t size)
{
	while (size--)
	{
		*(c++) = *(b++);
	}
}

/**
 * @brief UART4 write string constant to Data Register (DR)
 * @author LorisRT
 */
STM_UART_e UART_writeBurst(const char *s)
{
	STM_UART_e uart_status = UART_OK;
	char temp_elem;
	uint32_t i_flag = 0;

	while ((temp_elem = *(s++)) != '\0')
	{
		UART4->DR = temp_elem;
		while ((UART4->SR & GET_UART_TXE_FLAG) == 0 && i_flag++ < UART_FLAG_LIM);

		if (i_flag >= UART_FLAG_LIM)
		{
			uart_status = UART_ERROR_TX;
			break;
		}
	}

	return uart_status;

}

/**
 * @brief UART4 peripheral configuration according
 * 		  to STM32F4 user manual (RM0090)
 * @author LorisRT
 */
void configUART4(void)
{
	UART4->CR1 |= (1U << 13); /* Enable UART before configuration */
	UART4->CR1 &= ~(1U << 12); /* Word length: 1 Start bit, 8 Data bits, n Stop bit */
	UART4->CR2 &= ~(0x3U << 12); /* Stop bit configuration: 1 stop bit */
	UART4->CR1 &= ~(1U << 15); /* Oversampling configuration: 16 */

	/* UART Baud Rate (BR) configuration: 115200 bits/s */
	UART4->BRR &= ~(0xffffU << 0);
	UART4->BRR |= (0x0046U << 0); /* 4.3125 \approx 4.3403 = (8e6)/(115200 * 16) */

	UART4->CR1 |= (1U << 3); /* Enable transmission */
	UART4->CR1 |= (1U << 2); /* Enable reception */

	/* Initialise static buffer for byte reception */
	buffer.flag_discard = 0;
	buffer.flag_frame = 0;
	buffer.size = 0;
	while (buffer.size < UART_BUFFER_SIZE)
	{
		buffer.data[buffer.size++] = '\0';
	}
	buffer.size = 0;

	/* Enable interrupt for UART peripherals */
	UART4->CR1 |= (1U << 5); /* IRQ enable peripheral side: control bit */
	*ptr_NVIC_ISR1 |= (1U << (IRQ_NUMBER_UART4 - 32)); /* IRQ enable CPU side: NVIC */
}


/**
 * @brief Enable Clock (CLK) for GPIOC and UART4
 * @author LorisRT
 */
void configPeripheralClock(void)
{
	RCC->AHB1ENR |= (1U << 2); /* Enable GPIOC peripheral CLK */
	RCC->APB1ENR |= (1U << 19); /* Enable UART4 peripheral CLK */
}


/**
 * @brief Configure GPIO C pin 10 and 11 as AF for UART4
 * @author LorisRT
 */
void configGPIO4UART(void)
{
	/* Configure GPIO C mode as Alternate Function (AF) */
	GPIO_C->MODER &= ~((0x3U << 20) | (0x3U << 22));
	GPIO_C->MODER |= ((0x2U << 20) | (0x2U << 22));

	/* Configure AF as UART */
	GPIO_C->AFRH &= ~((0xfU << 8) | (0xfU << 12));
	GPIO_C->AFRH |= ((8U << 8) | (8U << 12));
}


/**
 * @brief Enable HSE (8MHz) CPU Clock from X2 External Oscillator
 * @author LorisRT
 */
STM_STATUS_e enableClock(void)
{
	uint32_t i = 0;

	/* Enable HSE ON */
	RCC->CR |= (1U << 16);
	while ((RCC->CR & GET_RCC_HSERDY_FLAG) == 0 && (i++ < HSE_FLAG_LIM));
	if (i >= HSE_FLAG_LIM)
	{
		return STM_INIT_ERROR;
	}

	/* System Clock Switch configuration for HSE */
	RCC->CFGR &= ~(0x3U << 0);
	RCC->CFGR |= (0x1U << 0);

	/* HSE Set */
	return STM_OK;
}
