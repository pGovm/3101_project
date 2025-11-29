#include "stm32f0xx.h"
#include <stdint.h>
#include <string.h>


//Initializing pin values for i/o
#define PX0       0
#define PX1       1
#define PX4       4
#define PX5       5

//Function to Initialize the UART
static void uart2_init(void) {
    // Enable GPIOA and USART2 clocks
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure PA2 as USART2_TX (AF1), PA3 as USART2_RX (AF1)
    GPIOA->MODER   &= ~((0x3u << (2u * 2u)) | (0x3u << (3u * 2u)));
    GPIOA->MODER   |=  ((0x2u << (2u * 2u)) | (0x2u << (3u * 2u)));
    GPIOA->AFR[0]  &= ~((0xFu << (2u * 4u)) | (0xFu << (3u * 4u)));
    GPIOA->AFR[0]  |=  ((0x1u << (2u * 4u)) | (0x1u << (3u * 4u)));

    // Configure USART2 registers
    USART2->CR1 = 0;             // Disable before configuration
    USART2->CR2 = 0;             // Default settings
    USART2->CR3 = 0;             // No flow control

    // Students will deduce baud rate and frame format from oscilloscope
    USART2->BRR = 0x0045;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;

    // Enable transmitter and USART
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

//Function to write to the UART
static void uart2_write(const char *data) {
    for(int i = 0; i < strlen(data); i++) {
      while (!(USART2->ISR & USART_ISR_TXE)) {
        // Wait until transmit data register is empty
      }

      USART2->TDR = data[i];
    
    
      while (!(USART2->ISR & USART_ISR_TC)) {
          // Wait for transmission complete
      }
    }
}

//Simple delay function between commands
static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 800; ++i) {
        __NOP();
    }
}

int main(void) {
  //Enabling GPIO clocks for Ports A and B
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

  //Enabling SYSCFG clock
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

  //Initializing uart
  uart2_init();

  //Initializing a couple variables
  uint16_t green = 2*PX0;
  uint16_t red = 2*PX1;
  uint16_t blue = 2*PX4;
  uint16_t yellow = 2*PX5;

  //Configuring PA0,1,4, and 5 as outputs(LED)
  GPIOA->MODER &= ~( 3u << (green) | 3u << (red) | 3u << (blue) | 3u << (yellow) ); // clear MODER
  GPIOA->MODER |=  ( 1u << (green) | 1u << (red) | 1u << (blue) | 1u << (yellow) ); // set MODER --> 01

  //Configuring PB0,1,4, and 5 as inputs(Buttons) with internal pull-ups
  GPIOB->MODER &= ~( 3u << (green) | 3u << (2*PX1) | 3u << (blue) | 3u << (yellow) ); // clear MODER and set
  GPIOB->MODER &= ~( 3u << (green) | 3u << (red) | 3u << (blue) | 3u << (yellow) ); // clear PUPDR
  GPIOB->PUPDR |=  ( 1u << (green) | 1u << (red) | 1u << (blue) | 1u << (yellow) ); // set PUPDR --> 01

  uart2_write("Press USER button (PC13) to start.");
  delay_ms(1000);
}