#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Initializing pin values for i/o
#define PX0       0
#define PX1       1
#define PX4       4
#define PX5       5
#define PC13      13

//Initializing a couple variables
uint16_t green = 2*PX0;
uint16_t red = 2*PX1;
uint16_t blue = 2*PX4;
uint16_t yellow = 2*PX5;
uint16_t start_button = 2*PC13;

//Function to initialize the i/o
static void io_init(void) {
    //Configuring PA0,1,4, and 5 as outputs(LED)
    GPIOA->MODER &= ~( (3u << green) | (3u << red) | (3u << blue) | (3u << yellow) ); // clear MODER
    GPIOA->MODER |=  ( 1u << green | 1u << red | 1u << blue | 1u << yellow ); // set MODER --> 01

    //Configuring PB0,1,4, and 5 as inputs(Buttons) with internal pull-ups
    GPIOB->MODER &= ~( (3u << green) | (3u << red) | (3u << blue) | (3u << yellow) ); // clear MODER and set
    GPIOB->MODER &= ~( (3u << green) | (3u << red) | (3u << blue) | (3u << yellow) ); // clear PUPDR
    GPIOB->PUPDR |=  ( (1u << green) | (1u << red) | (1u << blue) | (1u << yellow) ); // set PUPDR --> 01

    //Configuring PC13 as the Start button with internal pull-down
    GPIOC->MODER &= ~( 3u << start_button ); // clear MODER
    GPIOC->PUPDR |= ( 2u << start_button ); // set PUPDR --> 10
}

// Function to Initialize the UART
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

    USART2->BRR = 0x0045;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;

    // Enable transmitter and USART
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

// Function to write to the UART
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

// Simple delay function between commands
static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 800; ++i) {
        __NOP();
    }
}

void EXTI2_3_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR3) {       // Check if EXTI line 3 triggered
        EXTI->PR |= EXTI_PR_PR3;        // Clear the pending flag
        GPIOA->ODR ^= (1 << 5);         // Toggle LED on PA5
        uart2_write("button 1 pushed\r\n");
        delay_ms(20);  
    }
}

int main(void) {
    //Enabling GPIO clocks for Ports A, B, and C
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    //Enabling SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    //Calling required intiializing functions
    io_init();
    uart2_init();

    // Connect PB4 to EXTI3
    SYSCFG->EXTICR[0] &= ~(0xF << (3 * 4));
    SYSCFG->EXTICR[0] |= (0x1 << (3 * 4));  // Port B = 0001

    // Unmask EXTI3
    EXTI->IMR |= (1 << 3);
    // Trigger on falling edge
    EXTI->FTSR |= (1 << 3);

    // Enable EXTI2_3 interrupt in NVIC
    NVIC_EnableIRQ(EXTI2_3_IRQn);


    uart2_write("Press USER button (PC13) to start.\r\n");
    delay_ms(100);

    while(1){
        
    }

    // Idle animations until user button is pressed
    while((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)) { 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            break;
        }
        delay_ms(50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            break;
        }
        delay_ms(50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            break;
        }
        delay_ms(50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            break;
        }
        delay_ms(50);
    }

    // Game start
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
    uart2_write("Starting game...\r\n");
    delay_ms(500);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

    // Generate the random sequence pattern
    srand(HAL_GetTick());
    uint8_t expectedResponse[128] = {0};
    for (int i = 0; i < sizeof(expectedResponse); i++) {
        int randomNumber = rand() % 4;
        expectedResponse[i] = randomNumber;
        if(i > 4 && expectedResponse[i] == expectedResponse[i-1] && expectedResponse[i-2] ) {
            expectedResponse[i] = (randomNumber + 1) % 4;
        }
    }

    // uint8_t receivedResponse[128] = {0};
    uint16_t roundNum = 0; //Counter for number of rounds elapsed
    while (1) {
        roundNum++;
        char buffer[50];
        sprintf(buffer, "New Round: length=%d\r\n", roundNum);
        uart2_write(buffer);

        for (int i = 0; i < roundNum; i++) {
            sprintf(buffer, "Num %d, Random num: %d\r\n", i, expectedResponse[i]);
            uart2_write(buffer);

            switch (expectedResponse[i]) {
              case 0:
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
                  break;
              case 1:
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
                  break;
              case 2:
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
                  break;
              case 3:
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
                  break;
            }
            
            delay_ms(100);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
            delay_ms(50);
        }
        if (roundNum >= sizeof(expectedResponse)) {
            uart2_write("Game over, you win!\r\n");
            break;
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
        delay_ms(1000);
    }
}