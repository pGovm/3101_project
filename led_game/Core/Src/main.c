#include "stm32f091xc.h"
#include <stdint.h>

//Setting up quick macro functions
#define CLEAR(n)  ()
#define SET(n)    ()

#define PX0       0
#define PX1       1
#define PX4       4
#define PX5       5

int main(void) {
  //Enabling GPIO clocks for Ports A and B
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

  //Enabling SYSCFG clock
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

  //Initializing a couple variables
  uint16_t green = 2*PX0;
  uint16_t red = 2*PX1;
  uint16_t blue = 2*PX4;
  uint16_t yellow = 2*PX5;

  //Configuring PA0,1,4, and 5 as outputs(LED)
  GPIOA->MODER &= ~( 3u << (green) | 3u << (red) | 3u << (blue) | 3u << (yellow) ); // clear MODER
  GPIOA->MODER |=  ( 1u << (green) | 1u << (red) | 1u << (blue) | 1u << (yellow) ); // set MODER --> 01

  //Configuring PB0,1,4, and 5 as inputs(Buttons) with internal pull-ups
  GPIOB->MODER &= ~( 3u << (green) | 3u << (2*PX1) | 3u << (2*PX4) | 3u << (2*PX5) ); // clear MODER and set
  GPIOB->MODER &= ~( 3u << (green) | 3u << (red) | 3u << (2*PX4) | 3u << (2*PX5) ); // clear PUPDR
  GPIOA->PUPDR |=  ( 1u << (green) | 1u << (red) | 1u << (blue) | 1u << (yellow) ); // set PUPDR --> 01
}