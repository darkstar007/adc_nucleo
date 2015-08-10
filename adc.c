/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

volatile int32_t total = 0;
volatile int32_t count = 0;

void tim1_cc_isr(void)
{
     static int32_t last_value = 0;
     int32_t value;
     if (timer_get_flag(TIM1, TIM_SR_CC1IF) != 0) {
	  // Timer channel 1 interrupt -> First edge (outcoming signal).
	  timer_clear_flag(TIM1, TIM_SR_CC1IF);
	  //value = timer_get_counter(TIM1);
	  
	  //if (count) {
	  //     total += (value - last_value);
	  //}
	  count++;
	  //last_value = value;
	  
     }
}



static void tim1_setup_input_capture(void)
{
     // Enable clock for Timer 1.
     rcc_periph_clock_enable(RCC_TIM1);
     
     // Configure TIM1_CH1  as inputs.
     gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);

     gpio_set_af(GPIOA, GPIO_AF6, GPIO8);
     
     // Enable interrupts for TIM1 CC.	
     nvic_enable_irq        (NVIC_TIM1_CC_IRQ);
     
     timer_set_mode(TIM1,
		    TIM_CR1_CKD_CK_INT, // Internal 72 MHz clock
		    TIM_CR1_CMS_EDGE,   // Edge synchronization
		    TIM_CR1_DIR_UP);    // Upward counter
     
     timer_set_prescaler     (TIM1, 1);  // Counter unit = 1 us.
     timer_set_period        (TIM1, 0xFFFF);
     timer_set_repetition_counter(TIM1, 0);
     timer_continuous_mode   (TIM1);
     
     // Configure channel 1
     timer_ic_set_input     (TIM1, TIM_IC1, TIM_IC_IN_TI1);
     timer_ic_set_filter    (TIM1, TIM_IC1, TIM_IC_OFF);
     timer_ic_set_polarity  (TIM1, TIM_IC1, TIM_IC_RISING);
     timer_ic_set_prescaler (TIM1, TIM_IC1, TIM_IC_PSC_OFF);
     timer_ic_enable        (TIM1, TIM_IC1);
     timer_clear_flag       (TIM1, TIM_SR_CC1IF);
     timer_enable_irq       (TIM1, TIM_DIER_CC1IE);
     
     timer_enable_counter   (TIM1);

}

static void adc_setup(void)
{
     //ADC
     rcc_periph_clock_enable(RCC_ADC12);
     
     adc_off(ADC1);
     adc_set_clk_prescale(ADC_CCR_CKMODE_DIV2);
     adc_set_single_conversion_mode(ADC1);
     adc_disable_external_trigger_regular(ADC1);
     adc_set_right_aligned(ADC1);
     /* We want to read the temperature sensor, so we have to enable it. */
     adc_enable_temperature_sensor();
     adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR1_SMP_61DOT5CYC);
     uint8_t channel_array[16];
     channel_array[0]=16; // Vts (Internal temperature sensor
     channel_array[0]=1; //ADC1_IN1 (PA0)
     adc_set_regular_sequence(ADC1, 1, channel_array);
     adc_set_resolution(ADC1, ADC_CFGR_RES_12_BIT);
     adc_power_on(ADC1);
     
     /* Wait for ADC starting up. */
     int i;
     for (i = 0; i < 800000; i++)
	  __asm__("nop");

}

static void usart_setup(void)
{
     /* Enable GPIOD clock for LED & USARTs. */
     rcc_periph_clock_enable(RCC_GPIOA);
     
     /* Enable clocks for USART2. */
     rcc_periph_clock_enable(RCC_USART2);
     
     /* Setup GPIO pins for USART2 transmit and receive. */
     gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
     
     /* Setup USART2 TX pin as alternate function. */
     gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
     
     /* Setup UART parameters. */
     usart_set_baudrate(USART2, 115200);
     usart_set_databits(USART2, 8);
     usart_set_stopbits(USART2, USART_STOPBITS_1);
     usart_set_mode(USART2, USART_MODE_TX_RX);
     usart_set_parity(USART2, USART_PARITY_NONE);
     usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
     
     /* Finally enable the USART. */
     usart_enable(USART2);
}

static void gpio_setup(void)
{
     rcc_periph_clock_enable(RCC_GPIOA);
     
     gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
     gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
     
     
     gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
}

static void my_usart_print_int(uint32_t usart, int16_t value)
{
     int8_t i;
     int8_t nr_digits = 0;
     char buffer[25];
     
     if (value < 0) {
	  usart_send_blocking(usart, '-');
	  value = value * -1;
     }
     
     if (value == 0) {
	  usart_send_blocking(usart, '0');
     }
     
     while (value > 0) {
	  buffer[nr_digits++] = "0123456789"[value % 10];
	  value /= 10;
     }
     
     for (i = nr_digits-1; i >= 0; i--) {
	  usart_send_blocking(usart, buffer[i]);
     }
     
     usart_send_blocking(usart, '\r');
     usart_send_blocking(usart, '\n');
}

static void my_usart_print_vals(uint32_t usart, int32_t value)
{
     int8_t i;
     int8_t nr_digits = 0;
     char buffer[25];
     
     if (value < 0) {
	  usart_send_blocking(usart, '-');
	  value = value * -1;
     }
     
     if (value == 0) {
	  usart_send_blocking(usart, '0');
     }
     
     while (value > 0) {
	  buffer[nr_digits++] = "0123456789"[value % 10];
	  value /= 10;
     }
     
     for (i = nr_digits-1; i >= 0; i--) {
	  usart_send_blocking(usart, buffer[i]);
     }
}

static void clock_setup(void)
{
     rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);
}


int main(int argc, char *argv[])
{
     
     int i;
     
     //clock_setup();
     gpio_setup();
     adc_setup();
     usart_setup();
     tim1_setup_input_capture();
     
#ifdef ADC
     uint16_t temp = 0;	
     while (1) {
	  gpio_toggle(GPIOA, GPIO5);	/* LED on/off */
	  adc_start_conversion_regular(ADC1);
	  while (!(adc_eoc(ADC1)));
	  temp=adc_read_regular(ADC1);
	  //gpio_port_write(GPIOE, temp << 4);
	  my_usart_print_int(USART2, temp);
     }
#endif
     
     while (1) {
	  gpio_toggle(GPIOA, GPIO5);	/* LED on/off */
	  for (i = 0; i < 16000000; i++)
	       __asm__("nop");
	  my_usart_print_vals(USART2, total);
	  usart_send_blocking(USART2, ',');
	  my_usart_print_vals(USART2, count);
	  count=0;
	  //total=0;
	  usart_send_blocking(USART2, '\r');
	  usart_send_blocking(USART2, '\n');
     }
		
     return 0;
}

