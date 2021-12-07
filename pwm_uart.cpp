#include "pwm_uart.hpp"
#include "tobu.hpp"
float shigma_e,shiguma_a,shiguma_r;
static int chars_rxed = 0;
static int data_num=0;
uint8_t sbus_data[25];
uint8_t ch =0;
uint slice_num[3];
uint16_t Olddata[6];
uint16_t Chdata[19];
float Data1=0.0,Data2=0.0,Data3=0.0,Data4=0.0,Data5=0.0,Data6=0.0;

//関数の宣言
//uint8_t serial_settei(void);
//uint8_t pwm_settei();
void on_uart_rx();

/* Private variables ---------------------------------------------------------*/



/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */

//シリアル設定
uint8_t serial_settei(void){
  // Set up our UART with a basic baud rate.
  uart_init(UART_ID, 2400);

  // Set the TX and RX pins by using the function select on the GPIO
  // Set dat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asheet for more information on function select
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  // Actually, we want a different speed
  // The call will return the actual baud rate selected, which will be as close as
  // possible to that requested
  int actual = uart_set_baudrate(UART_ID, BAUD_RATE);

  // Set UART flow control CTS/RTS, we don't want these, so turn them off
  uart_set_hw_flow(UART_ID, false, false);

  // Set our data format
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

  // Turn off FIFO's - we want to do this character by character
  uart_set_fifo_enabled(UART_ID, true);

  // Set up a RX interrupt
  // We need to set up the handler first
  // Select correct interrupt for the UART we are using
  int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

  // And set up and enable the interrupt handlers
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);

  // Now enable the UART to send interrupts - RX only
  uart_set_irq_enables(UART_ID, true, false);
  return 0;
}
uint8_t pwm_settei(void){
  //pwmの設定
  // Tell GPIO 0 and 1 they are allocated to the PWM
  gpio_set_function(2, GPIO_FUNC_PWM);
  gpio_set_function(3, GPIO_FUNC_PWM);
  gpio_set_function(4, GPIO_FUNC_PWM);
  gpio_set_function(5, GPIO_FUNC_PWM);
  gpio_set_function(6, GPIO_FUNC_PWM);
  // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
  slice_num[0] = pwm_gpio_to_slice_num(3);
  slice_num[1] = pwm_gpio_to_slice_num(4);
  slice_num[2] = pwm_gpio_to_slice_num(6);

  // Set period
  pwm_set_wrap(slice_num[0], 3124);
  pwm_set_clkdiv(slice_num[0], 100.0);
  pwm_set_wrap(slice_num[1], 3124);
  pwm_set_clkdiv(slice_num[1], 100.0);
  pwm_set_wrap(slice_num[2], 3124);
  pwm_set_clkdiv(slice_num[2], 100.0);

  // Set channel A output high for one cycle before dropping
  pwm_set_chan_level(slice_num[0], PWM_CHAN_A, DUTYMAX);
  pwm_set_chan_level(slice_num[0], PWM_CHAN_B, DUTYMAX);
  pwm_set_chan_level(slice_num[1], PWM_CHAN_A, DUTYMAX);
  pwm_set_chan_level(slice_num[1], PWM_CHAN_B, DUTYMAX);
  // Set the PWM running
  pwm_set_enabled(slice_num[0], true);
  pwm_set_enabled(slice_num[1], true);
  pwm_set_enabled(slice_num[2], true);
  /// \end::setup_pwm[]
  sleep_ms(4000);
  pwm_set_chan_level(slice_num[0], PWM_CHAN_A, DUTYMIN);
  pwm_set_chan_level(slice_num[0], PWM_CHAN_B, DUTYMIN);
  pwm_set_chan_level(slice_num[1], PWM_CHAN_A, DUTYMIN);
  pwm_set_chan_level(slice_num[1], PWM_CHAN_B, DUTYMIN);
  pwm_clear_irq(slice_num[1]);
  pwm_set_irq_enabled(slice_num[1], true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP,MAINLOOP);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  return 0;
}
// RX interrupt handler
void on_uart_rx(void) {
  //short data;

  //gpio_put(25, 1);

  while (uart_is_readable(UART_ID)) {
    ch = uart_getc(UART_ID);
    if(ch==0x0f&&chars_rxed==0){
      sbus_data[chars_rxed]=ch;
      //printf("%02X ",ch);
      chars_rxed++;
    }
    else if(chars_rxed>0){
      sbus_data[chars_rxed]=ch;
      //printf("%02X ",ch);
      chars_rxed++;            
    }

    switch(chars_rxed){
      case 3:
        Olddata[0]=(sbus_data[1]|(sbus_data[2]<<8)&0x07ff);
        Data1=(float)(Olddata[0]-CH1MID)/((CH1MAX-CH1MIN)*0.5);
        //printf("%04d ",Olddata[0]);
        //printf("%04f ",Data1);
        break;
      case 4:
        Olddata[1]=(sbus_data[3]<<5|sbus_data[2]>>3)&0x07ff;	
        Data2=(float)(Olddata[1]-CH2MID)/((CH2MAX-CH2MIN)*0.5);
        //printf("%04d ",Olddata[1]);
        //printf("%04f ",Data2);
        break;
      case 6:
        Olddata[2]=(sbus_data[3]>>6|sbus_data[4]<<2|sbus_data[5]<<10)&0x07ff;
        Data3=(float)(Olddata[2]-CH3MIN)/(CH3MAX-CH3MIN);
        //printf("%04d ",Olddata[2]);
        // printf("%04f ",Data3);
        break;
      case 7:
        Olddata[3]=(sbus_data[6]<<7|sbus_data[5]>>1)&0x07ff;
        Data4=(float)(Olddata[3]-CH4MID)/((CH4MAX-CH4MIN)*0.5);
        //printf("%04d ",Olddata[3]);
        //printf("%04f ",Data4);
        break;
      case 8:
        Olddata[4]=(sbus_data[7]<<4|sbus_data[6]>>4)&0x07ff;
        Data5=(float)(Olddata[4]-CH5MID)/((CH5MAX-CH5MIN)*0.5);
        //printf("%04d ",Olddata[4]);
        //printf("%04f ",Data5);
        break;
      case 10:
        Olddata[5]=(sbus_data[7]>>7|sbus_data[8]<<1|sbus_data[9]<<9)&0x07ff;
        Data6=(float)(Olddata[5]-CH6MID)/((CH6MAX-CH6MIN)*0.5);
        //printf("%04d ",Olddata[5]);
        //printf("%04f ",Data6);
        break;
      case 11:
        Chdata[6]  = ((sbus_data[9]>>2|sbus_data[10]<<6) & 0x07FF);
        //printf("%04d ",Chdata[6]);
        break;
      case 12:
        Chdata[7]  = ((sbus_data[10]>>5|sbus_data[11]<<3) & 0x07FF);
        //printf("%04d ",Chdata[7]);
        break;
      case 14:
        Chdata[8]  = ((sbus_data[12]|sbus_data[13]<< 8) & 0x07FF);
        //printf("%04d ",Chdata[8]);
        break;
      case 15:
        Chdata[9]  = ((sbus_data[13]>>3|sbus_data[14]<<5) & 0x07FF);
        //printf("%04d ",Chdata[9]);
        break;
      case 16:
        Chdata[10] = ((sbus_data[14]>>6|sbus_data[15]<<2|sbus_data[16]<<10) & 0x07FF);
        //printf("%04d ",Chdata[10]);
        break;
      case 17:
        Chdata[11] = ((sbus_data[16]>>1|sbus_data[17]<<7) & 0x07FF);
        //printf("%04d ",Chdata[11]);
        break;
      case 19:
        Chdata[12] = ((sbus_data[17]>>4|sbus_data[18]<<4) & 0x07FF);
        //printf("%04d ",Chdata[12]);
        break;
      case 21:
        Chdata[13] = ((sbus_data[18]>>7|sbus_data[19]<<1|sbus_data[20]<<9) & 0x07FF);
        //printf("%04d ",Chdata[13]);
        break;
      case 22:
        Chdata[14] = ((sbus_data[20]>>2|sbus_data[21]<<6) & 0x07FF);
        //printf("%04d ",Chdata[14]);
        break;
      case 23:
        Chdata[15] = ((sbus_data[21]>>5|sbus_data[22]<<3) & 0x07FF);
        //printf("%04d ",Chdata[15]);
        break;
      case 24:
        Chdata[16] = sbus_data[23];
        //printf("%04x ",Chdata[16]);
        break;
    }

    if(chars_rxed==25){
      Chdata[17]=sbus_data[24];
      //printf("%04d ",Chdata[17]);
      //printf("\n");
      chars_rxed=0;
    }
  }
}
