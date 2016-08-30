#include "stm32f10x_it.h"

#include "contiki.h"
#include <stdio.h>

int (* uart1_input_handler)(unsigned char c) = NULL;

PROCESS(receive_process, "Receive Process");

AUTOSTART_PROCESSES(&receive_process);

static process_event_t event_data_ready;

PROCESS_THREAD(receive_process, ev, data)
{  
    PROCESS_BEGIN();

    for (;;) 
    {
        // wait for event
        PROCESS_WAIT_EVENT_UNTIL(ev == event_data_ready);
        
        printf("%d\r\n", *(int *)data);
    }
  
    PROCESS_END();
}

// redefine IRQ handler functions
void USART1_IRQHandler(void)
{
    static int counter = 0;
    uint8_t c = 0;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        c = USART_ReceiveData(USART1);

        // FIXME
        USART_SendData(USART1, c);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        
        if (NULL != uart1_input_handler) {
            uart1_input_handler(c);
        }
        //counter++;
        /// printf("%d\r\n",counter);
        //process_post(&receive_process, event_data_ready, (void*)&counter);        
    }
}

void USART2_IRQHandler(void)
{
    static unsigned char led = 1;
    uint8_t temp = 0;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        temp = USART_ReceiveData(USART2);
        printf("%x\r\n", temp);

        if (0x31 == temp) {
            led = !led;
            if (led) // LED ON
               GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);
            else  // LED OFF
               GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_SET);
        }
        if (0x30 == temp) {    
            USART_SendData(USART2, led?'1':'0');
            while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET );
        }
        //process_post(&receive_process, event_data_ready, (void*)&counter);        
    }
}

