/**
 * \file
 *         A very simple test application for the stm32_bv motes
 * \author
 *         Benjamin Vedder
 */



#include "contiki.h"
#include <stdio.h>

PROCESS(receive_process, "Receive Process");
PROCESS(send_process, "Send Process");

AUTOSTART_PROCESSES(&receive_process, &send_process);

static process_event_t event_data_ready;

PROCESS_THREAD(send_process, ev, data)
{
    static struct etimer timer;
    static int counter = 0;
    PROCESS_BEGIN();

    etimer_set(&timer, CLOCK_CONF_SECOND);
  
    for (;;) 
    {
        // 等待定时器事件，1S触发一次
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        
        // 发送时间，参数为counter
        counter++;
        process_post(&receive_process, event_data_ready, (void*)&counter);
        
        // 重置定时器
        etimer_reset(&timer);
    }
  
    PROCESS_END();
}

PROCESS_THREAD(receive_process, ev, data)
{  
    PROCESS_BEGIN();

    for (;;) 
    {
        // 等待消息
        PROCESS_WAIT_EVENT_UNTIL(ev == event_data_ready);
        
        printf("%d\r\n", *(int *)data);
    }
  
    PROCESS_END();
}
