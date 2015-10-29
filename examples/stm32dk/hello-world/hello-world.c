#include "contiki.h"
#include <stdio.h>

PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);

PROCESS_THREAD(hello_world_process, ev, data)
{
    static struct etimer timer;
    
    PROCESS_BEGIN();

    printf("Hello, World\r\n");

    etimer_set(&timer, CLOCK_CONF_SECOND / 2);
  
    for (;;) 
    {
        // 等待定时器事件
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        printf("A");
        
        // 重置定时器
        etimer_reset(&timer);
    }
  
    PROCESS_END();
}
