
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "stm32f10x.h"
#include "contiki.h"
#include "sys/autostart.h"

#include "uart-debug.h"
static void platform_init();

int main() 
{
	platform_init();
	printf("Hello Contiki\r\n");
	
	process_init();
	process_start(&etimer_process, NULL);

	ctimer_init();

	autostart_start(autostart_processes);

	for (;;) 
    {
		do 
        {

		} while(process_run() > 0);
	}
	
	return 0;
}

static void platform_init() 
{
	debug_init();
    clock_init();
	rtimer_init();
}
