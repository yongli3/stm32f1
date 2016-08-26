
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "contiki-conf.h"
#include "core/dev/serial-line.h"
#include "dev/cc2520/cc2520.h"
#include "net/ipv6/uip-ds6.h"

#include "apps/serial-shell/serial-shell.h"

#include "stm32f10x.h"
#include "contiki.h"
#include "sys/autostart.h"

#include "uart-debug.h"

extern int (*uart1_input_handler)(unsigned char c);

static void platform_init();

int main() 
{
	platform_init();
	printf("Hello Contiki\r\n");

    // LED ON
    GPIO_ResetBits(GPIOA,GPIO_Pin_8);
    GPIO_ResetBits(GPIOD,GPIO_Pin_2);

	process_init();
	process_start(&etimer_process, NULL);


    // Add shell commands
    uart1_input_handler = serial_line_input_byte;
    serial_line_init();
    serial_shell_init();
    shell_ps_init();
    shell_ping_init();
    shell_time_init();
    //shell_blink_init();
    //shell_vars_init();

	ctimer_init();

	//autostart_start(autostart_processes);

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
    uart2_init();
    led_init();
    clock_init();
    cc2520_init();
	rtimer_init();
}
