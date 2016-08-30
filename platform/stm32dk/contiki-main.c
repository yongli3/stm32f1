
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

    cc2520_init();

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

    {
        uint8_t longaddr[8];
        uint16_t shortaddr;
    
        shortaddr = (linkaddr_node_addr.u8[0] << 8) +
          linkaddr_node_addr.u8[1];
        memset(longaddr, 0, sizeof(longaddr));
        linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);
    
        printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",
               longaddr[0], longaddr[1], longaddr[2], longaddr[3],
               longaddr[4], longaddr[5], longaddr[6], longaddr[7]);
    
        cc2520_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr);
      }
      cc2520_set_channel(RF_CHANNEL);
    
      printf(CONTIKI_VERSION_STRING " started. ");
#if 0
      if(node_id > 0) {
        printf("Node id is set to %u.\n", node_id);
      } else {
        printf("Node id is not set.\n");
      }    
#endif
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
	rtimer_init();    
}
