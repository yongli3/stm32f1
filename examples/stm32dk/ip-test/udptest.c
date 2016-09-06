#include <stdio.h>
#include "contiki-net.h"

#include "shell.h"

#define SEND_INTERVAL		8 * CLOCK_SECOND
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

static uint8_t buffer[100];
static struct uip_udp_conn *l_conn;


PROCESS(udp_client_process, "Example protosocket client");

SHELL_COMMAND(udptest_command,
	      "udptest",
	      "udptest ",
	      &udp_client_process);

//AUTOSTART_PROCESSES(&example_psock_client_process);

/*---------------------------------------------------------------------------*/
static int
handle_connection()
{
    int len = 0;
    int i = 0;
    if (uip_newdata()) {
        len = uip_datalen();
        printf("receive %d bytes [", len);

        for (i = 0; i < len; i++) {
            printf("%c", ((unsigned char *)uip_appdata)[i]);    
        }
        printf("]\n");

        l_conn->rport = UIP_UDP_BUF->srcport;
        uip_ipaddr_copy(&l_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
        uip_udp_packet_send(l_conn, uip_appdata, len);
        printf("Sent %u bytes\n", len);
        /* Restore server connection to allow data from any node */
        uip_create_unspecified(&l_conn->ripaddr);
        l_conn->rport = 0;
        
        //udp_socket_connect(struct udp_socket * c,uip_ipaddr_t * remote_addr,uint16_t remote_port)
    }
    return 0;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
    static struct etimer et;   
  uip_ipaddr_t addr;

  PROCESS_BEGIN();


    uiplib_ipaddrconv("fe80:0000:0000:0000:5413:aca7:a6ee:6ab5", &addr);

  /* new connection with remote host */

#ifdef AXD
if(uip_udp_conn->lport != 0 &&
   UIP_UDP_BUF->destport == uip_udp_conn->lport &&
   (uip_udp_conn->rport == 0 ||
    UIP_UDP_BUF->srcport == uip_udp_conn->rport) &&
   (uip_is_addr_unspecified(&uip_udp_conn->ripaddr) ||
    uip_ipaddr_cmp(&UIP_IP_BUF->srcipaddr, &uip_udp_conn->ripaddr))) {
  goto udp_found;
}

#endif
  // set rport=0, local UDP server must use rport=0 
  l_conn = udp_new(NULL, 0, NULL);
  if(!l_conn) {
    printf("udp_new l_conn error.\n");
  }

  // bind to lport listen as a UDP server
  udp_bind(l_conn, UIP_HTONS(3412));

  printf("Link-Local connection with ");
  uip_debug_ipaddr_print(&l_conn->ripaddr);
  printf(" local/remote port %u/%u\n",
         UIP_HTONS(l_conn->lport), UIP_HTONS(l_conn->rport));

      
  printf("Connecting...\n");

  etimer_set(&et, SEND_INTERVAL);

  while(1) {
    PROCESS_YIELD();
    if(etimer_expired(&et)) {
      printf("timeout ...\n");
      break;
      //etimer_restart(&et);
    } else if(ev == tcpip_event) {
      handle_connection();
    }
  }

  PROCESS_END();
}

void udp_test_init()
{
    shell_register_command(&udptest_command);
}
/*---------------------------------------------------------------------------*/
