#include <stdio.h>
#include "contiki-net.h"

#include "shell.h"

static struct psock ps;
static uint8_t buffer[100];

PROCESS(tcp_client_process, "Example protosocket client");

SHELL_COMMAND(tcptest_command,
	      "tcptest",
	      "tcptest ",
	      &tcp_client_process);

//AUTOSTART_PROCESSES(&example_psock_client_process);

/*---------------------------------------------------------------------------*/
static int
handle_connection(struct psock *p)
{
  PSOCK_BEGIN(p);

  PSOCK_SEND_STR(p, "GET / HTTP/1.0\r\n");
  //PSOCK_SEND_STR(p, "Server: Contiki example protosocket client\r\n");
  //PSOCK_SEND_STR(p, "\r\n");

  //while(1) 
  {
    //PSOCK_READTO(p, '\n');
    PSOCK_READBUF_LEN(p, 3);
    printf("Got: [%s]\n", buffer);
  }
  
  PSOCK_END(p);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tcp_client_process, ev, data)
{
  uip_ipaddr_t addr;

  PROCESS_BEGIN();


    uiplib_ipaddrconv("fe80:0000:0000:0000:5413:aca7:a6ee:6ab5", &addr);

      
  tcp_connect(&addr, UIP_HTONS(1234), NULL);

  printf("Connecting...\n");
  PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);

  if(uip_aborted() || uip_timedout() || uip_closed()) {
    printf("Could not establish connection\n");
  } else if(uip_connected()) {
    printf("Connected\n");
    
    PSOCK_INIT(&ps, buffer, sizeof(buffer));

    do {
      handle_connection(&ps);
      PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    } while(!(uip_closed() || uip_aborted() || uip_timedout()));

    printf("\nConnection closed.\n");
  }
  PROCESS_END();
}

void tcp_test_init()
{
    shell_register_command(&tcptest_command);
}
/*---------------------------------------------------------------------------*/
