
#include <Arduino.h>
#include <limits.h>
#include "idx_command.h"
#include "DueTimer.h"

#include "CRC32.h"

int main(void) {
  struct command * msg;
  
  watchdogSetup();
  init(); // Initialize the board, probably
  delay(1);
  USBDevice.attach(); // Initialize the SerialUSB device. 

  //UART->UART_BRGR = 5; // 84000000 / 16 * x = BaudRate (write x into UART_BRGR)  5 -> 1050000
  Serial.begin(115200); // For debugging
  SerialUSB.begin(1050000); // For ros messages
 
  IDXCommandBuffer cbuf(SerialUSB);

  Serial.print("Starting command size = ");
  Serial.println(sizeof(struct command));
  int ticks = 0;
  

  for (;;) {
    //cbuf.run();

    while( cbuf.size() > 0 ){
      msg = cbuf.getMessage();
      Serial.print("Done ");Serial.println(msg->seq);
      cbuf.sendResponse(*msg, IDX_COMMAND_DONE );

      delete msg;
    }

    cbuf.run();
  
  }

  return 0;
}

