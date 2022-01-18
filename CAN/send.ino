
#include <mcp_can.h>
#include <SPI.h>

const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); // Chip select

void setup() {
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN BUS Module failed to start");
        Serial.println(" Try again");
        delay(100);
    }
    Serial.println("CAN BUS Module started.");

    for (int i=0; i <256; i++){
      Serial.println("-----------------------------");
      int q = random(0,100);
      int ledState = 1;
      if (q < 40){ledState = 0;}
      // Place data to be sent in output buffer
      byte bufout[8] = {ledState, i, i+1, 255, 100, 200, random(0,255), 255-i};
  
      // Sending:  ID = 0x05, standard frame, data len = 8, data in bufout
      CAN.sendMsgBuf(0x05, 0, 8, bufout);
      Serial.print("Packet sequence # ");
      Serial.println(i);
      for(int i = 0; i<8; i++){
          Serial.print(bufout[i]);
          Serial.print("\t"); 
      }
      Serial.println();
      delay(100);       // slow things down
    }
}
void loop(){}
