// CANBed V1 basic CAN TX test
// Sends 0x7FF with incrementing counter at 1 Hz
// Try MCP_8MHZ vs MCP_16MHZ if nothing appears on the bus

#include <SPI.h>
#include <mcp_canbus.h>  // Longan Labs library (not Seeed mcp_can.h)

MCP_CAN CAN(17);  // CANBed V1 CS pin

void setup() {
  //serial.begin(115200);
  //while (!serial);

  //serial.println("CANBed TX test (Longan lib)");

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    //serial.println("CAN init fail, retrying...");
    delay(500);
  }
  //serial.println("CAN init ok");
}

uint8_t count = 0;

void loop() {
  uint8_t data[8] = {count, 0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0};
  byte r = CAN.sendMsgBuf(0x7FF, 0, 8, data);
  //serial.print(r == CAN_OK ? "TX ok " : "TX FAIL ");
  //serial.println(count);
  count++;
  delay(1000);
}
