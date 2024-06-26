#include <SPI.h>          //SPI is used to talk to the CAN Controller
#include <mcp_can.h>

MCP_CAN CAN(10);          //set SPI Chip Select to pin 10

//Variables for button presses
unsigned char pins[4] = {3, 4, 5, 6};
unsigned char buf[8];
boolean flag1 = false;
boolean flag2 = false;
boolean flag3 = false;
boolean flag4 = false;
long lastdebouncetime = 0;
long debouncedelay = 300;
unsigned char buttonno[4] = {1, 2, 3, 4};
unsigned char butt = 0;
int inc = 0;

//CAN Message Lengths
unsigned int len2 = 3;
unsigned int len3 = 4;
unsigned int len4 = 3;
unsigned int len5 = 3;
unsigned int len6 = 4;
unsigned int len7 = 71;

//CAN Addresses and Data
unsigned int array_1[] = {0x423};
unsigned int array_2[] = {0x367, 0x394, 0x3d3};
unsigned int array_3[] = {/*0x228, */0x351, 0x3bb, 0xba};
unsigned int array_4[] = {0x262, 0x2e4, 0x3e6};
unsigned int array_5[] = {0x1aa, 0x384, 0x386,};
unsigned int array_6[] = {0x283, 0x365, 0x366, 0x3e7};
unsigned int array_7[] = {0x24, 0x25, /*0xaa, 0xb4, */0x1c4, 0x1d0,/* 0x1d2, 0x1d3, 0x223, 0x224,*/ 0x260, 0x2c1, 0x320, 0x343, 0x344, 0x380, 0x381, 0x389, 0x38f, 0x399, 0x3a5, 0x3b0, 0x3b1, 0x3b7,/* 0x3bc,*/ 0x3e8, 0x3e9, 0x3f9, 0x411, 0x412, 0x413, 0x414, 0x420, 0x45a, 0x489, 0x48a, 0x48b, 0x4ac, 0x4cb, 0x4d3, 0x4ff, 0x610, 0x611, 0x614, 0x615, 0x619, 0x61a, 0x620, 0x621, 0x622, 0x623, 0x624, 0x638, 0x63c, 0x63d, 0x640, 0x680, 0x6f3, 0x770, 0x778, 0x7c6, 0x7ce, 0x7e0, 0x7e1, 0x7e2, 0x7e3, 0x7e4, 0x7e5, 0x7e6, 0x7e7, 0x7e8};
//0x1d2 byte 7, 1st 4 bits(!=0) = cruise_state 0x1d3 byte 2, bit 1 (1) = main_on
//0xaa = wheel speed
//0x3bc = GEAR_PACKET
//0xb4 = SPEED

unsigned char data1[1] = {0x00};
unsigned char data2[2] = {0x0, 0x0};
unsigned char data3[4] = {0x0, 0x0, 0x0, 0x0};
unsigned char data4[5] = {0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char data5[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char data6[7] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char data7[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

unsigned char enable1[] = {0xf8, 0x24, 0x02, 0xf8, 0x00, 0x01, 0x80, 0x72};
unsigned char enable2[] = {0x00, 0xa8, 0x43, 0x10, 0xee, 0x00, 0x00, 0xc5};
unsigned char not_in_d[] = {0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};





void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("beginning...");

  // Setup Pins
  for (inc = 0; inc <= 4; inc++) {
    pinMode(pins[inc], INPUT);
  }
  // tries to initialize, if failed --> it will loop here for ever
START_INIT:

  //if (CAN_OK == CAN.begin(CAN_500KBPS)) {     // setting CAN baud rate to 500Kbps
  if(CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) { // library needed more arguments
    Serial.println("CAN BUS Shield init ok!");
  } else {
    goto START_INIT;
  }
  CAN.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}

void loop()
{


  //Temporary ints
  unsigned int temp1 = 0;
  unsigned int temp2 = 0;
  unsigned int temp3 = 0;
  unsigned int temp4 = 0;
  unsigned int temp5 = 0;
  unsigned int temp6 = 0;
  unsigned int temp7 = 0;


  unsigned int spd = analogRead(A0) * 18;
  uint8_t w2 = spd & 0xff;
  uint8_t w1 = (spd >> 8);
  unsigned char wheelpot[] = {w1, w2, w1, w2, w1, w2, w1, w2};
  unsigned char speedpak[] = {0x0, 0x0, 0x0, 0x0, w1, w2, 0x0};


  //Send Pot as Wheelspeed

  CAN.sendMsgBuf(0x423, 0, 1, data1);
  for (int i = 0; i < len2; i++)
  {
    delay(100);
    temp2 = array_2[i];
    CAN.sendMsgBuf(temp2, 0, 2, data2);
  }

  for (int i = 0; i < len3; i++)
  {
    delay(50);
    temp3 = array_3[i];
    CAN.sendMsgBuf(temp3, 0, 4, data3);
  }

  for (int i = 0; i < len4; i++)
  {
    delay(30);
    temp4 = array_4[i];
    CAN.sendMsgBuf(temp4, 0, 5, data4);
  }

  for (int i = 0; i < len5; i++)
  {
    delay(20);
    temp5 = array_5[i];
    CAN.sendMsgBuf(temp5, 0, 6, data5);
  }

  for (int i = 0; i < len6; i++)
  {
    delay(10);
    temp6 = array_6[i];
    CAN.sendMsgBuf(temp6, 0, 7, data6);
  }

  for (int i = 0; i < len7; i++)
  {
    temp7 = array_7[i];
    CAN.sendMsgBuf(temp7, 0, 8, data7);
    CAN.sendMsgBuf(0xaa, 0, 8, wheelpot);
    CAN.sendMsgBuf(0xb4, 0, 8, speedpak);
  }

  //Send data if button was pushed
  for (inc = 0; inc <= 3; inc++) {
    if ((millis() - lastdebouncetime) > debouncedelay) {
      if (digitalRead(pins[inc]) == HIGH) {
        lastdebouncetime = millis();
        butt = buttonno[inc];
        if (butt == 1) {
          flag1 = !flag1;
        }
        if (butt == 2) {
          flag2 = !flag2;
        }
        if (butt == 3) {
          flag3 = !flag3;
        }
        if (butt == 4) {
          flag4 = !flag4;
        }
      }
      //Send default values if button off
      else {
        butt == 0;
      }
    }
  }
  if (flag1 == true) {
    CAN.sendMsgBuf(0x1d1, 0, 8, enable1);
    Serial.println("enabled!");
  }
  else {
    CAN.sendMsgBuf(0x1d1, 0, 8, data7);
  }
  if (flag2 == true && flag1 == true) {
    CAN.sendMsgBuf(0x1d2, 0, 8, enable2);
    Serial.println("engaged!");
  }
  else {
    CAN.sendMsgBuf(0x1d2, 0, 8, data7);
  }
  if (flag3 == true) {
    CAN.sendMsgBuf(0x3bc, 0, 8, data7);
    Serial.println("in drive");
  }
  else  {
    CAN.sendMsgBuf(0x3bc, 0, 8, not_in_d);
  }
}


