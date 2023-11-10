#include <Arduino.h>
#include <FlexCAN_T4.h>

#define enablePPP 3
#define PPPenabled 5

#define PPPHigh 114 * 29.25
#define PPPLow 85 * 29.25

#define SyncByte 0xE6
#define SerialEnd 0x05

#define charToShort(high, low) ((uint16_t)high << 8 | low)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> ImaCan1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> FCan;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> ImaCan2;

uint8_t SerialStart;

uint8_t SerialRx[6];
uint8_t SerialTx[6];

uint8_t high, low;

uint8_t synced;

uint16_t VoltageOut;
uint16_t VoltageIn;
uint8_t CheckSum;

void CanPassthrough_1to2(const CAN_message_t &msg);
void CanPassthrough_2to1(const CAN_message_t &msg);
void CanEGas(const CAN_message_t &msg);

// put function declarations here:

void setup()
{
  // put your setup code here, to run once:
  synced = false;
  ImaCan1.begin();
  ImaCan2.begin();
  FCan.begin();

  Serial2.begin(20800,SERIAL_9E1);

  ImaCan1.setBaudRate(500 * 1000);
  ImaCan2.setBaudRate(500 * 1000);
  FCan.setBaudRate(1000 * 1000);

  ImaCan1.onReceive(CanPassthrough_1to2);
  ImaCan2.onReceive(CanPassthrough_2to1);
  FCan.onReceive(CanEGas);

  ImaCan1.enableMBInterrupts();
  ImaCan2.enableMBInterrupts();
  FCan.enableMBInterrupts();

  pinMode(PPPenabled, INPUT);
  pinMode(enablePPP, OUTPUT);
}

void loop()
{
  // put your main c
  ImaCan1.events();
  ImaCan2.events();
  FCan.events();

  if (Serial2.available())
  {
    if (!synced)
    {
      do
      {
        if (Serial2.read() == SyncByte)
          synced = true;
        Serial2.write(SerialTx[0]);
        SerialRx[0] = SyncByte;
        SerialStart = 1;
      } while (!synced);
    }
    else
      SerialStart = 0;


    for (uint8_t i = SerialStart; i < SerialEnd; i++)
    {
      SerialRx[i] = Serial2.read();
      Serial2.write(SerialTx[i]);
      SerialTx[i] = SerialRx[i];
    }

    synced++;
    
    if (synced > 4)
      synced = 0;

    if (digitalRead(PPPenabled))
    {
      VoltageOut = PPPLow;
    }
    else
    {
      high = SerialRx[1];
      low = SerialRx[2];

      low &= 0x70;

      if (charToShort(high, low) > PPPLow)
      {
        VoltageOut = PPPHigh;
      }
      else
      {
        VoltageOut = VoltageIn;
      }
    }

    SerialTx[1] = (VoltageOut >> 8) & 0xFF;
    SerialTx[2] = (VoltageOut)&0xFF;

    CheckSum = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
      CheckSum = CheckSum + SerialTx[i];
    }
    CheckSum = (~CheckSum) + 1;
    SerialTx[5] = CheckSum & 0x7f;
  }
}

void CanPassthrough_1to2(const CAN_message_t &msg)
{
  ImaCan2.write(msg);
}

void CanPassthrough_2to1(const CAN_message_t &msg)
{
  ImaCan1.write(msg);
}
void CanEGas(const CAN_message_t &msg)
{
  if (msg.id == 0x13a)
  {
    if (msg.buf[1] > 0x200)
      digitalWrite(enablePPP, 1);
    else
      digitalWrite(enablePPP, 0);
  }
}
