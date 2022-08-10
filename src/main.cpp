#include <Arduino.h>
#include <bq769x0.h>
#include <Wire.h>
#include <registers.h>
#include <FastLED.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#define BMS_ALERT_PIN 34     // attached to interrupt INT0
#define BMS_BOOT_PIN 18      // connected to TS1 input
#define BMS_I2C_ADDRESS 0x08

#define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
// #define BLYNK_TEMPLATE_ID   "YourTemplateID"


// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "sGZwg34WR9aDxtGkJDJz4adtpwAgfzpj";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "BOLT!-7157";
char pass[] = "sundaya2021";

bq769x0 BMS[2] = {bq769x0(bq76940, BMS_I2C_ADDRESS, 7), bq769x0(bq76940, BMS_I2C_ADDRESS, 6)};
size_t arrSize = sizeof(BMS)/sizeof(BMS[0]);

bool isFinish = false;
bool isFirstDataSent = false;
bool isFirstRun = true;
int state = 0;
unsigned long currTime;

int sda = 26;
int scl = 27;

String command;
TwoWire wire = TwoWire(0);

BlynkTimer timer;

// How many leds in your strip?
#define NUM_LEDS 8

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI
#define DATA_PIN 13
#define CLOCK_PIN 12

// Define the array of leds
CRGB leds[NUM_LEDS];

int cellBalAddr, cellBalBitPos, cellBalState;

void TCA9548A(uint8_t bus){
  wire.beginTransmission(0x70);  // TCA9548A address
  wire.write(1 << bus);          // send byte to select bus
  wire.endTransmission();
  // Serial.print(bus);
}

void dataToLed(CRGB leds[], int dataCell, size_t _arrSize)
{
  Serial.println("Converting Data to Led");
  int temp;
  for (int i = 0; i < _arrSize; i++)
  {
    temp = dataCell >> i;
    temp = temp & 0x01;
    if(temp)
    {
      leds[i] = CRGB::Blue;
    }
    else
    {
      leds[i] = CRGB::Black;
    }
  }
  FastLED.show();
}

bool serialRead()
{
  bool isReady = false;
  if (Serial.available()) 
  {
    char inByte = Serial.read();
    if (inByte != '\n')
    {
      command += inByte;
    }
    else
    {
      isReady = true;
    }
  }
  return isReady;
}

void Scanner ()
{
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  for (byte i = 8; i < 120; i++)
  {
    wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (wire.endTransmission () == 0)  // Receive 0 = success (ACK response)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
}

void cekBms()
{
  BMS[0].update();
  Serial.println("======Voltage Measurement========");
  Serial.println("Cell Configuration : " + String(BMS[0].getCellConfiguration()));
  Serial.println("TCA Channel : " + String(BMS[0].getTCAChannel()));
  for(int i = 1; i < 16; i++)
  {
    Serial.print("Cell Voltage " + String(i) + " : ");
    Serial.println(BMS[0].getCellVoltage(i));
  }

  Serial.print("Pack Voltage : ");
  Serial.println(BMS[0].getBatteryVoltage());
  Serial.println("=================================");

  for (int i = 0; i < 3; i++)
  {
    Serial.print("Temperature Cell " + String(i+1) + " = ");
    Serial.println(String(BMS[0].getTemperatureDegC(i+1)));
  }

  int data = BMS[0].readReg(SYS_STAT);
  BMS[0].writeReg(SYS_STAT, data);
  Serial.print("SYS_STAT = ");
  Serial.println(data, BIN);
}


void cekBms2()
{
  BMS[1].update();
  for(int i = 1; i < 16; i++)
  {
    Serial.print("Cell Voltage " + String(i) + " : ");
    Serial.println(BMS[1].getCellVoltage(i));
  }

  Serial.print("Pack Voltage : ");
  Serial.println(BMS[1].getBatteryVoltage());
  Serial.println("=================================");
  for (int i = 0; i < 3; i++)
  {
    Serial.print("Temperature Cell " + String(i+1) + " = ");
    Serial.println(String(BMS[1].getTemperatureDegC(i+1)));
  }
  int data = BMS[1].readReg(SYS_STAT);
  BMS[1].writeReg(SYS_STAT, data);
  Serial.print("SYS_STAT 2 = ");
  Serial.println(data, BIN);
}

void checkAllBMS(size_t _arrSize)
{
  for (int j = 0; j < _arrSize; j ++)
  {
    Serial.println("BMS " + String(j+1));
    Serial.println("=======Voltage Measurement=========");
    Serial.print("Device Status : ");
    if(BMS[j].isDeviceSleep())
    {
      Serial.println("Sleep");
    }
    else
    {
      Serial.println("Active");
    }
    Serial.println("Channel : " + String(BMS[j].getTCAChannel()));
    Serial.println("Cell Configuration : " + String(BMS[j].getCellConfiguration()) + " Cell(s)");
    for (int i = 1; i < 16; i ++)
    {
        Serial.print("Cell Voltage " + String(i) + " : ");
        Serial.println(BMS[j].getCellVoltage(i));
    }
    for (int i = 1; i < 4; i++)
    {
        Serial.print("Temperature Channel " + String(i) + " : ");
        Serial.print(BMS[j].getTemperatureDegC(i));
        Serial.println(" C");
        Serial.print("Temperature Channel " + String(i) + " : ");
        Serial.print(BMS[j].getTemperatureDegF(i));
        Serial.println(" F");
    }
    Serial.print("Pack Voltage : ");
    Serial.println(BMS[j].getBatteryVoltage());
    Serial.println("========End of Voltage Measurement========");
  }
}

void parsingString(String command, char delimiter)
{
  int delimiterIndex = command.indexOf(',');
  int secondDelimiterIndex = command.indexOf(',', delimiterIndex+1);
  int thirdDelimiterIndex = command.indexOf(',', secondDelimiterIndex+1);
  String temp = command.substring(delimiterIndex+1,secondDelimiterIndex);
  cellBalAddr = temp.toInt();
  temp = command.substring(secondDelimiterIndex+1, thirdDelimiterIndex);
  cellBalBitPos = temp.toInt();
  temp = command.substring(thirdDelimiterIndex+1);
  cellBalState = temp.toInt();
  if (cellBalState < 1)
  {
    cellBalState = 0;
  }
  else if (cellBalState > 1)
  {
    cellBalState = 1;
  }
}

void checkCommand(String command)
{
  Serial.println("Reading command..");
  if (command == "write1")
  {
    BMS[0].writeReg(CELLBAL1,1);
  }
  if (command == "write2")
  {
    BMS[1].writeReg(CELLBAL2,1);
  }
    if (command == "clear1")
  {
    BMS[0].writeReg(CELLBAL1,0);
  }
  if (command == "clear2")
  {
    BMS[1].writeReg(CELLBAL2,0);
  }
  if (command.indexOf("bal1") >= 0)
  {
    parsingString(command, ',');
    BMS[0].enableBalancingProtection();
    BMS[0].setBalanceSwitch(cellBalAddr, cellBalBitPos, cellBalState);
    // BMS[0].updateBalanceSwitches();
    Serial.println(BMS[0].getDataCell(cellBalAddr));
    // Test On WS2812 Led
    dataToLed(leds, BMS[0].getDataCell(cellBalAddr), 8);
  }
  if (command.indexOf("bal2") >= 0)
  {
    parsingString(command, ',');
    BMS[1].enableBalancingProtection();
    BMS[1].setBalanceSwitch(cellBalAddr, cellBalBitPos, cellBalState);
    // BMS[1].updateBalanceSwitches();
    Serial.println(BMS[1].getDataCell(cellBalAddr));
    // Test On WS2812 Led
    dataToLed(leds, BMS[1].getDataCell(cellBalAddr), 8);
  }

  if (command.indexOf("testbal") >= 0)
  {
    parsingString(command, ',');
    int testdata = 0b00000001;
    if(!BMS[0].testBalancing(cellBalAddr, cellBalBitPos, cellBalState,testdata))
    {
      Serial.println("Balancing Invalid");
    }
    else
    {
      Serial.println("Balancing Valid");
    }
  }
  if (command.indexOf("multibal") >= 0)
  {
    parsingString(command, ',');
    int data1 = 0b00010001;
    int data2 = 0b00000010;
    int data3 = 0b00010000;
    BMS[0].enableBalancingProtection();
    BMS[0].setBalanceSwitches(1,data1);
    BMS[0].setBalanceSwitches(2,data2);
    BMS[0].setBalanceSwitches(3,data3);
  }

  if (command.indexOf("setconfig") >= 0)
  {
    Serial.println("Change Cell Config");
    parsingString(command, ',');
    BMS[0].setCellConfiguration(cellBalAddr);
    BMS[1].setCellConfiguration(cellBalAddr);
    
  }

  if (command == "end")
  {
    for (int i = 0; i < arrSize; i++)
    {
      BMS[i].clearBalanceSwitches();
      BMS[i].updateBalanceSwitches();
    }
    
    for (int i = 1; i < 4; i++)
    {
      dataToLed(leds, BMS[0].getDataCell(i), 8);
    }
  }
  if (command == "scan")
  {
    for (int i = 0; i < 10; i++)
    {
      Scanner();
    }
  }
  if (command == "read")
  {
    for(int i = 0; i < arrSize; i++)
    {
      Serial.println("BMS " + String(i+1));
      Serial.println("Reading CELLBAL Register");
      Serial.print("CELLBAL1 = ");
      Serial.println(BMS[i].readReg(CELLBAL1), BIN);
      Serial.print("CELLBAL2 = ");
      Serial.println(BMS[i].readReg(CELLBAL2), BIN);
      Serial.print("CELLBAL3 = ");
      Serial.println(BMS[i].readReg(CELLBAL3), BIN);
    }
  }
  if (command == "shutdown1")
  {
    BMS[0].shutdown();
    Serial.println("BQ Shutdown");
  }
  if (command == "shutdown2")
  {
    BMS[1].shutdown();
    Serial.println("BQ Shutdown");
  }

  if (command == "wake1")
  {
    BMS[0].wake();
    Serial.println("BQ Wake Up");
  }
  if (command == "wake2")
  {
    BMS[1].wake();
    Serial.println("BQ Wake Up");
  }

  if (command == "cekbms")
  {
    cekBms();
  }
  if (command == "cekbms2")
  {
    cekBms2();
  }
  if (command == "checkall")
  {
    checkAllBMS(arrSize);
  }
}

void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  // Serial.println("My Timer Event");
  switch(state)
  {
    case 0:
      for (int i = 1; i < 11; i++)
      {
        Blynk.virtualWrite(i, BMS[0].getCellVoltage(i));
      }
      state++;
      // Serial.println("Case 0");
      break;
    case 1:
      for (int i = 11; i < 16; i++)
      {
        Blynk.virtualWrite(i, BMS[0].getCellVoltage(i));
      }
      Blynk.virtualWrite(16, BMS[0].getBatteryVoltage());
      state++;
      // Serial.println("Case 1");
      break;
    case 2:
      for (int i = 20; i < 30; i++)
      {
        Blynk.virtualWrite(i, BMS[1].getCellVoltage(i-19));
      }
      state++;
      // Serial.println("Case 2");
      break;
    case 3:
      for (int i = 30; i < 35; i++)
      {
        Blynk.virtualWrite(i, BMS[1].getCellVoltage(i-19));
      }
      Blynk.virtualWrite(35, BMS[1].getBatteryVoltage());
      state = 0;
      // Serial.println("Case 3");
      break;
  }
    

}

void setup() {
  // put your setup code here, to run once:
  Serial.println("BQ with TCA and Blynk Example");
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass, "iot.serangkota.go.id", 8080);
  timer.setInterval(1000L, myTimerEvent);
  wire.setPins(sda,scl);
  wire.begin();
  for (int i = 0; i < arrSize; i ++)
  {
    BMS[i].setI2C(&wire);
  }
  
  if (!isFinish)
  {
    Scanner();
  }

  int err;
  int lastErr;
  for (int i = 0; i < arrSize; i ++)
  {
    err = BMS[i].begin(BMS_ALERT_PIN, BMS_BOOT_PIN, &TCA9548A);
    lastErr = err;
    err = lastErr | err;
  }  
  
  if (err)
  {
    Serial.println("BMS Init Error");
  }

  BMS[0].setCellConfiguration(BMS[0].CELL_10);
  BMS[1].setCellConfiguration(BMS[1].CELL_10);

  // BMS.setTemperatureLimits(-20, 45, 0, 45);
  // BMS.setShuntResistorValue(5);
  // BMS.setShortCircuitProtection(14000, 200);  // delay in us
  // BMS.setOvercurrentChargeProtection(8000, 200);  // delay in ms
  // BMS.setOvercurrentDischargeProtection(8000, 320); // delay in ms
  // BMS.setCellUndervoltageProtection(2600, 2); // delay in s
  // BMS.setCellOvervoltageProtection(3650, 2);  // delay in s

  // BMS.setBalancingThresholds(0, 3300, 20);  // minIdleTime_min, minCellV_mV, maxVoltageDiff_mV
  // BMS.setIdleCurrentThreshold(100);
  // BMS.enableAutoBalancing();
  // BMS.enableDischarging();

  for (int i = 0; i < arrSize; i++)
  {
    Serial.println("BMS " + String(i+1) + " Configuration");
    int data = BMS[i].readReg(SYS_STAT);
    Serial.print("SYS_STAT " + String(i+1) + " : ");
    Serial.println(data, BIN);
    Serial.println("Clearing SYS_STAT " + String(i+1));
    BMS[i].writeReg(SYS_STAT, data);
    data = BMS[i].readReg(SYS_STAT);
    Serial.print("SYS_STAT " + String(i+1) + " : ");
    Serial.println(data, BIN);
  }

}

void test()
{
  Serial.println("test"); 
}


void loop() {

  // put your main code here, to run repeatedly:
  Blynk.run();
  timer.run();

  if (isFirstRun)
  {
    for (int i = 0; i < arrSize; i ++)
    {
        BMS[i].update();
    }
    currTime = millis();
    isFirstRun = false;
  }

  if ((millis() - currTime) > 250 )
  {
    for (int i = 0; i < arrSize; i ++)
    {
        BMS[i].update();
    }
    currTime = millis();
  }
  
  // Serial.println("======Voltage Measurement========");
  // Serial.println("Cell Configuration : " + String(BMS.getCellConfiguration()));
  // Serial.println("TCA Channel : " + String(BMS.getTCAChannel()));
  // for(int i = 1; i < 16; i++)
  // {
  //   Serial.print("Cell Voltage " + String(i) + " : ");
  //   Serial.println(BMS.getCellVoltage(i));
  // }
  // Serial.print("Pack Voltage : ");
  // Serial.println(BMS.getBatteryVoltage());
  // Serial.println("=================================");

  // Serial.println("======Voltage Measurement========");
  // Serial.println("Cell Configuration : " + String(BMS2.getCellConfiguration()));
  // Serial.println("TCA Channel : " + String(BMS2.getTCAChannel()));
  // for(int i = 1; i < 16; i++)
  // {
  //   Serial.print("Cell Voltage " + String(i) + " : ");
  //   Serial.println(BMS2.getCellVoltage(i));
  // }
  // Serial.print("Pack Voltage : ");
  // Serial.println(BMS2.getBatteryVoltage());
  // Serial.println("=================================");

  if (serialRead())
  {
    Serial.println(command);
    checkCommand(command);
    command = "";
  }
  // delay(250);
}

