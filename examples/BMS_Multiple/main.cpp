#include <Arduino.h>
#include <bq769x0.h>
#include <Wire.h>
#include <registers.h>

#define BMS_ALERT_PIN 34     // attached to interrupt INT0
#define BMS_BOOT_PIN 18      // connected to TS1 input
#define BMS_I2C_ADDRESS 0x08

bq769x0 BMS[2] = {bq769x0(bq76940, BMS_I2C_ADDRESS, 7), bq769x0(bq76940, BMS_I2C_ADDRESS, 6)};
bool isFirstRun = true;
unsigned long currTime;
TwoWire wire = TwoWire(0);

int sda = 26;
int scl = 27;

void TCA9548A(uint8_t bus){
  wire.beginTransmission(0x70);  // TCA9548A address
  wire.write(1 << bus);          // send byte to select bus
  wire.endTransmission();
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  wire.setPins(sda,scl);
  wire.begin();
  for (int i = 0; i < 2; i ++)
  {
    BMS[i].setI2C(&wire);
  }
  
  Scanner();
  int err;
  int lastErr;
  for (int i = 0; i < 2; i ++)
  {
    err = BMS[i].begin(BMS_ALERT_PIN, BMS_BOOT_PIN, &TCA9548A);
    lastErr = err;
    err = lastErr | err;
  }  
  
  if (err)
  {
    Serial.println("BMS Init Error");
  }
  BMS[0].setCellConfiguration(BMS[0].CELL_15);
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

  for (int i = 0; i < 2; i++)
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

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Multiple BMS Example");
  if (isFirstRun)
  {
    for (int i = 0; i < 2; i ++)
    {
        BMS[i].update();
    }
    currTime = millis();
    isFirstRun = false;
  }

  // According to library, BMS.update() should be called once every 250ms
  if ((millis() - currTime) > 250 )
  {
    for (int i = 0; i < 2; i ++)
    {
        BMS[i].update();
    }
    currTime = millis();
  }
  for (int j = 0; j < 2; j ++)
  {
    Serial.println("BMS " + String(j+1));
    Serial.println("=======Voltage Measurement=========");
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

