// SDL_Arduino_WeatherSenseThunderBoard
// SwitchDoc Labs January 2021
//



#define TXDEBUG
//#undef TXDEBUG
#include <JeeLib.h>

#include "MemoryFree.h"


// WeatherSenseProtocol of 8 is SolarMAX LiPo   BatV < 7V
// WeatherSenseProtocol of 10 is SolarMAX LeadAcid   BatV > 7V LoRa version
// WeatherSenseProtocol of 11 is SolarMAX4 LeadAcid BatV > 7V
// WeatherSenseProtocol of 15 is WeatherSense AQI 433MHz
// WeatherSenseProtocol of 16 is WeatherSense ThunderBoard 433MHz
// WeatherSenseProtocol of 17 is Generic data

#define WEATHERSENSEPROTOCOL 16

#define LED 13
// Software version
#define SOFTWAREVERSION 1

// unique ID of this WeatherSenseThunderBoard system - change if you have multiple WeatherSenseThunderBoard systems
#define WEATHERSENESTHBID 1
// Which WeatherSense ThunderBoard Protocol Version
#define WEATHERSENSEPROTOCOLVERSION 1

// ThunderBoard


// Device ID is changed if you have more than one WeatherSense ThunderBoard in the area
// #define WeatherSenseProtocol 2
// Number of milliseconds between wake up  30 seconds.  Every 5 wakeups send packet  - if you move this over 60000 ms, you will need to add the watchdog in the sleep loop - see SDL_Arduino_WeatherSenseAQI ResetWatchDog
#define SLEEPCYCLE 30000
//#define SLEEPCYCLE 14000

#include "Crc16.h"

//Crc 16 library (XModem)
Crc16 crc;

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}

#include <RH_ASK.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include "SDL_Arduino_INA3221.h"


SDL_Arduino_INA3221 INA3221;



// the three channels of the INA3221 named for INA3221 Solar Power Controller channels (www.switchdoc.com)
#define LIPO_BATTERY_CHANNEL 1
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3


// Other Pins
#define WATCHDOG_1 5

#define TXPIN 8
#define RXPIN 9



// Number of milliseconds between data ou






RH_ASK driver(2000, RXPIN, TXPIN);

unsigned long MessageCount = 0;



#include "avr/pgmspace.h"
#include <Time.h>
#include <TimeLib.h>


#include <Wire.h>

typedef enum  {

  NO_INTERRUPT,
  IGNORE_INTERRUPT,
  SLEEP_INTERRUPT,
  ALARM_INTERRUPT,
  REBOOT
} wakestate;


// Device Present State Variables

bool INA3221_Present;

bool AS3935_Present;

byte byteBuffer[200]; // contains string to be sent to RX unit

// State Variables

long TimeStamp;



// State Status

float BatteryVoltage;
float BatteryCurrent;
float LoadVoltage;
float LoadCurrent;
float SolarPanelVoltage;
float SolarPanelCurrent;
byte AuxA;

// AuxA has state information
// coded in the long integer
// 00000000 00000000 00000000 00000CAB

// A = 1, IN3221 (Solar) Present, 0 not present
// B = 1, AS3935 Present, 0 not present
// C = 1. Low battery, Lighting Chip shut off (too many false alarms in low voltage mode)


int protocolBufferCount;



wakestate wakeState;  // who woke us up?


long nextSleepLength;





int convert4ByteLongVariables(int bufferCount, long myVariable)
{

  int i;

  union {
    long a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];
    bufferCount++;
  }
  return bufferCount;

}

int convert4ByteFloatVariables(int bufferCount, float myVariable)
{
  int i;

  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = myVariable;

  for (i = 0; i < 4; i++)
  {
    byteBuffer[bufferCount] = thing.bytes[i];


    bufferCount++;
  }

  return bufferCount;
}


int convert2ByteVariables(int bufferCount, int myVariable)
{


  union {
    int a;
    unsigned char bytes[2];
  } thing;

  thing.a = myVariable;


  byteBuffer[bufferCount] = thing.bytes[0];
  bufferCount++;
  byteBuffer[bufferCount] = thing.bytes[1];
  bufferCount++;

  return bufferCount;

}

int convert1ByteVariables(int bufferCount, int myVariable)
{


  byteBuffer[bufferCount] = (byte) myVariable;
  bufferCount++;
  return bufferCount;

}

int checkSum(int bufferCount)
{
  unsigned short checksumValue;
  // calculate checksum
  checksumValue = crc.XModemCrc(byteBuffer, 0, 59);
#if defined(TXDEBUG)
  Serial.print(F("crc = 0x"));
  Serial.println(checksumValue, HEX);
#endif

  byteBuffer[bufferCount] = checksumValue >> 8;
  bufferCount++;
  byteBuffer[bufferCount] = checksumValue & 0xFF;
  bufferCount++;

  return bufferCount;
}




// variables

bool setIndoor = false;
bool disturbersEnabled = true;

byte LastInterruptResult = 0;
String LastResult;
String LastLightningResult = "";
long LightningCount = 0;
unsigned long LightningTimeStamp = 0;
long InterruptCount = 0;
byte LightningLastDistance = 0;
unsigned long InterruptTimeStamp = 0;
byte irqSource; // current interrupt







int buildProtocolMessage()
{

  int bufferCount;


  bufferCount = 0;

  bufferCount = convert4ByteLongVariables(bufferCount, MessageCount);


  byteBuffer[bufferCount] = WEATHERSENESTHBID; // WeatherSenseThunderBoard unique ID
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOL; // Type of WeatherSense System
  bufferCount++;
  byteBuffer[bufferCount] = WEATHERSENSEPROTOCOLVERSION; // WeatherSense ThunderBoard protocol version
  bufferCount++;

  // current interrupt source
  // last interrupt source
  // lightning count
  // last   lightning distance


  byteBuffer[bufferCount] = irqSource; // current interrupt read
  bufferCount++;
  byteBuffer[bufferCount] = LastInterruptResult; // last interrupt value > 0
  bufferCount++;
  byteBuffer[bufferCount] = LightningLastDistance;
  bufferCount++;
  byteBuffer[bufferCount] = 0; // Spare byte
  bufferCount++;
  bufferCount = convert4ByteLongVariables(bufferCount, LightningCount);
  bufferCount = convert4ByteLongVariables(bufferCount, InterruptCount);


  bufferCount = convert4ByteFloatVariables(bufferCount, LoadVoltage);  // Solar Data
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, BatteryCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, LoadCurrent);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelVoltage);
  bufferCount = convert4ByteFloatVariables(bufferCount, SolarPanelCurrent);



  byteBuffer[bufferCount] = AuxA | (SOFTWAREVERSION << 4); // Aux + Software Version
  bufferCount++;


  // protocolBufferCount = bufferCount + 2;
  //     bufferCount = convert1ByteVariables(bufferCount, protocolBufferCount);
  //  bufferCount = checkSum(bufferCount);

  return bufferCount;


}



void printStringBuffer()
{
  int bufferLength;

  bufferLength = protocolBufferCount;
  int i;
  for (i = 0; i < bufferLength; i++)
  {
    Serial.print(F("i="));
    Serial.print(i);
    Serial.print(F(" | "));
    Serial.println(byteBuffer[i], HEX);
  }

}




void return2Digits(char returnString[], char *buffer2, int digits)
{
  if (digits < 10)
    sprintf(returnString, "0%i", digits);
  else
    sprintf(returnString, "%i", digits);

  strcpy(returnString, buffer2);
}



void ResetWatchdog()
{


  digitalWrite(WATCHDOG_1, LOW);
  delay(200);
  digitalWrite(WATCHDOG_1, HIGH);

#if defined(TXDEBUG)
  Serial.println(F("Watchdog1 Reset - Patted the Dog"));
#endif

}



#define ENABLE 4
#define CONTROL 3


//
//
//

// I2c library by Wayne Truchsess
#include "I2C.h"
#include "SDL_Arduino_ThunderBoard_AS3935.h"





// Interrupt handler for AS3935 irqs
// and flag variable that indicates interrupt has been triggered
// Variables that get changed in interrupt routines need to be declared volatile
// otherwise compiler can optimize them away, assuming they never get changed
void AS3935Irq();
volatile int AS3935IrqTriggered;

// Library object initialization First argument is interrupt pin, second is device I2C address
// Note:   The SwitchDoc Labs Thunder Board has a fixed address of 0x020 for version 0240-092717-01 and a default address of 0x03 for any other version.




SDL_Arduino_ThunderBoard_AS3935 AS3935(2, 0x02);

void printAS3935Registers()
{
  int noiseFloor = AS3935.getNoiseFloor();
  int spikeRejection = AS3935.getSpikeRejection();
  int watchdogThreshold = AS3935.getWatchdogThreshold();
  Serial.print(F("Noise floor is: "));
  Serial.println(noiseFloor, DEC);
  Serial.print(F("Spike rejection is: "));
  Serial.println(spikeRejection, DEC);
  Serial.print(F("Watchdog threshold is: "));
  Serial.println(watchdogThreshold, DEC);
}


unsigned long wakeCount;

void setup()
{



  Serial.begin(115200);    // TXDEBUGging only
  // Pat the WatchDog
  ResetWatchdog();
  wakeCount = 0;

  AuxA = 0x00;
  // turn on USB Power for power check.

  Serial.println();
  Serial.println();
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.println(F("WeatherSense ThunderBoard"));
  Serial.println(F(">>>>>>>>>><<<<<<<<<"));
  Serial.print(F("Software Version:"));
  Serial.println(SOFTWAREVERSION);
  Serial.print(F("Unit ID:"));
  Serial.println(WEATHERSENESTHBID);

  if (!driver.init())
  {
    Serial.println(F("init failed"));
    while (1);
  }

  Serial.print("max message length=");
  Serial.println(driver.maxMessageLength());

  I2c.begin();
  I2c.pullup(true);
  I2c.setSpeed(1); //400kHz - 100kHz

  // reset all internal register values to defaults
  AS3935.reset();

  uint16_t chipNoiseFloor;  // for chip detection
  chipNoiseFloor = AS3935.getNoiseFloor();

  Serial.print("chipNoiseFloor=");
  Serial.println(chipNoiseFloor);


  if (chipNoiseFloor != 2) {
    Serial.println("AS3935 init failed!!!");
    Serial.println(F("AS3935 Not Present"));
    AS3935_Present = false;
  }
  else
  {
    AS3935_Present = true;

    // optional control of power for AS3935 via a PNP transistor
    // very useful for lockup prevention and power saving
    //pinMode(4, OUTPUT);
    //digitalWrite(4, LOW);

    // reset all internal register values to defaults
    AS3935.reset();

    // first let's turn on disturber indication and print some register values from AS3935
    // tell AS3935 we are indoors, for outdoors use setOutdoors() function
    //AS3935.setIndoors();
    AS3935.setOutdoors();
    // turn on indication of distrubers, once you have AS3935 all tuned, you can turn those off with disableDisturbers()
    AS3935.enableDisturbers();
    //AS3935.disableDisturbers();
    AS3935.setNoiseFloor(3);
    printAS3935Registers();
    Serial.println(F("AS3935 Present"));
    // State Variable
    AuxA = AuxA | 0X01;
  }







  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);

  // setup initial values of variables

  wakeState = REBOOT;

  nextSleepLength = SLEEPCYCLE;



  TimeStamp = 0;


  BatteryVoltage = 0.0;
  BatteryCurrent = 0.0;
  LoadCurrent = 0.0;
  SolarPanelVoltage = 0.0;
  SolarPanelCurrent = 0.0;







  pinMode(WATCHDOG_1, OUTPUT);
  digitalWrite(WATCHDOG_1, HIGH);


  //Wire.begin();




  // test for INA3221_Present
  INA3221_Present = false;



  int MIDNumber;
  INA3221.wireReadRegister(0xFE, &MIDNumber);
  Serial.print(F("Manuf ID:   0x"));
  Serial.print(MIDNumber, HEX);
  Serial.println();

  if (MIDNumber != 0x5449)
  {
    INA3221_Present = false;
    Serial.println(F("INA3221 Not Present"));
  }
  else
  {
    INA3221_Present = true;

    // State Variable
    AuxA = AuxA | 0X02;
  }


}



void loop()
{




  // Only send if source is SLEEP_INTERRUPT
#if defined(TXDEBUG)
  Serial.print(F("wakeState="));
  Serial.println(wakeState);
#endif


  if ((wakeState == SLEEP_INTERRUPT) || (wakeState == REBOOT))
  {

    wakeState = NO_INTERRUPT;






    TimeStamp = millis();

    // if INA3221 present, read charge data

    if (INA3221_Present)
    {


      BatteryVoltage = INA3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
      BatteryCurrent = INA3221.getCurrent_mA(LIPO_BATTERY_CHANNEL);

      SolarPanelVoltage = INA3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
      SolarPanelCurrent = -INA3221.getCurrent_mA(SOLAR_CELL_CHANNEL);


      LoadVoltage = INA3221.getBusVoltage_V(OUTPUT_CHANNEL);
      LoadCurrent = INA3221.getCurrent_mA(OUTPUT_CHANNEL) * 0.75;


    }


    if (BatteryVoltage < 2.80)
      AuxA = AuxA | 0x04;
    else
      AuxA = AuxA & 0xFB;



    // check the AS3935 for interrupts


    // first step is to find out what caused interrupt
    // as soon as we read interrupt cause register, irq pin goes low
    irqSource = AS3935.interruptSource();
    String LastInterrupt;

    if (irqSource != 0)
    {
      InterruptTimeStamp = now();
      // returned value is bitmap field, bit 0 - noise level too high, bit 2 - disturber detected, and finally bit 3 - lightning!

      InterruptCount++;

      Serial.println(F(">>>>>>>>>><<<<<<<<<"));
      Serial.print(F("IRQ Triggered="));
      Serial.println(irqSource);

      if (irqSource & 0b0001)
      {
        LastInterrupt = F("Noise Level Too High");
        //writeStatus("N");
        Serial.println(F("Noise level too high, try adjusting noise floor"));

      }


      if (irqSource & 0b0100)
      {
        //writeStatus("D");
        LastInterrupt = F("Disturber detected");
        Serial.println(F("Disturber detected"));

      }


      if (irqSource & 0b1000)
      {
        // Lightning!

        LastInterrupt = F("Lightning!");


        // need to find how far that lightning stroke, function returns approximate distance in kilometers,
        // where value 1 represents storm in detector's near victinity, and 63 - very distant, out of range stroke
        // everything in between is just distance in kilometers
        int strokeDistance = AS3935.lightningDistanceKm();
        if (strokeDistance == 1)
          Serial.println(F("Storm overhead, watch out!"));
        if (strokeDistance == 63)
          Serial.println(F("Out of range lightning detected."));
        if (strokeDistance < 63 && strokeDistance > 0)
        {
          Serial.print(F("Lightning detected "));
          Serial.print(strokeDistance, DEC);
          Serial.println(F(" kilometers away."));
          LightningLastDistance = strokeDistance;
          LastLightningResult = "";
          LastLightningResult.concat(F("Lightning "));
          LastLightningResult.concat(String(strokeDistance));
          LastLightningResult.concat(F("km"));
          LightningCount++;
          LightningTimeStamp = now();


        }
      }

      Serial.println(F(">>>>>>>>>><<<<<<<<<"));
    }





#if defined(TXDEBUG)
    Serial.println(F("###############"));
    Serial.print(F(" MessageCount="));
    Serial.println(MessageCount);
    Serial.print(F(" STATUS - WeatherSenseProtocol:"));
    Serial.println(WEATHERSENSEPROTOCOL);
    Serial.print(F(" WakeState="));
    Serial.println(wakeState);
    Serial.print(F(" wakeCount="));
    Serial.println(wakeCount);

    Serial.print(F("Current Interrupt Source: 0x"));
    Serial.println(irqSource, HEX);
    Serial.print(F("Last Interrupt Source: 0x"));
    Serial.println(LastInterruptResult, HEX);
    Serial.print(F("Last Interrupt Meaning: "));
    Serial.println(LastInterrupt);
    Serial.print(F("Lightning Last Distance: "));
    Serial.println(LightningLastDistance);
    Serial.print(F("Interrupt Count: "));
    Serial.println(InterruptCount);
    Serial.print(F("Lightning Count:"));
    Serial.println(LightningCount);
    Serial.print(F("Lightning Timestamp:"));
    Serial.println(LightningTimeStamp);





    Serial.print(F(" Battery Voltage:  ")); Serial.print(BatteryVoltage); Serial.println(F(" V"));
    Serial.print(F(" Battery Current:       ")); Serial.print(BatteryCurrent); Serial.println(F(" mA"));
    Serial.print(F(" Solar Panel Voltage:   ")); Serial.print(SolarPanelVoltage); Serial.println(F(" V"));
    Serial.print(F(" Solar Current:  ")); Serial.print(SolarPanelCurrent); Serial.println(F(" mA"));
    Serial.print(F(" Load Voltage:  ")); Serial.print(LoadVoltage); Serial.println(F(" V"));
    Serial.print(F(" Load Current:       ")); Serial.print(LoadCurrent); Serial.println(" mA");
    Serial.print(F(" Currentmillis() = "));
    Serial.println(millis());

    Serial.print(F("  AuxA State:"));
    Serial.print(AuxA);
    Serial.print(F(" "));
    Serial.println(AuxA, HEX);

    Serial.println(F("###############"));
#endif




    // check if it is time to send message (every 10 minutes or new interrupt) - 30 seconds pre check

    if (((wakeCount % 20) == 0) || (irqSource != 0))
    {

      if ((AuxA & 0x04)  == false)   // If the battery vboltage is less than 2.80V, then Lightning Detector is flaky
      {
        // Now send the message

        // write out the current protocol to message and send.
        int bufferLength;


        Serial.println(F("----------Sending packets----------"));
        bufferLength = buildProtocolMessage();

        // Send a message

        //driver.send(byteBuffer, bufferLength);



        driver.send(byteBuffer, bufferLength);
        Serial.println(F("----------After Sending packet----------"));

        for (int i = 0; i < bufferLength; i++) {
          Serial.print(" ");
          if (byteBuffer[i] < 16)
          {
            Serial.print(F("0"));
          }
          Serial.print(byteBuffer[i], HEX);           //  write buffer to hardware serial port
        }
        Serial.println();

        if (!driver.waitPacketSent(6000))
        {
          Serial.println(F("Timeout on transmission"));
          // re-initialize board
          if (!driver.init())
          {
            Serial.println(F("init failed"));
            while (1);
          }
          Serial.println(F("----------Board Reinitialized----------"));
        }



        Serial.println(F("----------After Wait Sending packet----------"));
        delay(100);
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);

        Serial.print(F("freeMemory()="));
        Serial.println(freeMemory());
        Serial.print(F("bufferlength="));
        Serial.println(bufferLength);
        delay(5000);





        MessageCount++;


        // Due to interference from 433MHz transmission to the Lightning Ddtector, we clear the interrupts alfter the transmit.
        irqSource = AS3935.interruptSource();

        Serial.println(F("----------Packet Sent.  Sleeping Now----------"));
      }
    }
    else
    {



      // no send yet
#if defined(TXDEBUG)
      Serial.println(F("No Packet sent"));
#endif

    }

    LastInterruptResult = irqSource;

  }
  // Pat the WatchDog
  ResetWatchdog();

  if (wakeState != REBOOT)
    wakeState = SLEEP_INTERRUPT;
  long timeBefore;
  long timeAfter;
  timeBefore = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeBeforeSleep="));
  Serial.println(timeBefore);
#endif
  delay(100);


  //Sleepy::loseSomeTime(nextSleepLength);
  for (long i = 0; i < nextSleepLength / 16; ++i)
    Sleepy::loseSomeTime(16);

  wakeState = SLEEP_INTERRUPT;

  wakeCount++;

#if defined(TXDEBUG)
  Serial.print(F("Awake now: "));
#endif
  timeAfter = millis();
#if defined(TXDEBUG)
  Serial.print(F("timeAfterSleep="));
  Serial.println(timeAfter);

  Serial.print(F("SleepTime = "));
  Serial.println(timeAfter - timeBefore);

  Serial.print(F("Millis Time: "));
#endif
  long time;
  time = millis();
#if defined(TXDEBUG)
  //prints time since program started
  Serial.println(time / 1000.0);
  Serial.print(F("2wakeState="));
  Serial.println(wakeState);
#endif





  // Pat the WatchDog
  ResetWatchdog();


}
