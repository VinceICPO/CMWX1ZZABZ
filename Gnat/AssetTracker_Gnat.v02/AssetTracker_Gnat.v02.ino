/* Gnat Asset Tracker contains:
   CMWX1ZZABZ (STM32L082 and SX1276)
   BMA400 accelerometer sensor
   MAX M8Q Concurrent GNSS engine

   https://hackaday.io/project/25790-asset-tracker

   Idea is ultra-low power for longest LiPo battery life so I would run this with
   4.2 MHz clock speed; this reduction plus use of STM32 stop mode means no serial
   through the USB.  

   Copyright 2018 Tlera Corporation

   For unlimited distribution with attribution

   This example code is in the public domain.
*/
#include <STM32L0.h>
#include "LoRaWAN.h"
#include "TimerMillis.h"
#include "GNSS.h"
#include <RTC.h>
#include "BMA400.h"
#include "CayenneLPP.h"

// Gnat Asset Tracker gnat1
const char *appEui = "70B3D57ED0037414";
const char *appKey = "415E84C9B846F236029C1E478021637E";
const char *devEui = "383434305737890a";
   
CayenneLPP myLPP(64);

// Cricket/GNAT pin assignments
#define myLed    10 // blue led 
#define myBat    A1 // LiPo battery ADC
#define myBat_en  2 // LiPo battery monitor enable

uint8_t LoRaData[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

GNSSLocation myLocation;
GNSSSatellites mySatellites;

volatile bool SendGPS = true;
volatile bool onReceive = false;
volatile bool test = false;
volatile int AlarmLevel = 0;

TimerMillis LoRaTimer; // instantiate callBackLoRaTx timer
TimerMillis RebootTimer; // instantiate a call for reboot (this is a temporary measure to try to avoid the device is hang)
TimerMillis NoMotionActivityTimer;  // instantiate low-frequency timer
TimerMillis InMotionActivityTimer;  // instantiate high-frequency timer

uint32_t UID[3] = {0, 0, 0}; 
char buffer[32];

bool SerialDebug = false;

// MAX M8Q GNSS configuration
  #define GNSS_en          5     // enable for GNSS 3.0 V LDO
  #define pps              4     // The PIN on which we will receive the 1 Hz pulse coming from the MAX M8Q GPS RTC)
  #define GNSS_backup     A0     // RTC backup for MAX M8Q

float Acceptable_ehpe = 15;     // This is the minimal EPE value we consider as a valid and accurate fix. Anything below 20 is acceptable. Max accuracy would be around 5.
float Acceptable_ehpe_OnTheMove  =  30;    // This is the minimal EPE for reporting position on the move

  static const char *fixTypeString[] = {
      "NONE",
      "TIME",
      "2D",
      "3D",
  };
  
  static const char *fixQualityString[] = {
      "/NONE",
      "/AUTONOMOUS",
      "/DIFFERENTIAL",
      "/PRECISE",
      "/RTK_FIXED",
      "/RTK_FLOAT",
      "/ESTIMATED",
      "/MANUAL",
      "/SIMULATION",
  };
  
  uint16_t count = 0, fixType = 0;
  int32_t latOut, longOut;
  float Long, Lat, Alt;
  float LongTx, LatTx, AltTx, ehpeTx;

  unsigned int myAcqTime =  45;
  unsigned int myOnTime  =  15;
  unsigned int myPeriod  = 120;

// end MAX M8Q configuration

uint16_t Hour = 1, Minute = 1, Second = 1, Millisec, Year = 1, Month = 1, Day = 1;
uint8_t hours = 12, minutes = 0, seconds = 0, year = 1, month = 1, day = 1;
uint32_t subSeconds, milliseconds;
bool ppsFlag = false, firstSync = false, alarmFlag = true;

float Temperature, STM32L0Temp;

// battery voltage monitor definitions
float VDDA, VBAT;

float VBUS; //for USB check (if = 1, then USB is connected)

//BMA400 definitions
#define BMA400_intPin1 A4   // interrupt1 pin definitions, wake-up from STANDBY pin
#define BMA400_intPin2  3   // interrupt2 pin definitions, data ready or sleep interrupt

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      SR_15_5Hz, SRW_25Hz, SR_50Hz, SR_100Hz, SR_200Hz, SR_400Hz, SR_800Hz 
      sleep_Mode, lowpower_Mode, normal_Mode 
      osr0 (lowest power, lowest oversampling,lowest accuracy), osr1, osr2, osr3 (highest power, highest oversampling, highest accuracy)
      acc_filt1 (variable filter), acc_filt2 (fixed 100 Hz filter), acc_filt_lp (fixed 100 Hz filter, 1 Hz bandwidth)
*/ 
uint8_t Ascale = AFS_2G, SR = SR_200Hz, power_Mode = lowpower_Mode, OSR = osr0, acc_filter = acc_filt2;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // accel bias offsets

// Logic flags to keep track of device states
bool BMA400_wake_flag = false;
bool BMA400_sleep_flag = false;
bool InMotion = false;
bool ActivityOn = true;

BMA400 BMA400(BMA400_intPin1, BMA400_intPin2); // instantiate BMA400 class

void (* RebootCall) (void) = 0; // Declare function RebootCall pointing to addr 0 of the flash, in effect, will reboot the device

//================================  SETUP ================================
void setup()
{
  /* Enable USB UART */
  Serial.begin(115200);
  delay(4000);
  Serial.println("===== Serial enabled =====");

  /* Get LoRa/LoRaWAN ID for SX1276 */
  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX); 

  LoRaWAN.getDevEui(buffer, 18);
  Serial.print("STM32L0 Device EUI = "); Serial.println(buffer); 
  
  /* configure IO pins */
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with blue led off, since active LOW

  pinMode(myBat_en, OUTPUT);
  pinMode(myBat, INPUT);    // set up ADC battery voltage monitor pin
  analogReadResolution(12); // use 12-bit ADC resolution

  pinMode(BMA400_intPin1, INPUT);  // define BMA400 wake and sleep interrupt pins as L082 inputs
  pinMode(BMA400_intPin2, INPUT);

  pinMode(pps, INPUT); // select pps as input from MAX M8Q

  /* initialize wire bus */
  Wire.begin(); // set master mode on default pins 14/15
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

  BMA400.I2Cscan(); // should detect BMA400 at 0x18 
  delay(1000);
  
  /* Initialize and configure GNSS */
 
  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_10HZ); // Start GNSS (to reduce consuption, reduce rate to RATE_1HZ).
  
  GNSS.setPeriodic(myAcqTime, myOnTime, myPeriod);
  //GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GALILEO); // choose satellites
  GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS);
  while (GNSS.busy()) { } // wait for begin to complete

  GNSS.setPlatform(GNSS.PLATFORM_BIKE); //Tell GNSS that this is for a vehicle tracker (Specs include _STATIONARY, _CAR, _PEDESTRIAN, _SEA, _BALLON (AVIATION_1G), _AVIATION_2G, _AVIATION_4G, _WRIST, _BIKE). However anything above _BALLON has not been implemented in Tiera's library yet.
  GNSS.setAntenna(GNSS.ANTENNA_EXTERNAL);
  GNSS.setQZSS(true);
  GNSS.setSBAS(true);
  //GNSS.setAutonomous(true);

  GNSS.enableWakeup();
  Serial.println("GNSS is awake and set");

  /* Set the RTC time */
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

  // Set the RTC date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  
  /* Check internal STML082 and battery power configuration */
  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  VBAT = 1.27f * 3.30f * ((float) analogRead(myBat)) / 4096.0f;
  STM32L0Temp = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
  if(VBUS ==  1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature is "); Serial.print(STM32L0Temp, 2); Serial.print(" degrees C");
  
  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial.println("BMA400 accelerometer...");
  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I am "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  delay(1000); 

  if(c == 0x90) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial.println("BMA400 is online...");
   
   aRes = BMA400.getAres(Ascale);                                       // get sensor resolutions, only need to do this once
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(100);      
   BMA400.selfTestBMA400();                                             // perform sensor self test
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(1000);                                                         // give some time to read the screen
   BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
   BMA400.initBMA400(Ascale, SR, power_Mode, OSR, acc_filter);          // Initialize sensor in desired mode for application                     
 
  }
  else 
  {
  if(c != 0x90) Serial.println(" BMA400 not functioning!");
  }

   // set alarm to update the RTC periodically
//  RTC.setAlarmTime(0, 0, 0);
//   RTC.enableAlarm(RTC.MATCH_SS);  // once per minute
    RTC.enableAlarm(RTC.MATCH_ANY); // once per second

  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(BMA400_intPin1, BMA_INT_onActivity, RISING);  // define wake-up interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2, BMA_INT_NoActivity, RISING);  // define data ready interrupt for INT2 pin output of BMA400
  BMA400.getStatus(); // read status of interrupts to clear

  attachInterrupt(pps, CAMM8QintHandler, RISING);


  // Configuree LoRaWAN connection 
  /* Parameter for LoRAWAN.begin() sets the frenquency plan
    - Asia       AS923
    - Australia  AU915
    - Europe     EU868
    - India      IN865
    - Korea      KR920
    - US         US915 (64 + 8 channels)
  
  
    LoRaWAN.begin(EU868);
    LoRaWAN.setADR(false); //false = device already has a DevEUI
    LoRaWAN.setDataRate(0);
    LoRaWAN.setTxPower(16);
    LoRaWAN.setPublicNetwork(true);
    LoRaWAN.setSaveSession(true);
    LoRaWAN.setAntennaGain(6.0);
    LoRaWAN.setSubBand(2); // 1 for MTCAP, 2 for TT gateways
    LoRaWAN.setDutyCycle(false);
    LoRaWAN.addChannel(1, 868300000, 0, 6);
    
    LoRaWAN.onReceive(callback_onReceive);
 */
    
// LoRaWAN.join moved to callBackLoRaTx


    /* For production
        NoMotionActivityTimer.start(callbackNoMotionActivity, 100000, 7200000);    // low  freq (two hours) timer
        InMotionActivityTimer.start(callbackInMotionActivity, 100000,   60000);    // high freq (one minute) timer
    
    // For testing
        NoMotionActivityTimer.start(callbackNoMotionActivity, 0, 360000);        // low  freq (five minute) timer
        InMotionActivityTimer.start(callbackInMotionActivity, 100000,   60000);  // high freq (one minute) timer
     */
    
    // For production TimerMillis is in milliseconds, hence 1000=1s, 10000=10s, 60000= 1mn, 360000=1h, etc.
        //LoRaTimer.start(callbackLoRaTx, 300000, 600000);      //  after an initial wait of 5mn, callbackLoRaTx avery 10 minutes

        NoMotionActivityTimer.start(callbackNoMotionActivity, 0, 7200000);  // low  freq (after an initial wait of 0mn then callback every two (2) hours
        InMotionActivityTimer.start(callbackInMotionActivity, 0,   60000);  // high freq (after an initial wait of 0mn then callback every minute
        RebootTimer.start(RebootCall, 0, 43200000); // Reboot every 12h
} /* end of setup */


//================================ LOOP ================================
void loop(){
 
  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
   BMA400_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   SendGPS = true;
   BMA400.activateNoMotionInterrupt();
   GNSS.resume();
   attachInterrupt(BMA400_intPin2, BMA_INT_NoActivity, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 
  }

  if(BMA400_sleep_flag)
  {
   BMA400_sleep_flag = false;            // clear the sleep flag
   detachInterrupt(BMA400_intPin2);       // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 
  }/* end of sleep/wake detect */

  
  /*GNSS*/

  if(GNSS.location(myLocation) && (myLocation.ehpe() > 0 )) //if GNSS has been sent to sleep then ehpe is = 0 and therefore we can skip all of the below)
  {
  Serial.print(" =====> LOCATION: ");
  Serial.print(fixTypeString[myLocation.fixType()]);
  Serial.print(". Satellites in range: ");
  Serial.print(myLocation.satellites());
  Serial.print(". ehpe: ");
  Serial.print(myLocation.ehpe(),3);
  Serial.println(" <=====");
  Serial.print("We will register location only if fix ehpe is less than ");
  Serial.print(Acceptable_ehpe_OnTheMove);
  Serial.println(". ");
  Serial.print("GNSS goes to sleep when the ehpe is less than ");
  Serial.print(Acceptable_ehpe);
  Serial.println(".");

    if (myLocation.fixType() >= GNSSLocation::TYPE_NONE)
    {
        Hour   = myLocation.hours();
        Minute = myLocation.minutes();
        Second = myLocation.seconds();
        Year   = myLocation.year();
        Month  = myLocation.month();
        Day    = myLocation.day();
  
        Serial.print("Fix quality: ");
        Serial.print(fixQualityString[myLocation.fixQuality()]);
        Serial.print(". Satellite timestamp: ");
        Serial.print(myLocation.day());
        Serial.print("/");
        Serial.print(myLocation.month());
        Serial.print("/");
        Serial.print(myLocation.year());
        Serial.print(" ");
        if (myLocation.hours() <= 9) {Serial.print("0");}
        Serial.print(myLocation.hours());
        Serial.print(":");
        if (myLocation.minutes() <= 9) {Serial.print("0");}
        Serial.print(myLocation.minutes());
        Serial.print(":");
        if (myLocation.seconds() <= 9) {Serial.print("0");}
        Serial.print(myLocation.seconds());
        Serial.print(".");
        if (myLocation.millis() <= 9) {Serial.print("0");}
        if (myLocation.millis() <= 99) {Serial.print("0");}
        Serial.print(myLocation.millis());
        if (myLocation.leapSeconds() != GNSSLocation::LEAP_SECONDS_UNDEFINED) {
                  Serial.print(" ");
                  Serial.print(myLocation.leapSeconds());
                  if (!myLocation.fullyResolved()) {
                      Serial.print("D");
                  };
         }
         Serial.println(" ");
  
        if (myLocation.fixType() >= GNSSLocation::TYPE_TIME)
        {
        Lat = myLocation.latitude();
        myLocation.latitude(latOut);
        Long = myLocation.longitude();
        myLocation.longitude(longOut);
        Alt = myLocation.altitude();
        
        Serial.print("LLA:");
        Serial.print(Lat, 7);
        Serial.print(",");
        Serial.print(Long, 7);
        Serial.print(",");
        Serial.println(Alt, 3);
    
        // ONLY save location values for transmit if they are below Acceptable_ehpe_OnTheMove (ehpe is an evaluated accuracy from the MAX M8Q GNSS).
        if ( (myLocation.fixType() >= GNSSLocation::TYPE_3D) && (myLocation.fixQuality() >=  GNSSLocation::QUALITY_AUTONOMOUS) && (myLocation.ehpe() <= Acceptable_ehpe_OnTheMove) /*&& myLocation.fullyResolved()*/)
          { 
            Serial.print(" ehpe,evpe:");
            Serial.print(myLocation.ehpe(), 3);
            Serial.print(",");
            Serial.print(myLocation.evpe(), 3);
            Serial.print(" hdop,vdop:");
            Serial.print(myLocation.hdop(), 2);
            Serial.print(",");
            Serial.print(myLocation.vdop(), 2);
            Serial.print(". Nav satellites:");
            Serial.print(myLocation.satellites());
            Serial.print(". SendGPS status:");
            if (SendGPS) { 
              Serial.println(" Yes.");
            } else {
              Serial.println(" No.");
            };
            
            LatTx = Lat;
            LongTx = Long;
            AltTx = Alt;
            ehpeTx = (myLocation.ehpe(), 3);
          };
          
      
        // Put the CAM M8Q to sleep once 3D fix is obtained with sufficient accuracy
        if ( (myLocation.fixType() >= GNSSLocation::TYPE_3D) && (myLocation.ehpe() <= Acceptable_ehpe) /*&& myLocation.fullyResolved()*/ && SendGPS)
          {
                      SendGPS = false;
                      Serial.println("***GNSS go to sleep!***");
                      GNSS.suspend(); // once we have a good 3D location fix let CAM M8Q go to sleep
                      Serial.println(" ===== LoRaWAN call to update stationary location when GNSS went to sleep =====");
                      callbackLoRaTx();  // update dashboard/backend via LoRaWAN
           } 
           else {
                if ( (myLocation.fixType() >= GNSSLocation::TYPE_3D) && (myLocation.ehpe() <= Acceptable_ehpe) /*&& myLocation.fullyResolved()*/)
                {
                            Serial.println("***GNSS go to sleep!***");
                            GNSS.suspend(); // once we have a good 3D location fix let CAM M8Q go to sleep
                 } 
            }
        }

    } 

  if (SerialDebug && GNSS.satellites(mySatellites) && (myLocation.ehpe() > 0)) //Print the list of satellites and their details - Check ehpe > 0 isn't really used here.
  {
  
  Serial.print("SATELLITES: ");
  Serial.println(mySatellites.count());

  for (unsigned int index = 0; index < mySatellites.count(); index++) {
      unsigned int svid = mySatellites.svid(index);
       /* 
       For the below svis to letters
       * svid vs PRN ranges:
       * UBX       |   GNSS
       * svid      |   Type  
       * ----------------------
       * 1-32      |   GPS
       * 33-64     |   BEIDOU
       * 65-96     |   GLONASS
       * 120-158   |   SBAS
       * 159-163   |   BEIDOU
       * 173-182   |   IMES
       * 193-200   |   QZSS
       * 211-246   |   GALILEO
       * 255       |   GLONASS
       */
      if ((svid >= 1) && (svid <= 32))
      {
          Serial.print("    ");
    
          if (svid <= 9)
          {
        Serial.print("  G"); // G is for GPS Satellites
        Serial.print(svid);
          }
          else
          {
        Serial.print(" G");
        Serial.print(svid);
          }
      }
      else if ((svid >= 65) && (svid <= 96))
      {
          Serial.print("    ");
    
          if ((svid - 64) <= 9)
          {
        Serial.print("  R"); // R is for GLONASS Satellites
        Serial.print(svid - 64);
          }
          else
          {
        Serial.print(" R");
        Serial.print(svid - 64);
          }
      }
      else if ((svid >= 120) && (svid <= 158))
      {
          Serial.print("    ");
          Serial.print("S");  // S is for SBAS
          Serial.print(svid);
      }
      else if ((svid >= 159) && (svid <= 163))
      {
          Serial.print("    ");
          Serial.print("B");  // B is for BEIDOU
          Serial.print(svid);
      }
      else if ((svid >= 173) && (svid <= 182))
      {
          Serial.print("    ");
          Serial.print("  I"); // I is for IMES Satellites
          Serial.print(svid - 172);
      }
      else if ((svid >= 193) && (svid <= 200))
      {
          Serial.print("    ");
          Serial.print("  Q"); // Q is for QZSS Satellites
          Serial.print(svid - 192);
      }
      else if ((svid >= 211) && (svid <= 246))
      {
          Serial.print("    ");
    
          if ((svid - 210) <= 9)
          {
        Serial.print("  E"); // E is for Galileo Satellites
        Serial.print(svid - 210);
          }
          else
          {
        Serial.print(" E");
        Serial.print(svid - 210);
          }
      }
      else if (svid == 255)
      {
          Serial.print("    ");
          Serial.print("RNUL"); // GLONASS "Null"
      }
      else
      {
          continue;
      }
    
      Serial.print(": SNR=");
      Serial.print(mySatellites.snr(index));
      Serial.print(", ELEVATION=");
      Serial.print(mySatellites.elevation(index));
      Serial.print(", AZIMUTH=");
      Serial.print(mySatellites.azimuth(index));
    
      if (mySatellites.unhealthy(index)) { Serial.print(", UNHEALTHY");}
      if (mySatellites.almanac(index)) { Serial.print(", ALMANAC");}
      if (mySatellites.ephemeris(index)) { Serial.print(", EPHEMERIS");}
      if (mySatellites.autonomous(index)) { Serial.print(", AUTONOMOUS");}
      if (mySatellites.correction(index)) { Serial.print(", CORRECTION");}
      if (mySatellites.acquired(index)) { Serial.print(", ACQUIRED");}
      if (mySatellites.locked(index)) { Serial.print(", LOCKED");}
      if (mySatellites.navigating(index)) { Serial.print(", NAVIGATING");}
      Serial.println();
    } //end For section
  } // end Print Sat details (if Serialdebug is true)
 }/* end of GNSS Satellites handling */


  /*RTC*/
  if (alarmFlag) { // update RTC output whenever there is a GNSS pulse
    alarmFlag = false;
    if(myLocation.ehpe() == 0) {
       Serial.println("Not enough satellites...");
    }
    if (SerialDebug && BMA400_wake_flag) {
      
     BMA400.readBMA400AccelData(accelCount); // get 12-bit signed accel data

    // Now we'll calculate the accceleration value into actual g's
     ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes - offset[1];   
     az = (float)accelCount[2]*aRes - offset[2]; 
     
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");    
    }

    VDDA = STM32L0.getVDDA();
    digitalWrite(myBat_en, HIGH);
    VBAT = 1.27f * VDDA * ((float) analogRead(myBat)) / 4096.0f;
    digitalWrite(myBat_en, LOW);
    STM32L0Temp = STM32L0.getTemperature();
    if(SerialDebug) {
      Serial.println(" === Serial debug ===");
      Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
      Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
      Serial.print("STM32L0 MCU Temperature is "); Serial.print(STM32L0Temp, 2);Serial.println(" degrees C");
      tempCount = BMA400.readBMA400TempData();  // Read the accel chip Temperature adc values
      Temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip Temperature in degrees Centigrade
      // Print Temperature in degrees Centigrade      
      Serial.print("Accel Temperature is ");  Serial.print(Temperature, 2);  Serial.println(" degrees C");
      Serial.println(" === end debug ===");
    }

    // Read RTC
    RTC.getDate(day, month, year);
    RTC.getTime(hours, minutes, seconds, subSeconds);

    milliseconds = ((subSeconds >> 17) * 1000 + 16384) / 32768;

    Serial.print("RTC Time = ");
    if (hours < 10)   {Serial.print("0");Serial.print(hours); } else Serial.print(hours);
    Serial.print(":");
    if (minutes < 10) {Serial.print("0"); Serial.print(minutes); } else Serial.print(minutes);
    Serial.print(":");
    if (seconds < 10) {Serial.print("0"); Serial.print(seconds); } else Serial.print(seconds);
    Serial.print(".");
        if (milliseconds <= 9) {
            Serial.print("0");
        }
        if (milliseconds <= 99) {
            Serial.print("0");
        }
    Serial.print(milliseconds);
    Serial.println();

    Serial.print("RTC Date = ");
    Serial.print(day); Serial.print("/"); Serial.print(month); Serial.print("/"); Serial.println(year);
    Serial.println();

    digitalWrite(myLed, !digitalRead(myLed)); delay(1); digitalWrite(myLed, !digitalRead(myLed));
        
    } // end of alarm section
    


    STM32L0.stop();        // Enter STOP mode and wait for an interrupt
   
}  /* end of loop*/

/*================================  OTHER FUNCTIONS ================================*/

void callbackLoRaTx(){ 

// This boggus data section is only required as long as Huminidy and Pressure sensor is not connected
  const float Pressure = 1080;
  const float Humidity = 70;
  int TimeOut;

    LoRaWAN.begin(EU868);
    LoRaWAN.setADR(false); //false = device already has a DevEUI
    LoRaWAN.setDataRate(0);
    LoRaWAN.setTxPower(16);
    LoRaWAN.setPublicNetwork(true);
    LoRaWAN.setSaveSession(true);
    LoRaWAN.setAntennaGain(6.0);
    LoRaWAN.setSubBand(2); // 1 for MTCAP, 2 for TT gateways
    LoRaWAN.setDutyCycle(false);
    LoRaWAN.addChannel(1, 868300000, 0, 6);
    
    LoRaWAN.onReceive(callback_onReceive);
  
  Serial.println(" ====== callbackLoRaTx =====");   
/*  // Send some data via LoRaWAN
  LoRaData[0]  = (uint16_t(STM32L0Temp*100.0) & 0xFF00) >> 8;
  LoRaData[1]  =  uint16_t(Temperature*100.0) & 0x00FF; //Temperature_C
  LoRaData[2] =  (uint16_t(Pressure*10.0      ) & 0xFF00) >> 8;   
  LoRaData[3] =   uint16_t(Pressure*10.0      ) & 0x00FF;         
  LoRaData[4] =  (uint16_t(Humidity*100.0     ) & 0xFF00) >> 8;
  LoRaData[5] =   uint16_t(Humidity*100.0     ) & 0x00FF;
  LoRaData[6] =  (uint16_t( (LongTx + 123.0)*10000.0 ) & 0xFF00) >> 8;
  LoRaData[7] =   uint16_t( (LongTx + 123.0)*10000.0 ) & 0x00FF;
  LoRaData[8] =  (uint16_t( (LatTx   - 37.0)*10000.0 ) & 0xFF00) >> 8;
  LoRaData[9] =   uint16_t( (LatTx   - 37.0)*10000.0 ) & 0x00FF;
  LoRaData[10] =  uint8_t(VDDA*50.0); // maximum should be 4.2 * 50 = 210
  LoRaData[11] =  uint8_t(ehpeTx); //a means of evaluating GPS accuracy 
 */
TimeOut = 0;

while ((!LoRaWAN.joined()) && (TimeOut < 10)) {
    TimeOut++;
    if (LoRaWAN.linkGateways()){
       Serial.println("LoRaWAN REJOIN( )"); 
       LoRaWAN.rejoinOTAA();
    }
    else {
      LoRaWAN.joinOTAA(appEui, appKey, devEui);
      Serial.print("Sent JoinOTAA to LoRaWan with appEui= "); Serial.print(appEui); Serial.print("; appKey = ");Serial.print(appKey);Serial.print("; devEui = ");Serial.println(devEui);
      if (LoRaWAN.joined()){
                Serial.print("TRANSMIT( ");
                Serial.print("TimeOnAir: ");
                Serial.print(LoRaWAN.getTimeOnAir());
                Serial.print(", NextTxTime: ");
                Serial.print(LoRaWAN.getNextTxTime());
                Serial.print(", MaxPayloadSize: ");
                Serial.print(LoRaWAN.getMaxPayloadSize());
                Serial.print(", DR: ");
                Serial.print(LoRaWAN.getDataRate());
                Serial.print(", TxPower: ");
                Serial.print(LoRaWAN.getTxPower(), 1);
                Serial.print("dbm, UpLinkCounter: ");
                Serial.print(LoRaWAN.getUpLinkCounter());
                Serial.print(", DownLinkCounter: ");
                Serial.print(LoRaWAN.getDownLinkCounter());
                Serial.println(" )");
      };
    };
    if (!LoRaWAN.joined()){
      Serial.print("LoRaWAN.joinOTAA attempt: ");
      Serial.print(TimeOut);
      delay(5000);
    };
};

//give up after 10 attempts, continue as if nothing happened !

  if (!LoRaWAN.busy() && LoRaWAN.joined()){
    Serial.println("===== LoRaWan - Joined =====");
    Serial.print("TRANSMIT( ");
    Serial.print("TimeOnAir: ");
    Serial.print(LoRaWAN.getTimeOnAir());
    Serial.print(", NextTxTime: ");
    Serial.print(LoRaWAN.getNextTxTime());
    Serial.print(", MaxPayloadSize: ");
    Serial.print(LoRaWAN.getMaxPayloadSize());
    Serial.print(", DR: ");
    Serial.print(LoRaWAN.getDataRate());
    Serial.print(", TxPower: ");
    Serial.print(LoRaWAN.getTxPower(), 1);
    Serial.print("dbm, UpLinkCounter: ");
    Serial.print(LoRaWAN.getUpLinkCounter());
    Serial.print(", DownLinkCounter: ");
    Serial.print(LoRaWAN.getDownLinkCounter());
    Serial.println(" )");
    
     // Send and confirm Cayenne payload on port 1*/
     myLPP.reset();
     myLPP.addGPS(1, LatTx, LongTx, AltTx);
     myLPP.addAnalogInput(2, VDDA);
     myLPP.addAnalogInput(3, AlarmLevel);
     myLPP.addAnalogInput(4, ehpeTx);
     myLPP.addTemperature(5, STM32L0Temp);
     myLPP.addAccelerometer(6, ax, ay, az);
     
     Serial.print(char(7)); //beep
     Serial.print(char(7)); //beep
     Serial.print(char(7)); //beep
     Serial.print(char(7)); //beep
     Serial.print(char(7)); //beep
     Serial.print(char(7)); //beep
     Serial.print(char(7)); //beep
     Serial.print(char(7)); //beep
          
     LoRaWAN.beginPacket(1);
     LoRaWAN.sendPacket(2,myLPP.getBuffer(), myLPP.getSize());
     LoRaWAN.write(myLPP.getBuffer(),myLPP.getSize());
     LoRaWAN.endPacket();     
     /*
     // Send and confirm LoRaData payload on port 2
     LoRaWAN.beginPacket(2);
     LoRaWAN.write(LoRaData, sizeof(LoRaData));
     LoRaWAN.endPacket();
     */ 
    }
}

void callbackNoMotionActivity(void){

  Serial.print(char(7)); //beep
  Serial.print(char(7)); //beep

  GNSS.resume(); // In this calback, we are in the long duty cycle. We simply resume GNSS and get a new fix to send an updated position (which should be same as last fix since we had no movement detection...)
  SendGPS = true;
}


void callbackInMotionActivity(void){

  Serial.print(char(7)); //beep

  if ((InMotion) && !(SendGPS))// While we are called by the short duty cycle, no need to do anything unless movement detected and therefore the BMA400_wake_flag was raised (InMotion is true)!
                            // We avoid calling the function again if SendGPS=true because it means there was already reported movement and GPS has not got a fix to send the position yet.
                            // Effectively calling this too many times eventually hangs the board.
                            // If all is good then we call GNSS and then loracallback for GPS update.
  {
   STM32L0.wakeup();
   AlarmLevel = 100;
   SendGPS = true;
   InMotion = false;
   GNSS.resume();
  }
}



void BMA_INT_onActivity(){
  BMA400_wake_flag = true; 

  InMotion = true;
  Serial.println("** BMA400 is awake! **");

  Serial.print(char(7)); //beep
  Serial.print(char(7)); //beep
  Serial.print(char(7)); //beep
  Serial.print(char(7)); //beep
  STM32L0.wakeup();
  GNSS.resume();

}


void BMA_INT_NoActivity(){
  BMA400_sleep_flag = true;
  //STM32L0.wakeup();
  Serial.println("** BMA400 is asleep! **"); //let cycle finish until fix is of good quality. Then MAX M8Q will go back to sleep.
}


void CAMM8QintHandler(){
  ppsFlag = true;
  STM32L0.wakeup();
}


void alarmMatch(){
  alarmFlag = true;
  STM32L0.wakeup();
}


void callback_onReceive(){
   AlarmLevel = 0;
   BMA400_wake_flag = true; 
   STM32L0.wakeup();
}

void syncRTC()
{
  // Set the time
  RTC.setSeconds(Second);
  RTC.setMinutes(Minute);
  if (Hour < 7) {
    RTC.setHours(Hour + 17);
  } else RTC.setHours(Hour - 7);
  RTC.setMinutes(Minute);

  // Set the date
  if (Hour < 7) {
    RTC.setDay(Day - 1);
  } else RTC.setDay(Day);
  RTC.setMonth(Month);
  RTC.setYear(Year - 2000);
} 
