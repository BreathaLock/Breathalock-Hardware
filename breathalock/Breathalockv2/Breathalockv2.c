//Stdlibs
#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

//Bluetooth
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BreathalockConfig.h"

//Fingerprint
#include <Adafruit_Fingerprint.h>

//////////
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


SoftwareSerial mySerial(FINGER_SOFTWARE_SERIAL_TX, FINGER_SOFTWARE_SERIAL_RX);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

boolean bFingerprintInitialised = false;
// This value is used for determining if the MQ3 has been intilized yet
boolean bMQ3Initialised = false;

int iFingerprintReadAttempts = 0;
boolean bUnlockedFingerprint = false;
boolean bDeviceLockedDueToFailedAuth = false;
boolean bUserIsOverTheLimit = false;
float fHighestAlcoholRead = 0;


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  
  if(initPinsOnBoard()) {
    Serial.println("Initialised GPIO...");
  }
  
  if(!initBleDevice()) {
    Serial.println("Initialised Bluetooth LE module...");
  }
  
  //Check if fingerprint is attached
  finger.begin(57600);
  Serial.print("Initialising Fingerprint module: ");
  if(!finger.verifyPassword()) {
    Serial.println("Did not find fingerprint sensor");
    bFingerprintInitialised = false;
  }else {
    Serial.println("OK!");
    bFingerprintInitialised = true;
  }
  //DEBUG WITHOUT FINGERPRINT UNCOMMENT BELOW
//  bUnlockedFingerprint=false; //COMMENT THIS COMMENT THIS COMMENT THIS!
  
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  float mq3AlcoholRead = readAlcoholGasSensor(100);
  float mq3Delta = 0;
  char commandToSend[30];
 
  if(!bMQ3Initialised) {
    if(initMQ3Sensor()) {
      bMQ3Initialised = true;
      Serial.println("Initilised MQ3");
    }
  }
  
  if(!bUnlockedFingerprint) {
    if(getFingerprintStatus() != -1) {
      bUnlockedFingerprint=true;
      finger.end();
      delay(500);
    }
  } else { 
    if(bMQ3Initialised) {
        mq3AlcoholRead = readAlcoholGasSensor(1000);
        mq3Delta = readAlcoholDelta();
        if( ((abs(mq3Delta*100)) >= BREATH_DELTA*100) && !bUserIsOverTheLimit) { //is breathing
          Serial.print("isBreathing delta = "); Serial.println(mq3Delta);
          Serial.print("isBreathing tolerance = "); Serial.println(BREATH_DELTA);
          if(mq3AlcoholRead > fHighestAlcoholRead) {
              fHighestAlcoholRead = mq3AlcoholRead;
          }
          if(mq3Delta*100 >= ALCOHOL_TOLERANCE_DELTA*100) { //drunk
            bUserIsOverTheLimit = true;
            Serial.print("isDrunk Delta = "); Serial.println(mq3Delta);
            digitalWrite(GATE_POWER_PIN,LOW);
            colorChange(50,0,0); //red
            stringifyAlcohol(fHighestAlcoholRead,false).toCharArray(commandToSend,30);
          }else if (mq3Delta*100 < 0) {
            Serial.print("notDrunk Delta = "); Serial.println(mq3Delta);
            digitalWrite(GATE_POWER_PIN,HIGH); 
            colorChange(0,50,0); //green
            stringifyAlcohol(fHighestAlcoholRead,true).toCharArray(commandToSend,30);
          }
          Serial.print("ble signal = "); Serial.println(commandToSend);
          ble.sendCommandCheckOK(commandToSend);
      }else {
        Serial.print("nothing read = "); Serial.println(mq3AlcoholRead);
        Serial.print("nothing abs(delta) = "); Serial.println( (float) abs( mq3Delta*100)/100);
        Serial.print("nothing delta = "); Serial.println(mq3Delta);
        Serial.print("nothing tolerance = "); Serial.println(BREATH_DELTA);
//        Serial.print("ble signal = "); Serial.println(commandToSend);
//        stringifyAlcohol(fHighestAlcoholRead,false).toCharArray(commandToSend,30);
//        ble.sendCommandCheckOK(commandToSend);
      }
    }
    
  }  
}

/**************************************************************************/
/*!
    @brief  initialize the bluetooth device
    @return true if successful otherwise return false
*/
/**************************************************************************/
bool initBleDevice() {
  if ( !ble.begin(VERBOSE_MODE) )
  {
    Serial.println(F("Couldn't find Bluefruit, make sure it's in CMD mode & check wiring?"));
    return false;
  }
  Serial.println( F("OK!") );

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  ble.verbose(VERBOSE_MODE); 
  ble.sendCommandCheckOK("AT+GAPDEVNAME=" DEVICE_NAME);
  
  return true;
}

/**************************************************************************/
/*!
    @brief  initialize the pins 
    @return true if successful otherwise return false
*/
/**************************************************************************/
bool initPinsOnBoard() {
  pinMode(GATE_POWER_PIN, OUTPUT); // for the unlock button
  pinMode(STATUS_LIGHT_YELLOW, OUTPUT); // if drunk this comes on
  pinMode(RED_RGB_PIN_STATUS, OUTPUT);
  pinMode(BLUE_RGB_PIN_STATUS, OUTPUT);
  pinMode(GREEN_RGB_PIN_STATUS, OUTPUT);
  
  return true;
}

/**************************************************************************/
/*!
    @brief     change status light led to corresponding color
    @return    return true on success
*/
/**************************************************************************/
boolean colorChange(int red, int blue, int green ) {
  
  analogWrite(RED_RGB_PIN_STATUS,red);
  analogWrite(GREEN_RGB_PIN_STATUS,green);
  analogWrite(BLUE_RGB_PIN_STATUS,blue);
  
  return true;
}


/**************************************************************************/
/*!
    @brief     convert alcohol Value to string
    @return    return false after 15secs
*/
/**************************************************************************/
String stringifyAlcohol(float alcohol_val,boolean pass) {
  String ATICommand= "AT+BLEUARTTX= ";
  String alcReadToStr;
  
  alcReadToStr = String(alcohol_val); //alcoholVal
  alcReadToStr.concat(":"); //alcoholVal + ":" + (P|F) 
  if(pass) {
    alcReadToStr.concat("P");
  }else {
    alcReadToStr.concat("F");
  }
  
  ATICommand.concat(alcReadToStr);

  return ATICommand;
}

/**************************************************************************/
/*!
    @brief     read the gas sensor value
    @return    return the average value take from the gas sensor
    @see       https://cdn.sparkfun.com/datasheets/Sensors/Biometric/MQ-3%20ver1.3%20-%20Manual.pdf
*/
/**************************************************************************/

float readAlcoholGasSensor(int reads) {
  float mq3Sum=0;
  float mq3RealValue = 0;
  float mq3Average = 0;
  
  //sum together READs number of analogReads for the gas sensor to collect an avg
  for(int i=0;i<reads;i++) {
    mq3Sum = mq3Sum + analogRead(GAS_SENSOR_PIN);
  }
  
  mq3Average = mq3Sum/reads; 
  if(RETURN_VOLTAGE) {
    return (mq3Average/1024)*5.0; 
  }
  return mq3Average;
} 


/**************************************************************************/
/*!
    @brief     read the gas sensor value
    @return    return the delta between two reads
*/
/**************************************************************************/

float readAlcoholDelta() {
  float mq3ReadOne = 0;
  float mq3ReadTwo = 0;

  mq3ReadOne=readAlcoholGasSensor(100);
  delay(500);
  mq3ReadTwo=readAlcoholGasSensor(100);

  return mq3ReadTwo - mq3ReadOne;
}


/**************************************************************************/
/*!
    @brief     This gives us our inital warmed up value that we utilize in comparisons later
    @return    return false after 15secs
*/
/**************************************************************************/
boolean initMQ3Sensor() {
  if(millis() >= WARMUP_TIME_MS) {
    digitalWrite(STATUS_LIGHT_YELLOW,LOW);
    return true;
  } 
  digitalWrite(STATUS_LIGHT_YELLOW,HIGH);
  return false;
}

/**************************************************************************/
/*!
    @brief     get the fingerprint statu
    @return    return the the id found that matches the fingerprint stored on the sensor itself
    @see       https://www.adafruit.com/product/751
*/
/**************************************************************************/
int getFingerprintStatus() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  {
    return -1;
  }

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) {
    return -1;
  }

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  {
    Serial.println(iFingerprintReadAttempts);
    if(iFingerprintReadAttempts >= (NUMBER_OF_FAILED_ATTEMPTS-1)) {
      colorChange(80,25,0); // yellow on failed finger scan
      finger.end();
    }
    iFingerprintReadAttempts++;
    return -1;
  }
  
  // found a match!
  colorChange(0, 0, 50); // blue on pass
  Serial.print("Found ID #"); Serial.print(finger.fingerID); 
  //To identify confidence use finger.confidence;however, practically useless.
  return finger.fingerID; 
}



