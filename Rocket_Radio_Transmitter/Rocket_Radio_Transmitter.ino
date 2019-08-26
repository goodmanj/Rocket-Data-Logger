/* Rocketlogger
 *  
 *  Captures data from a BMP180 pressure sensor and a ADXL377 hi-g accelerometer 
 *  and transmits them via RFM69 packet radio.
 *  
 *  
 *  
 The circuit:
 * Feather M0 with RFM69: https://www.adafruit.com/products/3176
 * BMP180 on I2C
 * 
 * ADXL377 on analog pins A1, A2, A3 (X, Y, Z)
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <RFM69.h>    //Lowpowerlab RFM69, get it here: https://www.github.com/lowpowerlab/rfm69

//*********************************************************************************************
// *********** RADIO SETTINGS *************
//*********************************************************************************************
#define NETWORKID     100  // The same on all nodes that talk to each other
#define NODEID        2    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets

//Match frequency to the hardware version of the radio on your Feather
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "rocketEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

#define RETRIES 10    // How many times to retry transmission
#define RETRY_MS 10   // Time between retries

/* for Feather M0 Radio */
  #define RFM69_CS      8
  #define RFM69_IRQ     3
  #define RFM69_IRQN    3  // Pin 3 is IRQ 3!
  #define RFM69_RST     4

//***************** END RADIO SETTINGS ********************************************************

// **************** ACCELEROMETER CALIBRATION SETTINGS **************************
// Calibration offsets for accelerometer
// For 100 ms delay in analog_multiread
#define XCAL 21.3
#define YCAL 21.9
#define ZCAL 20.5
#define SCALEX 1.15
#define SCALEY 1.18
#define SCALEZ 1.17

// ********** Battery monitor
#define VBATPIN 9

#define LED 13

int16_t packetnum = 0;  // packet counter, we increment per xmission

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

const int chipSelect = 4;

int accelprecision=12;   // Precision for analog accelerometer.  It's OK if this is more than the
                            // Arduino is capable of.
int accelscale=1<<(accelprecision-1);

float analog_multiread(int pin) {  // Read analog input several times, take the average
  int n = 20; // Number of reads to average
  int accum=0;
  for(int i=0; i<n ;i++) {
    delayMicroseconds(100);  
    accum += analogRead(pin) - accelscale;  // Subtract off bias voltage
  }
  return float(accum)/n;
}

void blink_fail() {  // Blink the LED if something's wrong
  while(1) {
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
  }
}

void setup() {
//  while(!Serial);
  pinMode(10, OUTPUT);
  pinMode(LED, OUTPUT);
  // Open serial communications but don't count on it
  Serial.begin(9600);
  Serial.println("Rocket radio transmitter starting.");

  /* *********** Radio Setup *****************/
  Serial.println("Feather RFM69HCW Transmitter");
  
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);

  // Turn it up to 11
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  radio.encrypt(ENCRYPTKEY);
  
  pinMode(LED, OUTPUT);
  Serial.print("\nTransmitting at ");
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");

  
  /* Initialise the Pressure sensor */
  if(!bmp.begin(BMP085_MODE_ULTRALOWPOWER))  // maximum data rate, minimum precision
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    blink_fail();
    return;
  }

  analogReference(AR_EXTERNAL);  // Use external reference for better precision
  analogReadResolution(accelprecision);  // High precision analog read
}

void loop() {
   
  float press_Pa, temperature_C;  // Pressure in Pascals, temperature in C
  float time_sec;
  String dataString = "";
  char radiopacket[80];

  float fullscale = 200*9.81; // m/s^2
  
  float sensorX,sensorY,sensorZ;    // raw sensor readings
  float aX,aY,aZ,amag;  // m/s^2


  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);

  if (event.pressure)
  {
    time_sec = millis()/1000.;
    dataString += String(time_sec,3);
    dataString += ",";

    press_Pa = event.pressure * 100;
    dataString += String(press_Pa) + ",";
    
    /* Get the current temperature from the BMP085 */
    bmp.getTemperature(&temperature_C);
    dataString += String(temperature_C) +  ",";

    digitalWrite(LED, LOW);   // flash the LED off while getting accelerometer data

    /* Get accelerometer data */
    sensorX = analog_multiread(1);
    aX = SCALEX*fullscale*(sensorX -XCAL)/accelscale;
    sensorY = analog_multiread(2);
    aY = SCALEY*fullscale*(sensorY -YCAL)/accelscale;
    sensorZ = analog_multiread(3);
    aZ = SCALEZ*fullscale*(sensorZ -ZCAL)/accelscale;

    digitalWrite(LED, HIGH);     // LED back on

    amag = sqrt(aX*aX +aY*aY + aZ*aZ);
    
    dataString += String(aX) + ",";
    dataString += String(aY) + ",";
    dataString += String(aZ) + ",";
    dataString += String(amag) + ",";

    // Get battery voltage
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1<<accelprecision; // convert to voltage
    dataString += String(measuredvbat);
   
    dataString.toCharArray(radiopacket,80); // Radio wants char[], not String

    Serial.print("Sending ");
    Serial.print(radiopacket);
    
    //target node Id, message as string or byte array, message length
    if (radio.sendWithRetry(RECEIVER, radiopacket, strlen(radiopacket),RETRIES,RETRY_MS)) { 
      Serial.println(" - OK");
    } else {
      Serial.println(" - NOT RECEIVED");
      Blink(LED, 100, 4); //blink LED 4 times, 100ms between blinks
    }

  radio.receiveDone(); //put radio in RX mode to reduce power consumption
    
  }

  
}

void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i=0; i<loops; i++)
  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
