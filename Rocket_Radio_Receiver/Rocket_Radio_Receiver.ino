/* Rocket Radio Receiver
 *  Receives arbitrary text data from a RF69 packet radio transmitter and writes it to an SD card on I2C.
 *  
 *  Adapted by Jason Goodman from code by Felix Rusu felix@lowpowerlab.com
 */

/*
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses></http:>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************/

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69registers.h>
#include <SPI.h>
#include <SD.h>

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     100  //the same on all nodes that talk to each other
#define NODEID        1  

//Match frequency to the hardware version of the radio on your Feather
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY      RF69_915MHZ
#define ENCRYPTKEY     "rocketEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW    true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   9600

/* for Feather M0  */
#define RFM69_CS      8
#define RFM69_IRQ     3
#define RFM69_IRQN    3  // Pin 3 is IRQ 3!
#define RFM69_RST     4
#define LED           13  // onboard blinky

//#define LED           0 //use 0 on ESP8266

int16_t packetnum = 0;  // packet counter, we increment per xmission

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

File dataFile;  // SD card file

const int SDSelect = 10; // SD card chip select pin

void blink_fail() {  // Blink the LED if something's wrong
  while(1) {
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
  }
}

void sdenable() {
  digitalWrite(RFM69_CS,HIGH);
  digitalWrite(SDSelect,LOW);
}

void sddisable() {
  digitalWrite(SDSelect,HIGH);
}

void setup() {
  delay(1000); // We don't want to wait for serial console, but give it some time after plugin,
              // in case it's there
  Serial.begin(SERIAL_BAUD);

  Serial.println("Feather RFM69HCW Receiver");
  
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  
  // Initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  radio.encrypt(ENCRYPTKEY);


  pinMode(SDSelect, OUTPUT);
  pinMode(LED, OUTPUT);

  Serial.print("\nListening at ");
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");

  // see if the card is present and can be initialized:
  if (!SD.begin(SDSelect)) {
    Serial.println("Card failed, or not present");
    blink_fail();
    // don't do anything more:
    return;
  }
  Serial.println("SD card initialized.");
  dataFile = SD.open("datalog.csv", O_CREAT | O_WRITE);
  if (!dataFile) {
    Serial.println("File open failed");
    blink_fail();
    return;
  }

  String header = "Time (s), \tPressure (Pa), Temperature (C), \tAx (m/s2), Ay (m/s2), Az(m/s2), Amag(m/s2), \tVbat (V), Signal (dBm)";
  Serial.print(header);
  dataFile.println(header);
  sddisable();  // Stop talking to SD
}

void loop() {
  //check if something was received (could be an interrupt from the radio)
  if (radio.receiveDone())
  {
    //check if sender wanted an ACK
    if (radio.ACKRequested())
    {
      radio.sendACK();
    }
    Blink(LED, 5, 1); //blink LED 3 times, 5ms between blinks
    //print message received to serial
    Serial.print((char*)radio.DATA);
    Serial.print(", ");Serial.println(radio.RSSI);
    Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
    //write message received to SD card
    sdenable();  // Start talking to SD
    dataFile.print((char*)radio.DATA);
    dataFile.print(", ");dataFile.println(radio.RSSI);
    dataFile.flush();    
    sddisable();  // Stop talking to SD
  }
  radio.receiveDone(); //put radio in RX mode
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
