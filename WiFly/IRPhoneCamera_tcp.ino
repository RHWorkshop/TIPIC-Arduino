
/*
Based in part on Tom Waldock's WebTimeA1 WiFly Example code

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 Copyright GPL 2.1 Tom Waldock 2011,2012
 */
/*
 
 Aim your browser at your WiFly address for a simple UTC time report.
 Add /irdata to the URL to get a single reading from the sensor.
 Add /eeprom to the URL to get a dump of the whole EEPROM from the MLX90620 Sensor.
 Add /getdata to the URL to get a dump of the whole EEPROM then continous 
 

Andy Rawson 2012
*/

#include <Arduino.h>
#include <WProgram.h>
#include <Time.h>
#include <SoftwareSerial.h>
#include <Streaming.h>
#include <PString.h>
#include <WiFlySerial.h>
#include "MemoryFree.h"
#include <i2cmaster.h>

// Set these to your local values
#define MY_WIFI_SSID "mySSID"
#define MY_WIFI_PASSPHRASE "myWiFiPassCode"

#define MY_WIFI_SSID2 "mySSID2"
#define MY_WIFI_PASSPHRASE2 "myWiFiPassCode2"

#define MY_NTP_SERVER "nist1-la.ustiming.org"

// Connect the WiFly TX pin to the Arduino RX pin  (Transmit from WiFly-> Receive into Arduino)
// Connect the WiFly RX pin to the Arduino TX pin  (Transmit from Arduino-> Receive into WiFly)
// 
// Connect the WiFly GND pin to an Arduino GND pin
// Finally, connect the WiFly BATT pin to the 3.3V pin (NOT the 5v pin)

#define ARDUINO_RX_PIN  2
#define ARDUINO_TX_PIN  3


prog_char s_WT_SETUP_000[] PROGMEM = "Arduino Rx Pin (connect to WiFly Tx):";  
prog_char s_WT_SETUP_001[] PROGMEM = "Arduino Tx Pin (connect to WiFly Rx):";  
prog_char s_WT_SETUP_01[] PROGMEM = "set u m 0x1";
prog_char s_WT_SETUP_02[] PROGMEM = "set comm remote 0";
prog_char s_WT_SETUP_03[] PROGMEM = "set comm idle 30";
prog_char s_WT_SETUP_04[] PROGMEM = "set comm time 1000";
prog_char s_WT_SETUP_05[] PROGMEM = "set comm size 128";
prog_char s_WT_SETUP_06[] PROGMEM = "set comm match 0x9";
prog_char s_WT_SETUP_07[] PROGMEM = "time";
prog_char s_WT_STATUS_SENSORS[] PROGMEM = "show q 0x177 ";
prog_char s_WT_STATUS_TEMP[] PROGMEM = "show q t ";
prog_char s_WT_STATUS_RSSI[] PROGMEM = "show rssi ";
prog_char s_WT_STATUS_BATT[] PROGMEM = "show battery ";
prog_char s_WT_MSG_JOIN[] PROGMEM = "Credentials Set, Joining ";
prog_char s_WT_MSG_LEAVE[] PROGMEM = "Leaving current WiFly ... ";
prog_char s_WT_MSG_START_WEBTIME[] PROGMEM = "Starting TIPIC - Please wait. ";
prog_char s_WT_MSG_RAM[] PROGMEM = "RAM :";
prog_char s_WT_MSG_START_WIFLY[] PROGMEM = "Started WiFly, :";
prog_char s_WT_MSG_WiFly[] PROGMEM = "Initial WiFly Settings :";
prog_char s_WT_MSG_APP_SETTINGS[] PROGMEM = "Configure TIPIC Settings...";
prog_char s_WT_MSG_AIM_BROWSER[] PROGMEM = "Aim your browser at ( /irdata and /eeprom optional): ";
prog_char s_WT_MSG_HTTP_READY[] PROGMEM = "Ready for HTTP Requests. ";
prog_char s_WT_MSG_TIMEOUT[] PROGMEM = "Timed out waiting for next connection. ";
prog_char s_WT_HTML_HEAD_01[] PROGMEM = "HTTP/1.1 200 OK \r ";
prog_char s_WT_HTML_HEAD_02[] PROGMEM = "Content-Type: text/html;charset=UTF-8\r ";
prog_char s_WT_HTML_HEAD_03[] PROGMEM = " Content-Length: ";
prog_char s_WT_HTML_HEAD_04[] PROGMEM = "Connection: close \r\n\r\n ";
prog_char s_WT_HTML_HEAD_05[] PROGMEM = "<META HTTP-EQUIV=\"REFRESH\" CONTENT=\"30\">";

#define IDX_WT_SETUP_000 0
#define IDX_WT_SETUP_001 1
#define IDX_WT_SETUP_01 IDX_WT_SETUP_001 +1
#define IDX_WT_SETUP_02 IDX_WT_SETUP_01 +1
#define IDX_WT_SETUP_03 IDX_WT_SETUP_02 +1
#define IDX_WT_SETUP_04 IDX_WT_SETUP_03 +1
#define IDX_WT_SETUP_05 IDX_WT_SETUP_04 +1
#define IDX_WT_SETUP_06 IDX_WT_SETUP_05 +1
#define IDX_WT_SETUP_07 IDX_WT_SETUP_06 +1

#define IDX_WT_TO_UDP_01 IDX_WT_SETUP_07 + 1
#define IDX_WT_TO_UDP_02 IDX_WT_TO_UDP_01 + 1
#define IDX_WT_TO_UDP_03 IDX_WT_TO_UDP_02 + 1
#define IDX_WT_TO_UDP_04 IDX_WT_TO_UDP_03 + 1

#define IDX_WT_FROM_UDP_01 IDX_WT_TO_UDP_04 + 1
#define IDX_WT_FROM_UDP_02 IDX_WT_FROM_UDP_01 + 1
#define IDX_WT_FROM_UDP_03 IDX_WT_FROM_UDP_02 + 1
#define IDX_WT_FROM_UDP_04 IDX_WT_FROM_UDP_03 + 1

#define IDX_WT_STATUS_SENSORS    IDX_WT_SETUP_07 +1
#define IDX_WT_STATUS_TEMP       IDX_WT_STATUS_SENSORS +1
#define IDX_WT_STATUS_RSSI       IDX_WT_STATUS_TEMP +1
#define IDX_WT_STATUS_BATT       IDX_WT_STATUS_RSSI +1

#define IDX_WT_MSG_JOIN          IDX_WT_STATUS_BATT +1
#define IDX_WT_MSG_LEAVE         IDX_WT_MSG_JOIN +1
#define IDX_WT_MSG_START_WEBTIME IDX_WT_MSG_JOIN +2
#define IDX_WT_MSG_RAM           IDX_WT_MSG_JOIN +3
#define IDX_WT_MSG_START_WIFLY   IDX_WT_MSG_JOIN +4
#define IDX_WT_MSG_WiFly          IDX_WT_MSG_JOIN +5
#define IDX_WT_MSG_AIM_BROWSER   IDX_WT_MSG_JOIN +6
#define IDX_WT_MSG_HTTP_READY    IDX_WT_MSG_JOIN +7
#define IDX_WT_MSG_TIMEOUT       IDX_WT_MSG_JOIN +8
#define IDX_WT_MSG_APP_SETTINGS  IDX_WT_MSG_JOIN +9

#define IDX_WT_HTML_HEAD_01      IDX_WT_MSG_APP_SETTINGS + 1
#define IDX_WT_HTML_HEAD_02      IDX_WT_HTML_HEAD_01 + 1
#define IDX_WT_HTML_HEAD_03      IDX_WT_HTML_HEAD_01 + 2
#define IDX_WT_HTML_HEAD_04      IDX_WT_HTML_HEAD_01 + 3
#define IDX_WT_HTML_HEAD_05      IDX_WT_HTML_HEAD_01 + 4


PROGMEM const char *WT_string_table[] = 	   
{   
  s_WT_SETUP_000,
  s_WT_SETUP_001,
  s_WT_SETUP_01,
  s_WT_SETUP_02,
  s_WT_SETUP_03,
  s_WT_SETUP_04,
  s_WT_SETUP_05,
  s_WT_SETUP_06,
  s_WT_SETUP_07,
  s_WT_STATUS_SENSORS,
  s_WT_STATUS_TEMP,
  s_WT_STATUS_RSSI,
  s_WT_STATUS_BATT,
  s_WT_MSG_JOIN,
  s_WT_MSG_LEAVE,
  s_WT_MSG_START_WEBTIME,
  s_WT_MSG_RAM,
  s_WT_MSG_START_WIFLY,
  s_WT_MSG_WiFly,
  s_WT_MSG_AIM_BROWSER,
  s_WT_MSG_HTTP_READY,
  s_WT_MSG_TIMEOUT,
  s_WT_MSG_APP_SETTINGS,
  s_WT_HTML_HEAD_01,
  s_WT_HTML_HEAD_02,
  s_WT_HTML_HEAD_03,
  s_WT_HTML_HEAD_04,
  s_WT_HTML_HEAD_05
};

#define REQUEST_BUFFER_SIZE 80
#define HEADER_BUFFER_SIZE 120 
#define BODY_BUFFER_SIZE 280
#define TEMP_BUFFER_SIZE 80

char bufRequest[REQUEST_BUFFER_SIZE];
char bufTemp[TEMP_BUFFER_SIZE];
char chMisc;
int iRequest = 0;
int iTrack = 0;
int loops = 1;
int eeloops = 0;


WiFlySerial WiFly(ARDUINO_RX_PIN ,ARDUINO_TX_PIN);


// Function for setSyncProvider
time_t GetSyncTime() {
  time_t tCurrent = (time_t) WiFly.getTime();
  WiFly.exitCommandMode();
  return tCurrent;
}

// GetBuffer_P
// Returns pointer to a supplied Buffer, from PROGMEM based on StringIndex provided.
// based on example from http://arduino.cc/en/Reference/PROGMEM

char* GetBuffer_P(const int StringIndex, char* pBuffer, int bufSize) { 
  strncpy_P(pBuffer, (char*)pgm_read_word(&(WT_string_table[StringIndex])), bufSize);  
  return pBuffer; 
}

void blinkLed(int times) {
       for (int i = 0; i < times; i++) {
       digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
       delay(100);               // wait for a second
       digitalWrite(13, LOW); 
       delay(100); 
       }
}

// Reconnects to a wifi network.
// DHCP is enabled explicitly.
// You may need to add the MAC address to your MAC filter list.
// Static IP settings available if needed.
boolean Reconnect() {

  WiFly.SendCommand(GetBuffer_P(IDX_WT_SETUP_01,bufTemp,TEMP_BUFFER_SIZE), ">",bufRequest, REQUEST_BUFFER_SIZE);
  WiFly.setDHCPMode(true);
  WiFly.SendCommand(GetBuffer_P(IDX_WT_SETUP_02,bufTemp,TEMP_BUFFER_SIZE),">",bufRequest, REQUEST_BUFFER_SIZE);
  Serial << "Leave current wifi network:" << WiFly.leave() << endl;
  // join
  WiFly.setPassphrase(MY_WIFI_PASSPHRASE);    
  Serial << GetBuffer_P(IDX_WT_MSG_JOIN,bufTemp,TEMP_BUFFER_SIZE) << MY_WIFI_SSID << endl;
  
  //Try joining the first SSID and try the second if the first f
  int joined;
  joined = WiFly.join(MY_WIFI_SSID);
  if (!joined) {
      blinkLed(5);
      WiFly.setPassphrase(MY_WIFI_PASSPHRASE2);    
      Serial << GetBuffer_P(IDX_WT_MSG_JOIN,bufTemp,TEMP_BUFFER_SIZE) << MY_WIFI_SSID2 << endl;
      joined = WiFly.join(MY_WIFI_SSID2);
  }
blinkLed(1);
  // Set NTP server, update frequency, 
  WiFly.setNTP(MY_NTP_SERVER); 
  WiFly.setNTP_Update_Frequency(" 15");
  // don't send *HELLO* on http traffic
  // close idle connections after n seconds
  // give enough time for packet data to arrive
  // make data packet size sufficiently large
  // send data packet when a \t appears in stream
  //  force time resync.

  // Configure application-specific settings

  Serial << GetBuffer_P(IDX_WT_MSG_APP_SETTINGS, bufTemp, TEMP_BUFFER_SIZE) << endl;
  for (int i = 0; i< 7 ; i++) {
    WiFly.SendCommand(GetBuffer_P(IDX_WT_SETUP_01 + i,bufTemp,TEMP_BUFFER_SIZE),">",bufRequest, REQUEST_BUFFER_SIZE);
  }

  setTime( WiFly.getTime() );
  delay(1000);
  setSyncProvider( GetSyncTime );

  // reboot if not working right yet.
  iTrack++;
  if ( iTrack > 5 ) {
    WiFly.reboot();
    iTrack = 0;
  }

}

    int edev = 0xA0; //eeprom device address
    int dev = 0xC0; //ram device address
    int eepromData[255]; //store the eeprom data

void setup(){
	Serial.begin(9600);
        pinMode(13, OUTPUT);
	//Serial.println("Setup...");

       //PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups

       Serial << GetBuffer_P(IDX_WT_MSG_START_WEBTIME,bufTemp,TEMP_BUFFER_SIZE) << endl << GetBuffer_P(IDX_WT_MSG_RAM,bufTemp,TEMP_BUFFER_SIZE) << freeMemory() << endl;

  WiFly.begin();
  Serial << GetBuffer_P(IDX_WT_MSG_START_WIFLY,bufTemp,TEMP_BUFFER_SIZE) <<  freeMemory() << endl;

  // get MAC
  Serial << "MAC: " << WiFly.getMAC(bufRequest, REQUEST_BUFFER_SIZE) << endl;

  Reconnect();
  //Serial << "DateTime:" << year() << "-" << month() << "-" << day() << " " << hour() << ":" << minute() << ":" << second() << endl;

  Serial << GetBuffer_P(IDX_WT_MSG_WiFly,bufTemp,TEMP_BUFFER_SIZE) << endl  
    << F("IP: ") << WiFly.getIP(bufRequest, REQUEST_BUFFER_SIZE) << endl
    << F("Netmask: ") << WiFly.getNetMask(bufRequest, REQUEST_BUFFER_SIZE) << endl
    << F("Gateway: ") << WiFly.getGateway(bufRequest, REQUEST_BUFFER_SIZE) << endl
    << F("DNS: ") << WiFly.getDNS(bufRequest, REQUEST_BUFFER_SIZE) << endl
    << F("RSSI: ") << WiFly.getRSSI(bufRequest, REQUEST_BUFFER_SIZE) << endl;
 memset (bufRequest,'\0',REQUEST_BUFFER_SIZE);

  // close any open connections
  WiFly.closeConnection();
  Serial << "After Setup RAM:" << freeMemory() << endl ;

  WiFly.exitCommandMode();
  //  WiFly.setDebugChannel( (Print*) &Serial);
 
       i2c_init(); //Initialise the i2c bus
       delay(10);
       EEPROMDump();
       SetOscTrim();
       SetConfig();
       
  blinkLed(2);
}

void loop(){
  
  WiFly.getDeviceStatus();
  // if not connected restart link
  while (! WiFly.isAssociated() ) {
    Reconnect();
  } // while not connected
  
    while ((chMisc = WiFly.read()) > -1) {
    Serial << chMisc;
  }
  
  Serial << GetBuffer_P(IDX_WT_MSG_HTTP_READY,bufTemp,TEMP_BUFFER_SIZE) << freeMemory() << endl ;
  blinkLed(3);  
  iRequest = WiFly.serveConnection();
  if (  iRequest  ) {
    memset (bufRequest,'\0',REQUEST_BUFFER_SIZE);
    //WiFly.bWiFlyInCommandMode = false;
    Serial << F("Connected ") << endl;
    // analyse request for GET;
    WiFly.ScanForPattern( bufRequest, REQUEST_BUFFER_SIZE, " HTTP/1.1", 1000 );
    Serial << F("GET request,  bytes: ") << strlen(bufRequest) << endl << bufRequest << endl;

    // read past the browser's header details
    while ((chMisc = WiFly.read()) > -1)
      Serial << chMisc;

    char* pHeader = (char*) malloc(HEADER_BUFFER_SIZE);
    char* pBody = (char*) malloc(BODY_BUFFER_SIZE);

    // Form header and body of response
    int i = 0;
    do  {
    MakeReponseBody(pBody,  bufRequest, REQUEST_BUFFER_SIZE);
    //MakeResponseHeader( pHeader,  pBody);

    // send reply.  note the \t, whch is the end-of-packet signal for wifly, as set in SET MATCH 0x09 above.
    
    //WiFly << pHeader << pBody << "\r\n\r\n" << "\t";

    WiFly << pBody << "\r\n\r\n" << "\t";
    delay(100);
    i++;
    //Serial.println(i);
    } while (i < loops);

    // give the web browser time to receive the data
    // NewSoftSerial will trickle data up to the WiFly in the background after print stmt completed.
    // settings are conservative ... more rapid responses possible by calcuguessing message length / (baudrate / bits/byte) x SomeFudgeFactor x milliseconds
    // baud bits/byte is >8 and < 10 by the time ecc figured in.
    // and subtract estimated time to send 'close' command (1000 ms?)
    // 
    int iDelay = ( ( (strlen(pHeader) + strlen(pBody))/ (9600/10) ) * 1000 ) - 1000;
    
    //delay( ( iDelay < 0 ? 0 : iDelay) )  ;

    //Serial << endl << F("Header:") << endl << pHeader << F("Body:") << pBody << endl;
    
    // close connection
    WiFly.closeConnection();
    
    // release memory back to pool - even if freeMemory() doesn't recognize it.
    // Note: release AFTER connection closed and writing completed.

    free(pHeader);
    free(pBody);
    pHeader = NULL;
    pBody = NULL;

  } else {
    // Timout - no clients.  Do stuff, then go back to waiting.
    // A larger timeout value would be good as well.
    Serial << GetBuffer_P(IDX_WT_MSG_TIMEOUT,bufTemp,TEMP_BUFFER_SIZE)
        << year() << "-" << month() << "-" << day() << " " << hour() << ":" << minute() << ":" << second() << endl;
  }  // if Open connection found.
}

void EEPROMDump(){
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    i2c_start_wait(edev+I2C_WRITE);
    //Serial.println("I2C Started");
    i2c_write(0x00);
    
    for (int eAddress = 0; eAddress < 256; eAddress++) { 
        // read
    i2c_rep_start(edev+I2C_READ);
    //Serial.print("Address ");
    //Serial.println(eAddress, HEX);
    data_low = i2c_readNak(); //Read 1 byte and then send ack
    //Serial.println(data_low, HEX);
    //data_high = i2c_readAck(); //Read 1 byte and then send ack
    //Serial.println(data_high, HEX);
   
    eepromData[eAddress] = data_low;
    //Serial.println((data_high<<8) + data_low,DEC);
    //Serial.print(data_high,HEX);
    }
    pec = i2c_readNak();
    i2c_stop();

    //GetData(0x91); //Compensation Pixel
    //GetData(0x92); //Config Register
    //GetData(0x93); //Oscillator Trimming Register
    
//    for (int i = 0; i < 256; i++) {
//      Serial.print(i);
//      Serial.print(",");
//      Serial.println(eepromData[i]);
//    }
    
}

void SetOscTrim() {
i2c_start_wait(dev+I2C_WRITE);
i2c_write(0x04);
i2c_write(0xAD);
i2c_write(0x57);
i2c_write(0x56);
i2c_write(0x00);
   i2c_stop();
}

void SetConfig() {
i2c_start_wait(dev+I2C_WRITE);
i2c_write(0x03);
//change both of the following to change refresh rate
//refresh rate 0xB9 1hz, 0xB8 2hz, 0xB7 4hz
i2c_write(0xB7);
//refresh rate 0x0E 1hz, 0x0D 2hz, 0x0C 4hz
i2c_write(0x0C);
i2c_write(0x1F);
i2c_write(0x74);
   i2c_stop();
}

int GetData(int address){

    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    //Serial.println(dev+I2C_WRITE);
    i2c_start_wait(dev+I2C_WRITE);
    //Serial.println("I2C Started");
    i2c_write(0x02);
    //Serial.println("0x02 Read Address Command sent");
    i2c_write(address);
    //Serial.println("Array Address sent");
    i2c_write(0x00);
    //Serial.println("0x00 Address Step sent");
    i2c_write(0x01);
    //Serial.println("0x01 Number of Reads sent");
   
    // read
    i2c_rep_start(dev+I2C_READ);
    //Serial.print("Address ");
    //Serial.println(address, HEX);
    data_low = i2c_readAck(); //Read 1 byte and then send ack
    //Serial.println(data_low, HEX);
    data_high = i2c_readAck(); //Read 1 byte and then send ack
    //Serial.println(data_high, HEX);
    pec = i2c_readNak();
    i2c_stop();
    return (data_high<<8) + data_low;
    //Serial.println("I2C Stopped");

    
}


// Make Response Body
// Based on GET request string, generate a response.
int MakeReponseBody( char* pBody,  char* pRequest, const int sizeRequest) {

  PString strBody( pBody, BODY_BUFFER_SIZE);

if ( strstr(pRequest, "/getdata" ) ) {
 // Send the EEPROM data then start sending the IR data   
  if (eeloops >= 9) {
    for (int i = 0; i < 64; i++) { 
        strBody <<  GetData(i) << ",";
    }
    strBody <<  GetData(144) << ",";
    strBody <<  GetData(145);

    //strBody << F("\r\n\r\n");

  } 
  else { 
    int startLoops = 0;
    int endLoops = 0;

    if (!eeloops) {
      loops = 10000;
      endLoops = 31;
      startLoops = 0;
      eeloops = 2;
    }  
    else {
      endLoops = (eeloops * 32) -1;
      startLoops = (eeloops -1) * 32;
      eeloops++;
    }    
    for (int i = startLoops; i < endLoops; i++) { 
    strBody <<  eepromData[i] << ",";
    }
    strBody <<  eepromData[endLoops];
    //strBody << F("\r\n\r\n");
  } 
}
  
  else if (strstr(pRequest, "/irdata" )){
    loops = 1;
    for (int i = 0; i < 64; i++) { 
        strBody <<  GetData(i) << ",";
    }
    strBody <<  GetData(144) << ",";
    strBody <<  GetData(145);
  }
  
  
  
  else if ( strstr(pRequest, "/eeprom" ) ) {   
    int startLoops = 0;
    int endLoops = 0;

    if (!eeloops) {
      loops = 8;
      endLoops = 31;
      startLoops = 0;
      eeloops = 2;
    }  
    else {
      endLoops = (eeloops * 32) -1;
      startLoops = (eeloops -1) * 32;
      eeloops++;
    }    
    for (int i = startLoops; i < endLoops; i++) { 
      strBody <<  eepromData[i] << ",";
    }
    strBody <<  eepromData[endLoops];
    //strBody << F("\r\n\r\n");
  }
  
  else {
    loops = 1;
    if ( strstr(pRequest,"/auto") ) {
      strBody << GetBuffer_P(IDX_WT_HTML_HEAD_05,bufTemp,TEMP_BUFFER_SIZE);
    }
    strBody << F("<html>Current request:") << pRequest << F("</br>Millis:") << millis() << F(" Micros:") << micros()
      << F("</br>DateTime:") << year() << "-" << month() << "-" << day() << " " << hour() << ":" << minute() << ":" << second()
        << F("</html>");
    // No calls back to WiFly command mode; hence no need to exit Command mode that wasn't entered.
  }
  
  return strBody.length();
}



// MakeResponseHeader
// Form a HTML header, including length of body.
int MakeResponseHeader( char* pHeader, char* pBody ) {

  PString strHeader( pHeader, HEADER_BUFFER_SIZE);
  // send a standard http response header    

  strHeader << GetBuffer_P(IDX_WT_HTML_HEAD_01,bufTemp,TEMP_BUFFER_SIZE)
    << GetBuffer_P(IDX_WT_HTML_HEAD_02,bufTemp,TEMP_BUFFER_SIZE)
      << GetBuffer_P(IDX_WT_HTML_HEAD_03,bufTemp,TEMP_BUFFER_SIZE) << (int) strlen(pBody) << " \r"
        << GetBuffer_P(IDX_WT_HTML_HEAD_04,bufTemp,TEMP_BUFFER_SIZE);

  return strHeader.length();
}


