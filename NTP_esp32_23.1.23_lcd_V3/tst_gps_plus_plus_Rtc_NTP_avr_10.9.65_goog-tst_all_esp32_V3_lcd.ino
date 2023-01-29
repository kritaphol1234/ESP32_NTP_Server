#define FOR_WINDOWS true

#include <TinyGPSPlus.h>        // include TinyGPS++ library
#include <TimeLib.h>          // include Arduino time library
//#include <SoftwareSerial.h>   // include software serial library
#include "RTClib.h"
#include <SPI.h>                  // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h> 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  
#include <WiFi.h>
#include "esp_bt.h"

RTC_DS3231 rtc;
DateTime rtcNow;  
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 16, 2);
 
//#define S_RX    2   // define software serial RX pin
//#define S_TX    6   // define software serial TX pin
 
//SoftwareSerial SoftSerial(S_RX, S_TX);   // configure SoftSerial library

#define NTP_PORT 123

byte mac[] = {                    // LM: Substitute fake MAC address associated with IP
//0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
  0x02, 0xB6, 0x7E, 0x00, 0x00, 0x02 };
//#define time_offset   25200 
IPAddress ip(192, 168, 3, 250);

static const int NTP_PACKET_SIZE = 48;

// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE]; 
#define LED 2
#define cs 5
// An Ethernet UDP instance 
EthernetUDP Udp;
#define MAXUINT32 4294967295.
//#define time_offset   0 
char Time[]  = "00:00:00, ";
//char Time[]  = "TIME: 00:00:00";
//char Date[]  = "DATE: 00-00-2000, ";
//char Time[]  = "00:00:00";
char Date[]  = "00/00/2000, ";
byte last_second, Second, Minute, Hour, Day, Month, hundredths;
short Year;
byte PR;
int ct = 0;

short int SAT;
unsigned long myTime;
unsigned long updateRTC;
unsigned long lcdup;
unsigned long lcdtime;
unsigned long omstime;
double Lat;
double Lng;
String str;
char myLat[10];
char myLng[11];
const unsigned long seventyYears = 2208988800UL;
uint32_t timestamp, tempval;
unsigned long lastGPSsync = 0;
int milliseconds;
int looptime;
int olooptime;
uint32_t fractionalSecond;
bool NTP_Service = false;
bool RTC_Update = false;
bool GPS_available = false;
bool LED_State = false;

byte stick[] = {
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000
};
void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  lcd.begin();
  lcd.backlight();
  lcd.createChar(0, stick);
  lcd.print("  Server START  ");
  Serial.begin(9600);
  Ethernet.init(cs);
  Ethernet.begin(mac, ip);
  Udp.begin(NTP_PORT);
  pinMode(LED, OUTPUT);
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.setSleep(true);
  btStop();
   if (! rtc.begin()) {
    lcd.setCursor(0 ,0);
    lcd.print(" RTC Fail !");
   // Serial.println("Couldn't find RTC");
   // Serial.flush();
    while (1) delay(10);
   }
  // lcd.setCursor(1, 2);
     lcd.setCursor(0, 1);
   lcd.print("TIME-SERVER v.2");
   delay(500);
}

void loop() {
  looptime = micros() - olooptime;
  olooptime = micros();
   if(omstime !=  millis()){
   
   if(ct > 11000) ct = 0;
    omstime = millis();
    ct++;
    }
   
if(ct == 1 
|| ct == 500
|| ct == 1000
|| ct == 1500
|| ct == 2000
|| ct == 2500
|| ct == 3000
|| ct == 3500
|| ct == 4000
|| ct == 4500
|| ct == 5000
|| ct == 5500
|| ct == 6000
|| ct == 6500
|| ct == 7000
|| ct == 7500
|| ct == 8000){
 
  LCDPrint();
}
 
if(ct == 8500) LCDPrint2();


if(GPS_available == false || gps.date.isValid() == false || gps.time.isValid() == false) NTP_Service = false;
if(GPS_available == true && gps.date.isValid() == true && gps.time.isValid() == true) NTP_Service = true;
    

   if(NTP_Service == true) processNTP();
      // put your main code here, to run repeatedly:
while (Serial.available() > 0){
     if(gps.encode(Serial.read())){
      
//if(LED_State == true){
//    LED_State = false;
//    digitalWrite(LED, LOW);
//  }else{
//   LED_State = true;
//    digitalWrite(LED, HIGH); 
//    }

//if(NTP_Service == true && GPS_available == true && RTC_Update == true){
  //digitalWrite(LED_STATUS, HIGH);
  //}else{
    //digitalWrite(LED_STATUS, LOW);
    //}
   // if(NTP_Service == true){
     //   processNTP();
  //  }
    
      // get time from GPS module
      
 
      // get date from GPS module
      if (gps.date.isValid()){
        Day   = gps.date.day();
        Month = gps.date.month();
        Year  = gps.date.year();
      }
      
     // get date from GPS module
 if (gps.location.isValid()){
   
    str = String(gps.location.lat(), 6);
    str.toCharArray(myLat, 10);

     
    str = String(gps.location.lng(), 6);
    str.toCharArray(myLng, 11);
}  
if (gps.time.isValid()){
        Hour   = gps.time.hour();
        Minute = gps.time.minute(); 
        Second = gps.time.second();
       // Centisecond = gps.time.centisecond();
      }

      if(last_second != gps.time.second()){  // if time has changed
        last_second = gps.time.second();

             // set current UTC time
        //setTime(Hour, Minute, Second, Day, Month, Year);

        //DateTime rtctime (Year, Month, Day, Hour, Minute, Second);
          DateTime rtctime (Year, Month, Day, Hour, Minute, Second);
  if(gps.time.isValid() && gps.date.isValid()){
    GPS_available = true;
    }else{
      GPS_available = false;
      }

     if(RTC_Update == false && GPS_available == true){ 
        rtc.adjust(rtctime);
        NTP_Service = true;
        RTC_Update = true;
     
     }
    if(myTime - updateRTC >= 100000 && GPS_available == true ){ //update every 1 days
        rtc.adjust(rtctime);
  updateRTC = myTime;
  
  Ethernet.init(cs);
  Ethernet.begin(mac,ip);
  Udp.begin(NTP_PORT);
  
  }

  
  SAT = gps.satellites.value();
  /*
        // add the offset to get local time
       // adjustTime(time_offset);
       
 Serial.print(Year);
 Serial.print("/");
 Serial.print(Month);
  Serial.print("/");
 Serial.print(Day);
  Serial.print("  ");\
  if(Hour < 10)Serial.print("0");
 Serial.print(Hour);
 Serial.print(":");
 if(Minute < 10)Serial.print("0");
 Serial.print(Minute);
  Serial.print(":");
 if(Second < 10)Serial.print("0");
 Serial.print(Second);
 Serial.print(".");
 //Serial.print(Centisecond);

//====================Read-RTC================//
DateTime now = rtc.now();
  Serial.print("  ");
  Serial.print(now.year());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.day());
  Serial.print("  ");\
  if(now.hour() < 10)Serial.print("0");
 Serial.print(now.hour());
 Serial.print(":");
 if(now.minute() < 10)Serial.print("0");
 Serial.print(now.minute());
  Serial.print(":");
 if(now.second() < 10)Serial.print("0");
 Serial.print(now.second());
 Serial.print("  ");
 Serial.print("GPS_available = ");
 Serial.print(GPS_available);
 Serial.print(" RTC_Update = ");
 Serial.print(RTC_Update);
 Serial.print(" gpstime = ");
 Serial.print(gps.time.isValid());
 Serial.print(" gpsdate = ");
 Serial.println(gps.date.isValid());
 
// Serial.print(".");
// Serial.println(now.centisecond());       
/*
        if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.println(gps.location.lng(), 6);
  }
  else
  {
    Serial.println(F("INVALID"));
  }
    */
    
      }
     }
    }
    /*
 ct = ct++;
if(ct == 1){
  lcd.begin();
  lcd.backlight();
  lcd.print("start void 1      ");//LCDPrint();
}
 
if(ct == 5000){
  lcd.begin();
  lcd.backlight();
  lcd.print("start void 2      ");// LCDPrint2();
}

if(ct > 10000) ct = 0;
    
   */
 myTime = millis();
  }
 void processNTP() {
//Serial.println("NTP_service_start");
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if(packetSize)
  {
    Udp.read(packetBuffer,NTP_PACKET_SIZE);
    IPAddress Remote = Udp.remoteIP();
    int PortNum = Udp.remotePort(); 
//==================Sart_NTP_Protocol=======================//
 packetBuffer[0] = 0b00100100;   // LI, Version, Mode
    // Have to spoof stratum 1 because Galleon test client interprets stratum 2 ID
    // as a date/time (not IP)
    packetBuffer[1] = 1 ;   // stratum (GPS)
//  packetBuffer[1] = 2 ;   // stratum (RTC)
    packetBuffer[2] = 6 ;   // polling minimum (64 seconds - default)
//  packetBuffer[3] = 0xFA; // precision (reference sketch - ~15 milliseconds)
    packetBuffer[3] = 0xF7; // precision (2^-9 ~2 milliseconds)
//  packetBuffer[3] = 0x09; // precision (2^9 Testing)

    packetBuffer[7] = 0;    // root delay
    packetBuffer[8] = 0;
    packetBuffer[9] = 8;
    packetBuffer[10] = 0;

    packetBuffer[11] = 0;   // root dispersion
    packetBuffer[12] = 0;
    packetBuffer[13] = 0xC;
    packetBuffer[14] = 0;

    readRTC();              // Assume succeeds
    // readRTC() has cracked date/time and timestamp
    // No additional parsing required here
    
                           // Reference identifier (for Stratum 1 type)
    packetBuffer[12] = 82; //"R";
    packetBuffer[13] = 84; //"T";
    packetBuffer[14] = 67; //"C";
    packetBuffer[15] = 0;  //"0";
/*
    packetBuffer[12] = 192; // IP address of synchronization source
    packetBuffer[13] = 168; // (Test client from Galleon Systems interprets as date/time)
    packetBuffer[14] = 1;   //
    packetBuffer[15] = 225; //
*/
    // Reference timestamp
    tempval = timestamp;
    packetBuffer[16] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[17] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[18] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[19] = (tempval) & 0xFF;

    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;


    // Originate timestamp from incoming UDP transmit timestamp
    packetBuffer[24] = packetBuffer[40];
    packetBuffer[25] = packetBuffer[41];
    packetBuffer[26] = packetBuffer[42];
    packetBuffer[27] = packetBuffer[43];
    packetBuffer[28] = packetBuffer[44];
    packetBuffer[29] = packetBuffer[45];
    packetBuffer[30] = packetBuffer[46];
    packetBuffer[31] = packetBuffer[47];

    // Receive timestamp
    tempval = timestamp;    // Same as reference timestamp
    packetBuffer[32] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[33] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[34] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[35] = (tempval) & 0xFF;

    packetBuffer[36] = 0;
    packetBuffer[37] = 0;
    packetBuffer[38] = 0;
    packetBuffer[39] = 0;

    // Transmit timestamp
    packetBuffer[40] = (tempval >> 24) & 0XFF;
    tempval = timestamp;
    packetBuffer[41] = (tempval >> 16) & 0xFF;
    tempval = timestamp;
    packetBuffer[42] = (tempval >> 8) & 0xFF;
    tempval = timestamp;
    packetBuffer[43] = (tempval) & 0xFF;

#if FOR_WINDOWS
    // LM: Fractional second - Use 0 with Windows client until issue resolved
    packetBuffer[44] = 0;
    packetBuffer[45] = 0;
    packetBuffer[46] = 0;
    packetBuffer[47] = 0;
#else
    // LM: Fractional second - Test NTP clients accept the following, but Windows does not
    tempval = fractionalSecond;
    packetBuffer[44] = (tempval >> 24) & 0xFF;
    tempval = fractionalSecond;
    packetBuffer[45] = (tempval >> 16) & 0xFF;;
    tempval = fractionalSecond;
    packetBuffer[46] = (tempval >> 8) & 0xFF;;
    tempval = fractionalSecond;
    packetBuffer[47] = (tempval) & 0xFF;;
#endif
//==================end_NTP_Protocol=======================//
    Udp.beginPacket(Remote, PortNum);
    Udp.write(packetBuffer,NTP_PACKET_SIZE);
    Udp.endPacket();
    
  }
}
void readRTC() {
  rtcNow = rtc.now();
   long delta = millis() - lastGPSsync;
  // To do: Handle rollover rigorously - Next is placeholder
  //        Or power-cycle the Arduino occasionally (before 50 days)
  if (delta < 0) {          // Rollover has occurred
                            // Constant below is 2^32 - 1
    delta = millis() + (4294967295 - lastGPSsync);
  }
  milliseconds = delta % 1000;
  // I can't see where hundredths were used in the reference sketch
  hundredths = milliseconds / 10;
  timestamp = rtcNow.unixtime() + seventyYears;   // 1900 Epoch
  // Compute fractional seconds
  fractionalSecond = ((double) milliseconds / 1000.) * MAXUINT32;
}

void LCDPrint() {
 // if(i == 10000) sli = sli++;
//  switch(sli){
 //   case 1:
// lcd.setCursor(0 ,0);
// lcd.print(looptime);
// lcd.print("     ");
DateTime now = rtc.now(); 
  lcd.setCursor(0 ,0);
  if(now.day() < 10)lcd.print("0");
  lcd.print(now.day());
  lcd.print("-");
  
  if(now.month() < 10)lcd.print("0");
  lcd.print(now.month());
  lcd.print("-");

  lcd.print(now.year());
  lcd.print(" ");
  
  lcd.setCursor(11, 0);
  lcd.print("| ");
 // lcd.write(0);
  lcd.print("SAT    ");
  
  lcd.setCursor(0 ,1);
  if (now.hour() < 10)lcd.print("0");
 lcd.print(now.hour());
 lcd.print(":");
 if(now.minute() < 10)lcd.print("0");
 lcd.print(now.minute());
  lcd.print(":");
 if(now.second() < 10)lcd.print("0");
 lcd.print(now.second());
 lcd.print("   ");
lcd.setCursor(11, 1);
   lcd.print("|  ");
  // lcd.write(0);
  if(SAT < 10) lcd.print("0");
  lcd.print(SAT);
  lcd.print(" ");
}
 //  break;
void LCDPrint2(){
   
lcd.setCursor(0, 0);
if(NTP_Service == true) lcd.print("Time Service:OPR"); 
if(NTP_Service == false) lcd.print("System Fail !   ");

lcd.setCursor(0, 1);
lcd.print(" ");
lcd.print(ip);
lcd.print("  ");
}  
  
  
