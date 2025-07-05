#define TINY_GSM_MODEM_SIM800
#define ARDUINOJSON_USE_LONG_LONG 1

#include <Arduino.h>
#include <permaDefs.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SPIMemory.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <elapsedMillis.h>
#include <avr/sleep.h>
#include <TimeLib.h>
#include <time.h>
#include <SoftwareSerial.h>

SoftwareSerial gsmSerial(PIN_PF1, PIN_PF0);

// Definittions //

#define SerialMon Serial
#define SerialAT gsmSerial

// Library Definitions //

elapsedMillis mTime;
elapsedMillis Btime;
TinyGPSPlus gps;
SPIFlash flash(FCS);
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);
uint32_t lastReconnectAttempt = 0;

//*********** Setting Variables ***********//

// Mode Control variable //
int mainMode = 0;                   // switch Mode variable(0 = main, 1 = live tracking)

// GPS Control Variables //
unsigned int gpsTimeout = 120;      // GPS Timeout in seconds  *** USER CONFIG ***
unsigned int gpsFrequency = 5;              // GPS Frequency in Minutes *** USER CONFIG ***
int gpsHdop = 5;                    // GPS HDOP *** USER CONFIG ***
int sleepCounter = 0;               // Sleep Counter for live traccking mode

// GSM Control Variables //
unsigned int transmissionFrequency = 1440;     // Transmission Frequency in MINS

// GPS Storage Variables // 
double lat;
double lng;
unsigned int count; 

// Memory Variables //
unsigned long wAdd = 1;
unsigned long rAdd = 0;
unsigned int cnt = 0;

// Time Variables //
time_t last_act_trigger;
time_t mortality_trigger_time;       // Mortality mode is triggered after this time
time_t strtTime;
time_t currentTime;


// Boolean Variables //
bool activate = true;
bool ret;
bool act_mode = false;              // Activity Mode flag - shows if device is in activity mode
bool mortality = false;             // Mortality Flag - show inactivity
bool activity_enabled = false;      //** Activity Enabled Flag - 
bool rtc_int_triggered = false;
bool act_int_triggered = false;
bool wipe_memory = false;           //** Memory Wipe flag -
bool noSleepLt = false;

// Control Variables //
uint16_t pingSecondCounter;
uint16_t gpsSecondCounter;
uint16_t pingCounterTarget;
uint16_t gpsCounterTarget;

// //************************************************//
// //***************    FUNCTIONS    ****************//
// //************************************************//
void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

  RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 16384, resulting in 32.768kHz/16384 = 2Hz */
  | RTC_PITEN_bm;                       /* Enable PIT counter: enabled */
}

ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
  pingSecondCounter = pingSecondCounter + 1;
  gpsSecondCounter = gpsSecondCounter + 1;
  currentTime = currentTime +1;
}

boolean mqttConnect() {
  SerialMon.print(F("Connecting to "));
  SerialMon.print(broker);

  // Connect to MQTT Broker
  // boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect(id);
  mqtt.setKeepAlive(20000);
  mqtt.setSocketTimeout(20000);
  mqtt.setBufferSize(128);
  if (status == false) {
    SerialMon.println(F(" fail"));
    return false;
  }
  SerialMon.println(F(" success"));
  mqtt.subscribe(settingsSupTopic);
  return mqtt.connected();

}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print(F("Message arrived ["));
  SerialMon.print(topic);
  SerialMon.print(F("]: "));
  SerialMon.println();

  Serial.println(len);
  Serial.print(F("PLD:")); 
  for (int i = 0; i < len; i++)
  {
    Serial.print((char)payload[i]);
  }
  
  // char x[len+2];
  // strncpy(x, (char*)payload, len+2);
  // delay(3000);
  // String top = (char*)topic;
  // Serial.println(x);
  // Serial.println(F("DONE"));

  // if (top == "status")
  // {
  //   Serial.println(F("Status cmd received"));
  //   StaticJsonDocument<100> doc;

  //   DeserializationError error = deserializeJson(doc, x);

  //   if (error) {
  //       Serial.print(F("deserializeJson() failed: "));
  //       Serial.println(error.f_str());
  //       return;
  //   }
  //   // Serial.println((char*)doc["activate"]);

  //   if (doc["activate"] == true)
  //   {
  //       Serial.println(F("Activating system"));
  //       activate = true;
  //   }else
  //   {
  //       activate = false;
  //   }
  //   if (doc["wipe"] == true)
  //   {
  //     wipe_memory = true;
  //   }else{
  //     wipe_memory = false;
  //   }
  //   if (doc["ltr"] == true)
  //   {
  //     mainMode = 1;
  //   }else{
  //     mainMode = 0;
  //   }    
  // }

  // if (top == "settings")
  // {
  //   StaticJsonDocument<200> doc;

  //   DeserializationError error = deserializeJson(doc, x);

  //   if (error) {
  //       Serial.print(F("deserializeJson() failed: "));
  //       Serial.println(error.f_str());
  //       return;
  //   }else{
  //     bool x = doc["NEW"];
  //     if (x == true)
  //     {
  //       gpsHdop = doc["HDOP"]; // 5
  //       gpsFrequency = doc["GFRQ"]; // 1
  //       transmissionFrequency = doc["TFRQ"]; // 9
  //       gpsTimeout = doc["GTO"]; // 10

  //       Serial.println(gpsHdop);
  //       Serial.println(gpsFrequency);
  //       Serial.println(gpsTimeout);
  //       Serial.println(transmissionFrequency);

  //       if (!mqtt.connected())
  //       {
  //         mqttConnect();
  //         if(mqtt.publish(telemetryTopic, setRep)){
  //         Serial.println(F("Reset Settings"));
  //         }
  //       }else{
  //         if(mqtt.publish(telemetryTopic, setRep)){
  //         Serial.println(F("Reset Settings"));
  //         }
  //       }
  //     }else{
  //       Serial.println(F("No New Settings"));
  //     }         
  //   }
  // }
  
  // Serial.println(activate);
  // ret = true;  

}

void networkInit(){
  modem.init();
  String modemInfo = modem.getModemInfo();
  SerialMon.print(F("Modem Info: "));
  SerialMon.println(modemInfo);

  SerialMon.print(F("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(F("fail"));
    // delay(10000);
    return;
  }
  SerialMon.println(F("success"));

  if (modem.isNetworkConnected()) { 
    SerialMon.println(F("Network connected")); 

     // MQTT Broker setup
      mqtt.setServer(broker, 1883);
      mqtt.setCallback(mqttCallback);

      if (!modem.isGprsConnected()) {
          SerialMon.println(F("GPRS not connected!"));
          SerialMon.print(F("Connecting to "));
          SerialMon.println(apn);
          if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
            SerialMon.println(F(" fail"));
            delay(10000);
            return;
          }
        }

      if (modem.isGprsConnected()){ 
        SerialMon.println(F("GPRS reconnected")); 
      }
      if (!mqtt.connected()) {
        SerialMon.println(F("=== MQTT NOT CONNECTED ==="));
        // Reconnect every 10 seconds
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L) {
          lastReconnectAttempt = t;
          if (mqttConnect()){
            lastReconnectAttempt = 0;
          }
        }
        delay(100);
        return;
      }
  }else{
    SerialMon.println(F("No Network"));
  }  
}

void postMetaAttributes(){
  Serial.println(F("Posting Meta Attributes"));
  char a[100];
  // digitalWrite(RTC_PIN, HIGH);
  // time_t x = rtc.get();
  // digitalWrite(RTC_PIN, LOW);
  StaticJsonDocument<120> doc;
  doc[F("Battery")] = modem.getBattPercent();
  doc[F("Signal")] = modem.getSignalQuality();
  doc[F("Data")] = cnt;
  doc[F("id")] = (String)tag;
  doc[F("tts")] = now();
  serializeJson(doc, a);  
  Serial.println(a);
  if (!mqtt.connected())
  {
    Serial.println(F("Mqtt Disconnected.. Retrying"));
    mqttConnect();
    if(mqtt.publish(telemetryTopic, (char*)a)){
      Serial.println(F("Posted Meta Attributes"));
    } 
  }else{
    if(mqtt.publish(telemetryTopic, (char*)a)){
      Serial.println(F("Posted Meta Attributes"));
    }
  } 
}

void syncSettings(){
  Btime = 0;
  delay(100);
  if (!mqtt.connected())
  {
    Serial.println(F("Mqtt Disconnected.. Retrying"));
    mqttConnect();  
    mqtt.setKeepAlive(10000);
  }else{
    mqtt.subscribe(settingsSupTopic);
    if(mqtt.publish(telemetryTopic, setReq)){
    Serial.println(F("Requested Settings"));
    }
    if(mqtt.publish(telemetryTopic, stAlrt)){
    Serial.println(F("Sent Alert"));
    }
  }

  while (Btime <= 5000)
  {
      mqtt.loop();
      delay(100);                
  }
  
}

void syncltr(){
  Btime = 0;
  delay(100);
  if (!mqtt.connected())
  {
    Serial.println(F("Mqtt Disconnected.. Retrying"));
    mqttConnect();  
    mqtt.setKeepAlive(10000);
  }else{
    mqtt.subscribe(statusSubTopic);
    if(mqtt.publish(telemetryTopic, statReq)){
    Serial.println(F("Requested Settings"));
    }
  }

  while (Btime <= 5000)
  {
      mqtt.loop();
      delay(100);                
  }
}

void acqGPS(){
  digitalWrite(GPS_PIN, HIGH);
      do{ 
        while (Serial1.available() > 0)
        {
          if (gps.encode(Serial1.read()))
          {
            if (!gps.location.isValid())
            {
              Serial.println(F("Not Valid"));
            }else{
              Serial.println(gps.location.isUpdated());
              Serial.print("Location Age:");
              Serial.println(gps.location.age());
              Serial.print("Time Age:");
              Serial.println(gps.time.age());
              Serial.print("Date Age:");
              Serial.println(gps.date.age());
              Serial.print("Satellites:");
              Serial.println(gps.satellites.value());
              Serial.print("HDOP:");
              Serial.println(gps.hdop.hdop());
            }
          }
        }
      }while(!gps.location.isValid());
    if (gps.location.age() < 60000)
    {
      //pack data into struct
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
    if (gps.time.isValid())
    {
      setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
      time_t n = now();
      strtTime = n;
      currentTime = n;
      Serial.print(F("START TIME : ")); Serial.println(strtTime);
    }    
    digitalWrite(GPS_PIN, LOW);
}

void recGPS(){
  Btime = 0;
  digitalWrite(GPS_PIN, HIGH);
  Serial.println(gpsTimeout*1000);
  while (Btime <= (gpsTimeout*1000))
  {
    while (Serial1.available())
    {
      if (!gps.encode(Serial1.read()))
      {
        if (!gps.location.isValid())
        {
          // Serial.println(F("Acquiring"));
        }else{
          Serial.println(gps.location.isUpdated());
          Serial.print(F("Location Age:"));
          Serial.println(gps.location.age());
          Serial.print(F("Time Age:"));
          Serial.println(gps.time.age());
          Serial.print(F("Date Age:"));
          Serial.println(gps.date.age());
          Serial.print(F("Satellites:"));
          Serial.println(gps.satellites.value());
          Serial.print(F("HDOP:"));
          Serial.println(gps.hdop.hdop());
        }       
      }      
    }
    if (gps.hdop.hdop() < (double)gpsHdop && gps.location.age() < 1000 && gps.time.age() < 1000 && mTime > 3000)
    {
      break;
    }  
  }   
  
  digitalWrite(GPS_PIN, LOW);
  
  data dat;

  if (gps.location.age() < 60000)
  {
    //pack data into struct
    lat = gps.location.lat();
    lng = gps.location.lng();
    dat.lat = gps.location.lat();
    dat.lng = gps.location.lng();
  }else{
    // pack data into struct with lat long = 0
    dat.lat = 0;
    dat.lng = 0;
  }
    Serial.print(gps.date.day());Serial.print(gps.date.month());Serial.println(gps.date.year());
    // digitalWrite(RTC_PIN, HIGH);
    setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day (), gps.date.month (),gps.date.year());
    time_t x = now();
    dat.datetime = x;
    // digitalWrite(RTC_PIN, LOW);
    dat.locktime = mTime/1000;
    dat.hdop = gps.hdop.hdop();
    dat.act = act_mode;
    
    
    Serial.println(dat.datetime);
    Serial.println(dat.lat);
    Serial.println(dat.lng);
    Serial.println(dat.locktime);
    Serial.println(dat.hdop);


  if (flash.powerUp())
  {
    Serial.println(F("Powered Up"));
    delay(500);
    Serial.println((int)sizeof(dat));
    wAdd = flash.getAddress(sizeof(dat));
    Serial.println(wAdd);
    if (flash.writeAnything(wAdd, dat))
    {
      Serial.println(F("Write Successful"));
      cnt = cnt + 1;
    }else
    {
      Serial.println(F("Write Failed"));
      Serial.println(flash.error(VERBOSE));
    }     
  }else
  {
    Serial.println(F("Power Up Failed"));
  }   
  flash.powerDown();

}

void read_send(){ 
  data dat;
  char a[150];
  StaticJsonDocument<150> doc;

  if (flash.powerUp())
  {
    if (flash.readAnything(rAdd, dat))
    {
      // // dat.id = tag;
      // Serial.println(F("Read Successful"));
      // Serial.println(dat.datetime);
      // Serial.println(dat.hdop);
      // Serial.println(dat.lat);
      // Serial.println(dat.lng);
      // Serial.println(dat.locktime);
      // Serial.println(dat.act);
      doc[F("ts")] = (unsigned long long)dat.datetime*1000;
      doc[F("Lat")] = dat.lat;
      doc[F("Lng")] = dat.lng;
      doc[F("hdop")] = dat.hdop;
      doc[F("LT")] = dat.locktime; 
      doc[F("Count")] = cnt;
      doc[F("id")] = tag;

      
    }else
    {
      Serial.println(F("Read Failed"));
    }    
  }

  serializeJson(doc, a);
  Serial.println(a);
  if (!mqtt.connected())
  {
    mqttConnect();  
    Serial.println(F("Transmitting"));
    if(mqtt.publish(telemetryTopic, (char*)a)){
      Serial.println(F("MQTT SUCCESS"));
    }
    mqtt.loop();
  }else{
    Serial.println(F("Transmitting"));
    if(mqtt.publish(telemetryTopic, (char*)a)){
      Serial.println(F("MQTT SUCCESS"));
    }
  }
  delay(50);  
}

// void mortalityCheck(bool m){
//   if (m == true)
//   {
//     if (currentTime - (last_act_trigger + 86400) > 86400)
//     {
//       mortality = true; 
//     }    
//   }  
// }

// void risr(){
//   rtc_int_triggered = true;
//   detachInterrupt(RINT);
//   detachInterrupt(AINT1);
// }

// void aisr(){
//   act_int_triggered = true;
//   mortality = false;
//   detachInterrupt(AINT1);
//   detachInterrupt(RINT);
// }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600);
  gsmSerial.begin(9600);
  pinMode(GPS_PIN, OUTPUT);
  pinMode(GSM_PIN, OUTPUT);
  pinMode(RTC_PIN, OUTPUT);
  // Wire.swap(uint8_t(1));
  // Wire.usePullups();
  // Wire.begin();
  SPI.begin();

  Serial.print(F("Tag ID :")); Serial.println(tag);Serial.println();
  Serial.println(F("Initializing..."));
  
  //***************************************************//
  
// if(!acce.begin()){
//   Serial.println(F("Acc Error"));
//   delay(1000);
//   }else{
//   Serial.print(F("chip id : "));
//   Serial.println(acce.getID(),HEX);
//   }
//   acce.softReset();
//   acce.setRange(DFRobot_LIS2DW12::e4_g);
//   acce.setFilterPath(DFRobot_LIS2DW12::eLPF);
//   acce.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
//   acce.setWakeUpDur(/*dur = */2);
//   acce.setWakeUpThreshold(/*threshold = */0.3);
//   acce.setPowerMode(DFRobot_LIS2DW12::eContLowPwrLowNoise1_12bit);
//   acce.setActMode(DFRobot_LIS2DW12::eDetectAct);
//   acce.setInt1Event(DFRobot_LIS2DW12::eWakeUp);
//   acce.setDataRate(DFRobot_LIS2DW12::eRate_100hz);
//   if (activity_enabled == true)
//   {
//     attachInterrupt(digitalPinToInterrupt(AINT1), aisr, CHANGE);
//   }else{
//     detachInterrupt(AINT1);
//   }

//***************************************************//

  digitalWrite(GSM_PIN, HIGH);
  delay(5000);

  SerialAT.println(F("AT+CNETLIGHT=1"));
  Btime = 0;
  while (Btime < 2000)
  {
    if (SerialAT.available())
    {
      SerialMon.println(SerialAT.readString());
    }    
  }
  
  networkInit();
  if (modem.isNetworkConnected())
  {
    SerialMon.print(F("Connecting to "));
    SerialMon.print(broker);

    // Connect to MQTT Broker

    // Or, if you want to authenticate MQTT:
    boolean status = mqtt.connect((char*)&tag);
    mqtt.setKeepAlive(10000);

    if (status == false) {
      SerialMon.println(F(" fail"));
      // return false;
    }
    SerialMon.println(F("success"));
    mqtt.subscribe(statusSubTopic);    
  }else{
    Serial.println(F("No Network"));
  }
  if (mqtt.connected())
  {
     Serial.println(F("Transmitting"));
        if(mqtt.publish(telemetryTopic, (char*)statReq)){
          Serial.println(F("MQTT SUCCESS"));
        } 
  }
  
  Btime = 0;
  while (Btime <= 10000)
  {
    mqtt.loop();
    delay(100);     
  }
  
  postMetaAttributes();
  syncSettings();
  mqtt.disconnect();

  SerialAT.println(F("AT+CNETLIGHT=0"));
  Btime = 0;
  while (Btime < 2000)
  {
    if (SerialAT.available())
    {
      Serial.println(SerialAT.readString());
    }
    
  }

  SerialAT.println(F("AT&W"));
  Btime = 0;
  while (Btime < 2000)
  {
    if (SerialAT.available())
    {
      Serial.println(SerialAT.readString());
    }
    
  }
  digitalWrite(GSM_PIN, LOW);

//***************************************************//

  if(flash.powerUp()){
    Serial.println(F("Powered Up1"));
  }
  if(!flash.begin()){
    Serial.println(F("Starting Flash"));
    Serial.println(flash.error(VERBOSE));
  } 
  Serial.println(flash.getManID());
  if(flash.powerUp()){
    Serial.println(F("Powered Up"));
  }else{
    Serial.println(F("PWR UP Failed!"));
    Serial.println(flash.error(VERBOSE));
  }
  if (wipe_memory == true)
  {
    Serial.println(F("WIPING FLASH"));
    if(flash.eraseChip()){
    Serial.println(F("Memory Wiped"));  
    }else
    {
      Serial.println(flash.error(VERBOSE));
    }
  }else{
    rAdd = flash.getAddress(16);
    wAdd = flash.getAddress(16);
  }    
  if(flash.powerDown()){
    Serial.println(F("Powered Down"));
    digitalWrite(1, HIGH);
  }else{
    Serial.println(flash.error(VERBOSE));
  }

//***************************************************//
  
  if (activate == true)
  {
    // acqGPS();
    Serial.println(strtTime);
    // digitalWrite(RTC_PIN, HIGH);
    // rtc.set(strtTime);
    // Serial.println(rtc.get());
    delay(100);

    digitalWrite(GSM_PIN, HIGH);
    networkInit();
    if (modem.isNetworkConnected())
    {
      SerialMon.print(F("Connecting to "));
      SerialMon.print(broker);

      // Connect to MQTT Broker

      // Or, if you want to authenticate MQTT:
      mqttConnect();
      
    }else{
      Serial.println(F("No Network"));
    }
          
    data dat;
    char a[150];
    StaticJsonDocument<150> doc;
    doc[F("ts")] = strtTime;
    doc[F("Lat")] = lat;
    doc[F("Lng")] = lng;
    doc[F("hdop")] = 0;
    doc[F("LT")] = 0; 
    doc[F("Count")] = 0;
    doc[F("id")] = tag;
    serializeJson(doc, a);
    Serial.println(a);
    
    if (!mqtt.connected())
    {
      mqttConnect();  
      Serial.println(F("Transmitting"));
      if(mqtt.publish(telemetryTopic, (char*)a)){
        // Serial.println(F("MQTT SUCCESS"));
      }
      mqtt.loop();
    }else{
      Serial.println(F("Transmitting"));
      if(mqtt.publish(telemetryTopic, (char*)a)){
        Serial.println(F("MQTT SUCCESS"));
      }
    }
    delay(500);
    mqtt.disconnect();    
    digitalWrite(GSM_PIN, LOW);

    RTC_init();

  }else{

    Serial.println(F("Sleep Forever!"));

  }

  gpsCounterTarget = gpsFrequency*60;
  pingCounterTarget = transmissionFrequency*60;  

//***************************************************//
  
  Serial.println(F("SYSTEM READY"));
  Serial.flush();
//***************************************************//
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void loop() {
  // // put your main code here, to run repeatedly:
  // switch (mainMode)
  // {
  // case 0: // Normal Mode loop
  //   Serial.println(F("Mode 1"));
  //   mTime = 0;
  //   if (gpsSecondCounter >= gpsCounterTarget)
  //   {
  //     recGPS();
  //     gpsSecondCounter = 0;
  //     if ((mTime/1000) < (gpsFrequency*60))
  //     {
  //       gpsCounterTarget = (gpsFrequency*60)-(mTime/1000);
  //     }else{
  //       gpsCounterTarget = gpsFrequency*60;
  //     }
      
      
  //   }
  //   if (pingSecondCounter >= pingCounterTarget)
  //   {
  //     Serial.println(F("Ping"));
  //     digitalWrite(GSM_PIN, HIGH);
  //     networkInit();
  //     if (!mqtt.connected())
  //     {
  //       mqttConnect();
  //     }      
  //     postMetaAttributes(); 
  //     while (rAdd <= wAdd)
  //     {
  //       read_send();
  //       rAdd = rAdd + 19;
  //     }
  //     syncSettings();
  //     digitalWrite(GSM_PIN, LOW);    
      
  //     pingSecondCounter = 0;
  //     if ((mTime/1000) < (transmissionFrequency*60))
  //     {
  //       pingCounterTarget = (transmissionFrequency*60)-(mTime/1000);
  //     }else{
  //       pingCounterTarget = transmissionFrequency*60;
  //     }
  //   }
  //   Serial.print(gpsSecondCounter); Serial.println(pingSecondCounter);
  //   Serial.flush();
  //   sleep_cpu();
  //   break;

  // case 1: // Live Tracking Mode Loop    
  // RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */
  //   if (noSleepLt)
  //   {
  //     Serial.println(F("Live Mode"));
  //     digitalWrite(GPS_PIN, HIGH);
  //     digitalWrite(GSM_PIN, HIGH);
  //     mTime = 0;
  //     while (mTime <= 60000)
  //     {
  //       while (Serial1.available())
  //       {
  //         if (!gps.encode(Serial1.read()))
  //         {
  //           if (!gps.location.isValid())
  //           {
  //             Serial.println(F("Acquiring"));
  //           }else{
  //             Serial.println(gps.location.isUpdated());
  //             Serial.print(F("Location Age:"));
  //           }       
  //         }
  //         if (gps.hdop.hdop() < (double)gpsHdop && gps.location.age() < 1000 && gps.time.age() < 1000 && mTime > 3000)
  //         {
  //           lat = gps.location.lat();
  //           lng = gps.location.lng();
  //           setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
  //           break;
  //         }      
  //       }
  //     }    
      
  //     digitalWrite(GPS_PIN, LOW);
  //     SerialAT.println(F("AT"));
  //     Btime = 0;
  //     while (Btime < 2000)
  //     {
  //       if (SerialAT.available())
  //       {
  //         SerialMon.println(SerialAT.readString());
  //       }    
  //     }
  //     if (!modem.isNetworkConnected())
  //     {
  //       networkInit();
  //     }else{
  //       if (!modem.isGprsConnected()) {
  //         SerialMon.println(F("GPRS not connected!"));
  //         SerialMon.print(F("Connecting to "));
  //         SerialMon.println(apn);
  //         if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
  //           SerialMon.println(F(" fail"));
  //           delay(10000);
  //           return;
  //         }
  //       }
  //       if (!mqtt.connected())
  //       {
  //         mqttConnect();
  //       }
  //       char a[100];
  //       StaticJsonDocument<100> doc;
  //       doc[F("ts")] = (unsigned long long)now();
  //       doc[F("Lat")] = lat;
  //       doc[F("Lng")] = lng;
  //       doc[F("id")] = (String)tag;
  //       serializeJson(doc, a);
  //       if(mqtt.publish(telemetryTopic, (char*)a)){
  //         Serial.println(F("MQTT SUCCESS"));
  //       }
  //       mqtt.loop();
  //     }
  //     mqtt.disconnect();
  //     modem.gprsDisconnect();
  //     SerialAT.println(F("AT+CSCLK=2"));
  //     Btime = 0;
  //     while (Btime < 2000)
  //     {
  //       if (SerialAT.available())
  //       {
  //         SerialMon.println(SerialAT.readString());
  //       }    
  //     }
  //     noSleepLt = false;

  //   }else{
  //     if (sleepCounter < 60)
  //     {
  //       sleepCounter = sleepCounter + 1;
  //       Serial.println(sleepCounter);
  //       Serial.flush();
  //       sleep_cpu();
  //     }else{
  //       noSleepLt = true;
  //       sleepCounter = 0;
  //     }       
  //   }          
  //   break;
    
  // default:
  //   break;
  // }    
}