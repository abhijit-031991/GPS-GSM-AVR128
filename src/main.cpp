#define TINY_GSM_MODEM_SIM800
#define ARDUINOJSON_USE_LONG_LONG 1

#include <Arduino.h>
#include <permaDefs.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SPIMemory.h>
#include <ArduinoHttpClient.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <elapsedMillis.h>
#include <MCP79412RTC.h>
#include <DFRobot_LIS2DW12.h>
#include <avr/sleep.h>
#include <TimeLib.h>
#include <time.h>

// Definittions //

#define SerialMon Serial
#define SerialAT Serial2

// Library Definitions //

elapsedMillis mTime;
elapsedMillis Btime;
TinyGPSPlus gps;
SPIFlash flash(FCS);
MCP79412RTC rtc(false);
DFRobot_LIS2DW12_I2C acce;
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);
uint32_t lastReconnectAttempt = 0;

//*********** Setting Variables ***********//

// Mode Control variable //
int mainMode = 0;                   // switch Mode variable(0 = main, 1 = live tracking)

// GPS Control Variables //
int gpsTimeout = 60;                // GPS Timeout in seconds  *** USER CONFIG ***
int gpsFrequency = 10;               // GPS Frequency in Minutes *** USER CONFIG ***
int gpsHdop = 5;                    // GPS HDOP *** USER CONFIG ***

// GSM Control Variables //
int transmissionFrequency = 1;     // Transmission Frequency in Hours

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
time_t next_gps_wakeup;
time_t next_gsm_wakeup;
time_t currentTime;
time_t live_tracking_start;
time_t live_tracking_end;


// Boolean Variables //
bool activate;
bool ret;
bool act_mode = false;              // Activity Mode flag - shows if device is in activity mode
bool mortality = false;             // Mortality Flag - show inactivity
bool activity_enabled = false;      //** Activity Enabled Flag - 
bool rtc_int_triggered = false;
bool act_int_triggered = false;
bool wipe_memory = false;           //** Memory Wipe flag -

//************************************************//
//***************    FUNCTIONS    ****************//
//************************************************//

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print(F("Message arrived ["));
  SerialMon.print(topic);
  SerialMon.print(F("]: "));
  SerialMon.println();

  Serial.println(len);
  char x[len+2];
  strncpy(x, (char*)payload, len+2);
  delay(2000);
  String top = (char*)topic;
  Serial.println(x);
  Serial.println(F("DONE"));

  if (top == "status")
  {
    Serial.println(F("Status cmd received"));
    StaticJsonDocument<100> doc;

    DeserializationError error = deserializeJson(doc, x);

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
    }
    // Serial.println((char*)doc["activate"]);

    if (doc["activate"] == true)
    {
        Serial.println(F("Activating system"));
        activate = true;
    }else
    {
        activate = false;
    }
    if (doc["wipe"] == true)
    {
      wipe_memory = true;
    }else{
      wipe_memory = false;
    }
    
  }

  if (top == "settings")
  {
        StaticJsonDocument<100> doc;

        DeserializationError error = deserializeJson(doc, x);

        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }else{
          bool x = doc["NEW"];
          if (x == true)
          {
            gpsHdop = doc["HDOP"]; // 5
            gpsFrequency = doc["GFRQ"]; // 1
            transmissionFrequency = doc["TFRQ"]; // 9
            gpsTimeout = doc["GTO"]; // 10

            Serial.println(gpsHdop);
            Serial.println(gpsFrequency);
            Serial.println(gpsTimeout);
            Serial.println(transmissionFrequency);

            if (mqtt.connected())
            {
              if(mqtt.publish(telemetryTopic, setRep)){
              Serial.println(F("Reset Settings"));
              }
            }
            


          }else{
            Serial.println(F("No New Settings"));
          }
          
          
        }      
     
  }
  
  Serial.println(activate);
  ret = true;  

}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  // boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect((char*)&tag);
  mqtt.setKeepAlive(10000);

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe(settingsSupTopic);
  return mqtt.connected();

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
  digitalWrite(RTC_PIN, HIGH);
  time_t x = rtc.get();
  digitalWrite(RTC_PIN, LOW);
  StaticJsonDocument<120> doc;
  doc[F("Battery")] = modem.getBattPercent();
  doc[F("Signal")] = modem.getSignalQuality();
  doc[F("Data")] = cnt;
  doc[F("id")] = (String)tag;
  doc[F("tts")] = (uint32_t)x;
  serializeJson(doc, a);  
  Serial.println(a);
  if (!mqtt.connected())
  {
    Serial.println(F("Mqtt Disconnected.. Retrying"));
    mqttConnect(); 
  }else{
    if(mqtt.publish(telemetryTopic, (char*)a)){
      Serial.println(F("Posted Meta Attributes"));
    }
  } 
}

void syncSettings(){
  if (!mqtt.connected())
  {
      mqttConnect();
      mqtt.setKeepAlive(10000);
  }
  Btime = 0;
  delay(100);
  if (mqtt.connected())
  {
      mqtt.subscribe(settingsSupTopic);
      if(mqtt.publish(telemetryTopic, setReq)){
      Serial.println(F("Requested Settings"));
      }
  }

  while (Btime <= 5000)
  {
      mqtt.loop();
      delay(100);                
  }
  if (mqtt.connected())
  {
    if(mqtt.publish(telemetryTopic, stAlrt)){
    Serial.println(F("Sent Alert"));
    }
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
      Serial.print(F("START TIME : ")); Serial.println(strtTime);
    }    
    digitalWrite(GPS_PIN, LOW);
}

void recGPS(){
  mTime = 0;
  digitalWrite(GPS_PIN, HIGH);
  Serial.println(gpsTimeout*1000);
  while (mTime <= gpsTimeout*1000)
  {
    while (Serial1.available())
    {
      if (!gps.encode(Serial1.read()))
      {
        if (!gps.location.isValid())
        {
          Serial.println(F("Acquiring"));
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
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    dat.datetime = (uint32_t)now();
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
      doc[F("id")] = (String)tag;

      
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

void mortalityCheck(bool m){
  if (m == true)
  {
    if (currentTime - (last_act_trigger + 86400) > 86400)
    {
      mortality = true; 
    }    
  }  
}

void risr(){
  rtc_int_triggered = true;
  detachInterrupt(RINT);
  detachInterrupt(AINT1);
}

void aisr(){
  act_int_triggered = true;
  mortality = false;
  detachInterrupt(AINT1);
  detachInterrupt(RINT);
}

void validateAlarms(){
  time_t x = rtc.get();
  Serial.println(x);
  if (x > next_gps_wakeup)
  {
    Serial.println(F("GPS Alarm Missed"));
    next_gps_wakeup = x + 60;
    rtc.setAlarm(0, next_gps_wakeup);      
    rtc.enableAlarm(0, ALM_MATCH_DATETIME);
  }
  if (x > next_gsm_wakeup)
  {
    Serial.println(F("GSM Alarm Missed"));
    next_gsm_wakeup = x + 60;
    rtc.setAlarm(1, next_gps_wakeup);      
    rtc.enableAlarm(1, ALM_MATCH_DATETIME);
  }  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);
  pinMode(GPS_PIN, OUTPUT);
  pinMode(GSM_PIN, OUTPUT);
  pinMode(RTC_PIN, OUTPUT);
  Wire.swapModule(&TWI1);
  Wire.usePullups();
  Wire.begin();
  SPI.begin();

  Serial.print(F("Tag ID :")); Serial.println(tag);Serial.println();
  Serial.println(F("Initializing..."));

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
    // boolean status = mqtt.connect("GsmClientTest");

    // Or, if you want to authenticate MQTT:
    boolean status = mqtt.connect((char*)&tag);
    mqtt.setKeepAlive(10000);

    if (status == false) {
      SerialMon.println(F(" fail"));
      // return false;
    }
    SerialMon.println(F("success"));
    mqtt.subscribe(statusSubTopic);
    Btime =0;
    while (Btime < 1000)
    {
      mqtt.loop();
      delay(200);
    }
    
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
  while (Btime <= 5000)
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

  if(!acce.begin()){
  Serial.println("Communication failed, check the connection and I2C address setting when using I2C communication.");
  delay(1000);
  }else{
  Serial.print("chip id : ");
  Serial.println(acce.getID(),HEX);
  }
  acce.softReset();
  acce.setRange(DFRobot_LIS2DW12::e4_g);
  acce.setFilterPath(DFRobot_LIS2DW12::eLPF);
  acce.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  acce.setWakeUpDur(/*dur = */2);
  acce.setWakeUpThreshold(/*threshold = */0.3);
  acce.setPowerMode(DFRobot_LIS2DW12::eContLowPwrLowNoise1_12bit);
  acce.setActMode(DFRobot_LIS2DW12::eDetectAct);
  acce.setInt1Event(DFRobot_LIS2DW12::eWakeUp);
  acce.setDataRate(DFRobot_LIS2DW12::eRate_100hz);
  if (activity_enabled == true)
  {
    attachInterrupt(digitalPinToInterrupt(AINT1), aisr, CHANGE);
  }else{
    detachInterrupt(AINT1);
  }

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
    Serial.println("Powered Down");
    digitalWrite(1, HIGH);
  }else{
    Serial.println(flash.error(VERBOSE));
  }

//***************************************************//
  if (activate == true)
  {
    acqGPS();
    // strtTime = 1672052568;
    Serial.println(strtTime);
    next_gps_wakeup = strtTime + (gpsFrequency*60);
    
    next_gsm_wakeup = strtTime + (transmissionFrequency*3600);

    digitalWrite(RTC_PIN, HIGH);
    rtc.set(strtTime);
    Serial.println(rtc.get());
    delay(100);
    rtc.alarmPolarity(HIGH);
    rtc.setAlarm(0, next_gps_wakeup);
    rtc.setAlarm(1, next_gsm_wakeup);
    rtc.enableAlarm(0, ALM_MATCH_DATETIME);
    rtc.enableAlarm(1, ALM_MATCH_DATETIME);
    digitalWrite(RTC_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(RINT), risr, CHANGE);
  }  

//***************************************************//
  Serial.println("SYSTEM READY");
  Serial.flush();
//***************************************************//
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void loop() {
  // put your main code here, to run repeatedly:

    if (rtc_int_triggered)
    {
      digitalWrite(RTC_PIN, HIGH);
      if(rtc.alarm(0)){
        Serial.println(F("ALARM 0"));
        currentTime = rtc.get();
        next_gps_wakeup = currentTime + gpsFrequency*60;
        rtc.setAlarm(0, next_gps_wakeup);      
        rtc.enableAlarm(0, ALM_MATCH_DATETIME);
        recGPS();
      }
      //************************************************************//
      if(rtc.alarm(1)){
        Serial.println(F("ALARM 1"));
        Serial.println(rtc.get());
        delay(10);
        currentTime = rtc.get();
        next_gsm_wakeup = currentTime + transmissionFrequency*3600;
        rtc.setAlarm(1, next_gsm_wakeup); 
        rtc.enableAlarm(1, ALM_MATCH_DATETIME);
        digitalWrite(GSM_PIN, HIGH);
        networkInit();
        mqttConnect();
        postMetaAttributes(); 
        while (rAdd < wAdd)
        {
          read_send();
          rAdd = rAdd + 16;
        }
        syncSettings();
        digitalWrite(GSM_PIN, LOW);
      }
      validateAlarms();
      digitalWrite(RTC_PIN, LOW);
    }
    attachInterrupt(digitalPinToInterrupt(RINT), risr, CHANGE);
    Serial.flush();
    sleep_cpu();    
}