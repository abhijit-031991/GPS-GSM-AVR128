////////////////////////////////////////////
/////      Firmware Definitions        /////
////////////////////////////////////////////

///// Device Definitions /////

// Device ID //

const uint16_t tag = 10101;
const uint8_t devType = 107;

// Firmware Version //
const float firmwareVersion = 2.0;

//// Pin Definitions ////

#define GPS_PIN PIN_PD4
#define GSM_PIN PIN_PA2
#define RTC_PIN PIN_PD7
#define RINT PIN_PD6
#define AINT1 PIN_PD4
#define AINT2 PIN_PD5
#define FCS PIN_PA7
#define NRFCS PIN_PC2

#define GSM_TX PIN_PF0
#define GSM_RX PIN_PF1

// Server Variables // 

const char statReq[] = "{\"id\":\"10101\", \"req\":1}";
const char setReq[] = "{\"id\":\"10101\", \"req\":2}";
const char setRep[] = "{\"id\":\"10101\", \"req\":3}";
const char stAlrt[] = "{\"id\":\"10101\", \"req\":4}";

const char* broker = "65.1.242.158";

const char* telemetryTopic = "telemetry";
const char* statusSubTopic = "status";
const char* settingsSupTopic = "settings";

//GPRS credentials //
const char apn[]      = "airtelgprs.com";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Structs //

struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    float hdop;
    bool act;
  };


