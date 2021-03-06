/*  KonaEvObd for Hyundai Kona EV + OBD Vgate iCar Pro BT4.0 
 *  Version: v4.01
  *   
  * SafeString by Matthew Ford: https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
  * Elmduino by PowerBroker2: https://github.com/PowerBroker2/ELMduino
  * https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
  * 
*/
#include "SafeString.h"
#include "ELMduino.h"
#include "EEPROM.h"
#include "Button.h"
#include "TFT_eSPI.h"
#include "BT_communication.h"
#include "Wifi_connection.h"
#include "FreeRTOS.h"
#include "HTTPClient.h"
#include "HTTPSRedirect.h"
#include "WiFiClientSecure.h"

#define DEBUG_PORT Serial

TaskHandle_t Task1;
TaskHandle_t Task2;

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom display library

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define TFT_BL  4  // Display backlight control pin
#define ADC_EN  14  //ADC_EN is the ADC detection enable port
#define ADC_PIN 34

//TFT y positions for texts and numbers
#define textLvl1 8            // y coordinates for text
#define textLvl2 68
#define textLvl3 129
#define textLvl4 189
#define drawLvl1 38            // and numbers
#define drawLvl2 99
#define drawLvl3 159
#define drawLvl4 219            // TTGO 135x240 TFT display

#define BUTTON_PIN  0
#define BUTTON_2_PIN  35

Button bouton(BUTTON_PIN);
Button bouton2(BUTTON_2_PIN);

#define BATTtemp_Warning 40.0       // RED color above this value (Celsius)
#define SoCpercent_Warning 10       // RED color below this value (Percent)
#define AUXSoCpercent_Warning 60    // RED color below this value (Percent)
#define BATTv_LOW_Warning 320       // Main Battery Low Voltage warning (Volts)
#define BATTv_HIGH_Warning 410      // Main Battery High Voltage warning (Volts)
#define AUXBATTv_LOW_Warning 11.8   // Main Battery Low Voltage warning (Volts)
#define AUXBATTv_HIGH_Warning 14.5  // Main Battery High Voltage warning (Volts)
#define pagenumbers 7               // number of pages to display

int ledBacklight = 100; // Initial TFT backlight intensity on a scale of 0 to 255. Initial value is 80.

/*////// Setting PWM properties, do not change this! /////////*/
const int pwmFreq = 5000;
const int pwmResolution = 8;
const int pwmLedChannelTFT = 0;

const int VESSoff = 12;
boolean SelectOn = true;
unsigned long StartMillis;
unsigned long ToggleDelay = 1000;
boolean ResetOn = true;
int screenNbr = 0;

float BattMinT;
float BattMaxT;
float AuxBattV;
float AuxBattC;
float AuxBattSoC;      
float Batt12V;
float BATTv;
float BATTc;
float MAXcellv;
float MINcellv;
float CellVdiff;
float CEC;
float CED;
float CDC;
float CCC;
float BmsSoC;
float Max_Pwr;
float Max_Reg;
float SoC;
float SOH;  
float Heater;
float COOLtemp;
float OUTDOORtemp;
float INDOORtemp;
char SpdSelect;
uint32_t Odometer;
float Speed;
byte TransSelByte;
byte Park;
byte Reverse;
byte Neutral;
byte Drive;
char selector[1];
byte StatusWord;
byte BMS_ign;
float OPtimemins;
float OPtimehours;
float TireFL_P;
float TireFR_P;
float TireRL_P;
float TireRR_P;
float TireFL_T;
float TireFR_T;
float TireRL_T;
float TireRR_T;
float Power;
float CurrInitOdo = 0;
float CurrInitCEC = 0;
float CurrInitCED = 0;
float CurrInitSoC= 0;
float CurrTripOdo;
float CurrNet_kWh;
float CurrUsedSoC;
float CurrTripDisc;
float CurrTripReg;
float Prev_kWh = 0;
float Net_kWh = 0;
float UsedSoC = 0;
float Net_Ah = 0;
float DischAh = 0;
float RegenAh = 0;
float TripOdo;
int InitOdo = 0;
float PrevOPtimemins;
float TripOPtime;
float CurrTimeInit;
float CurrOPtime;
float InitSoC = 0;
float InitCEC = 0;
float InitCED = 0;
float InitCCC = 0;
float InitCDC = 0;
float PrevSoC = 0;
float Regen = 0;
float Discharg = 0;
float LastSoC = 0;
double integral; // variable to calculate energy between to SoC values
double interval; // variable to calculate energy between to SoC values
float return_kwh; // variable to calculate energy between to SoC values
float EstFull_kWh;
float EstFull_Ah;
float kWh_corr;
float left_kwh;
float used_kwh;
float degrad_ratio;
float old_kWh_100km = 14;
float old_lost = 1;
float EstLeft_kWh;
float MeanSpeed;
float Time_100km;
float TripkWh_100km;
float kWh_100km;
float Est_range;
bool DriveOn = false;
bool winter = false;
bool SetupOn = false;
bool StartWifi = true;
bool initscan = false;
bool InitRst = false;
bool kWh_update = false;
bool corr_update = false;
int update_lock = 0;
bool DrawBackground = true;
char title1[12];
char title2[12];
char title3[12];
char title4[12];       
char value1[5];
char value2[5];
char value3[5];        
char value4[5];
char prev_value1[5];
char prev_value2[5];
char prev_value3[5];        
char prev_value4[5];        
bool negative_flag1;
bool negative_flag2;
bool negative_flag3;
bool negative_flag4;
float value1_float;
float value2_float;
float value3_float;
float value4_float;
int nbr_decimal1;
int nbr_decimal2;
int nbr_decimal3;
int nbr_decimal4;


/*////// Variables for Google Sheet data transfer ////////////*/
bool send_enabled = false;
bool send_data = false;
bool data_sent = false;
int nbParam = 45;    //number of parameters to send to Google Sheet
unsigned long sendInterval = 5000;
unsigned long currentTimer = 0;
unsigned long previousTimer = 0;
bool sendIntervalOn = false;

const char* resource = "/trigger/SendData/with/key/dqNCA93rEfn0CAeqkVRXvl";

// Maker Webhooks IFTTT
const char* server = "maker.ifttt.com";

/*////// Variables for Google Sheet data transfer ////////////*/
const int httpsPort = 443;
const char* fingerprint = "A0:92:D5:4F:FD:BB:96:90:40:7F:61:75:64:CD:BB:42:F7:15:4F:58";
const char* host = "script.google.com";
const char *GScriptId = "AKfycbyWlbpQIBM2NEVZOU1FQ8zpJ3hYuwJK-L4Qk81cnsUaEhHDN25hIBuhImjuRY3vJL9hyg";
String url = String("/macros/s/") + GScriptId + "/exec?";
String payload2 = "id=Sensor_1&SOC=67.00&Power=-6.47&BattMinT=8.00&BATTv=378.10&AuxBattSOC=86.00";
String url2 = "https://script.google.com/macros/s/AKfycbyWlbpQIBM2NEVZOU1FQ8zpJ3hYuwJK-L4Qk81cnsUaEhHDN25hIBuhImjuRY3vJL9hyg/exec?id=Sensor_1&SOC=67.00&Power=-6.47&BattMinT=8.00&BATTv=378.10&AuxBattSOC=86.00";
static bool flag = false;

/*////// Variables for OBD data timing ////////////*/
const long obd_update = 100; // interval to update OBD data (milliseconds)
unsigned long currentMillis;  // timing variable to sample OBD data
unsigned long previousMillis = 0; // timing variable to sample OBD data
int pid_counter;

/*////// Variables for Buttons ////////////*/
const long Longinterval = 3000; // interval for long button press (milliseconds)
unsigned long InitMillis; // will store time when button is pressed
unsigned long InitMillis2; // will store time when button is pressed

/*///////Define a structure to store the PID query data frames ///////////*/
struct dataFrames_struct {
  char frames[9][20]; // 9 frames each of 20chars
};

typedef struct dataFrames_struct dataFrames; // create a simple name for this type of data
dataFrames results; // this struct will hold the results


/*////////////////////////////////////////////////////////////////////////*/
/*                         START OF SETUP                                 */
/*////////////////////////////////////////////////////////////////////////*/

void setup() {

  /*////////////////////////////////////////////////////////////////*/   
  /*              Open serial monitor communications                */
  /*////////////////////////////////////////////////////////////////*/

  Serial.begin(9600);  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    
  Serial.println("Serial Monitor - STARTED");
  

  //pinMode(VESSoff, OUTPUT); // enable output pin that activate a relay to temporary disable the VESS

  /*//////////////Initialise buttons ////////////////*/

  bouton.begin(); // left button
  bouton2.begin(); // right button

  /*//////////////Initialise OLED display ////////////////*/

  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);

  Serial.print("Configuring PWM for TFT backlight... ");
  ledcSetup(pwmLedChannelTFT, pwmFreq, pwmResolution);
  ledcAttachPin(TFT_BL, pwmLedChannelTFT);
  Serial.println("DONE");

  Serial.print("Setting PWM for TFT backlight to default intensity... ");
  ledcWrite(pwmLedChannelTFT, ledBacklight);
  Serial.println("DONE");
  
  tft.init(); // display initialisation
  tft.begin();
  tft.setRotation(0);  // 0 & 2 Portrait. 1 & 3 landscape
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2); 
  
  /*////// initialize EEPROM with predefined size ////////*/
  EEPROM.begin(128);

  /* uncomment if you need to display Safestring results on Serial Monitor */
  //SafeString::setOutput(Serial);

  

  //xTaskCreatePinnedToCore(
    //Task2code, /* Function to implement the task */
    //"Task2", /* Name of the task */
    //10000,  /* Stack size in words */
    //NULL,  /* Task input parameter */
    //0,  /* Priority of the task */
    //&Task2,  /* Task handle. */
    //0); /* Core where the task should run */
    //delay(500);
  
  /*////// Get the stored values from last re-initialisation /////*/
  
  Net_kWh = EEPROM.readFloat(0);
  InitCED = EEPROM.readFloat(4);    
  InitCEC = EEPROM.readFloat(8);
  InitSoC = EEPROM.readFloat(12);
  UsedSoC = EEPROM.readFloat(16);
  InitOdo = EEPROM.readFloat(20);
  InitCDC = EEPROM.readFloat(24);
  InitCCC = EEPROM.readFloat(28);
  old_lost = EEPROM.readFloat(32);
  old_kWh_100km = EEPROM.readFloat(36);
  winter = EEPROM.readBool(40);
  PrevOPtimemins = EEPROM.readFloat(44);
  kWh_corr = EEPROM.readFloat(48);
        
/*/////////////////////////////////////////////////////////////////*/
/*                    CONNECTION TO OBDII                          */
/*/////////////////////////////////////////////////////////////////*/
    
  ConnectToOBD2(tft);
             

/*/////////////////////////////////////////////////////////////////*/
/*                     CONNECTION TO WIFI                         */
/*/////////////////////////////////////////////////////////////////*/

  if (StartWifi){
    ConnectWifi(tft);
    if (WiFi.status() == WL_CONNECTED) {
      send_enabled = true;      
    }
  }

  initscan = true; // To write header name on Google Sheet on power up

  /*//////////////Initialise Task on core0 to send data on Google Sheet ////////////////*/
  
  xTaskCreatePinnedToCore(
    makeIFTTTRequest, /* Function to implement the task */
    "makeIFTTTRequest", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    5,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */
    Serial.println("Task started");
    delay(500);

  tft.fillScreen(TFT_BLACK);
  
}   

/*////////////////////////////////////////////////////////////////////////*/
/*                         END OF SETUP                                   */
/*////////////////////////////////////////////////////////////////////////*/

//----------------------------------------------------------------------------------------
//              OBDII Payloads Processing Functions                                           
//----------------------------------------------------------------------------------------

void clearResultFrames(dataFrames& results) {
  for (int i = 0; i < 9; i++) {
    results.frames[i][0] = '\0';
  }
}

// format is <headerBytes> then <frameNumberByte>:<frameDataBytes> repeated
void processPayload(char *OBDdata, size_t datalen, dataFrames& results) {
  cSFPS(data, OBDdata, datalen); // wrap in a SafeString
  clearResultFrames(results);
  size_t idx = data.indexOf(':'); // skip over header and find first delimiter
  while (idx < data.length()) {
    int frameIdx = data[idx - 1] - '0'; // the char before :
    if ((frameIdx < 0) || (frameIdx > 8)) { // error in frame number skip this frame, print a message here
      //SafeString::Output.print("frameIdx:"); SafeString::Output.print(frameIdx); SafeString::Output.print(" outside range data: "); data.debug();
      idx = data.indexOf(':', idx + 1); // step over : and find next :
      continue;
    }
    cSFA(frame, results.frames[frameIdx]); // wrap a result frame in a SafeString to store this frame's data
    idx++; // step over :
    size_t nextIdx = data.indexOf(':', idx); // find next :
    if (nextIdx == data.length()) {
      data.substring(frame, idx);  // next : not found so take all the remaining chars as this field
    } else {
      data.substring(frame, idx, nextIdx - 1); // substring upto one byte before next :
    }
    //SafeString::Output.print("frameIdx:"); SafeString::Output.print(frameIdx); SafeString::Output.print(" "); frame.debug();
    idx = nextIdx; // step onto next frame
  }
}

//------------------------------------------------------------------------------------------
//                  End of Payloads Processing                                  
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
//             Bytes extraction from dataFrame                                  
//------------------------------------------------------------------------------------------

int convertToInt(char* dataFrame, size_t startByte, size_t numberBytes) {
  int offset = (startByte -1) * 2;
  // define a local SafeString on the stack for this method
  cSFP(frame, dataFrame);
  cSF(hexSubString, frame.capacity()); // allow for taking entire frame as a substring
  frame.substring(hexSubString, offset, offset + (numberBytes * 2)); // endIdx in exclusive in SafeString V2
  hexSubString.debug(F(" hex number "));
  long num = 0;
  if (!hexSubString.hexToLong(num)) {
    hexSubString.debug(F(" invalid hex number "));
  }
  return num;
}

//------------------------------------------------------------------------------------------
//               End of Bytes extraction from dataFrame                                  
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
//         Data retreve from OBD2 and extract values of it                               
//------------------------------------------------------------------------------------------

void read_data(){

  pid_counter++; 
    
  // read in rawData via ODBII
  switch (pid_counter){
        
    case 1:
        
        myELM327.sendCommand("AT SH 7E4");       // Set Header for BMS
        
        if (myELM327.queryPID("220101")) {      // Service and Message PID = hex 22 0101 => dec 34, 257
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
          
          processPayload(payload, payloadLen, results);
            
          int BattMinTraw = convertToInt(results.frames[2], 6, 1); //specify frame#, starting Byte(o in TorquePro) and # of bytes required
          if (BattMinTraw > 127){ //conversition for negative value
             BattMinT = -1 * (256 - BattMinTraw);
          }
          else{
            BattMinT = BattMinTraw;
          }
          int BattMaxTraw = convertToInt(results.frames[2], 5, 1); //specify frame#, starting Byte(o in TorquePro) and # of bytes required
          if (BattMaxTraw > 127){ //conversition for negative value
             BattMaxT = -1 * (256 - BattMaxTraw);
          }
          else{
            BattMaxT = BattMaxTraw;
          }
          AuxBattV = convertToInt(results.frames[4], 6, 1) * 0.1;
          BATTv = convertToInt(results.frames[2], 3, 2) * 0.1;
          int CurrentByte1= convertToInt(results.frames[2], 1, 1);
          int CurrentByte2= convertToInt(results.frames[2], 2, 1);
          if (CurrentByte1 > 127){  // the most significant bit is the sign bit so need to calculate commplement value if true
            BATTc = -1 * (((255 - CurrentByte1) * 256) + (256 - CurrentByte2)) * 0.1;
          }
          else{
            BATTc = ((CurrentByte1 * 256) + CurrentByte2) * 0.1;
          }
          CEC = convertToInt(results.frames[6], 1, 4) * 0.1;
          CED = ((convertToInt(results.frames[6], 5, 3) << 8) + convertToInt(results.frames[7], 1, 1)) * 0.1;
          CCC = ((convertToInt(results.frames[4], 7, 1) << 24) + convertToInt(results.frames[5], 1, 3)) * 0.1;
          CDC = convertToInt(results.frames[5], 4, 4) * 0.1;
          BmsSoC = convertToInt(results.frames[1], 2, 1) * 0.5;
          StatusWord = convertToInt(results.frames[7], 6, 1); // Extract byte that contain BMS status bits
          BMS_ign = bitRead(StatusWord,2);
          MAXcellv = convertToInt(results.frames[3], 7, 1) * 0.02;
          MINcellv = convertToInt(results.frames[4], 2, 1) * 0.02;          
          OPtimemins = convertToInt(results.frames[7], 2, 4) * 0.01666666667;
          OPtimehours = OPtimemins * 0.01666666667;
          }
          UpdateNetEnergy();
          break;

    case 2:
  
        myELM327.sendCommand("AT SH 7E4");       // Set Header for BMS
        
        if (myELM327.queryPID("220105")) {      // Service and Message PID = hex 22 0105 => dec 34, 261
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
                  
          processPayload(payload, payloadLen, results);
          Max_Pwr = convertToInt(results.frames[3], 2, 2) * 0.01;    
          Max_Reg = (((convertToInt(results.frames[2], 7, 1)) << 8) + convertToInt(results.frames[3], 1, 1)) * 0.01;
          SoC = convertToInt(results.frames[5], 1, 1) * 0.5;
          SOH = convertToInt(results.frames[4], 2, 2) * 0.1;  
          int HeaterRaw = convertToInt(results.frames[3], 7, 1);
          if (HeaterRaw > 127){ //conversition for negative value
             Heater = -1 * (256 - HeaterRaw);
          }
          else{
            Heater = HeaterRaw;          
          }
          
        }
        break;

    case 3:
  
        myELM327.sendCommand("AT SH 7E4");       // Set Header for BMS
        
        if (myELM327.queryPID("220106")) {      // Service and Message PID = hex 22 0106 => dec 34, 262
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;          

          processPayload(payload, payloadLen, results);
          int COOLtempRaw = convertToInt(results.frames[1], 2, 1) * 0.01; // Cooling water temperature          
          if (COOLtempRaw > 127){ //conversition for negative value
             COOLtemp = -1 * (256 - COOLtempRaw);
          }
          else{
            COOLtemp = COOLtempRaw;          
          }
        }
        break;
        
     case 4: 
        myELM327.sendCommand("AT SH 7E2");     // Set Header for Vehicle Control Unit
        
        if (myELM327.queryPID("2101")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;         

          processPayload(payload, payloadLen, results);
          TransSelByte = convertToInt(results.frames[1], 2, 1); // Extract byte that contain transmission selection bits
          //Serial.print("SelByte: "); Serial.println(TransSelByte, 1);
          Park = bitRead(TransSelByte,0);
          Reverse = bitRead(TransSelByte,1);
          Neutral = bitRead(TransSelByte,2);
          Drive = bitRead(TransSelByte,3);
          
          if (Park) selector[0] = 'P';
          if (Reverse) selector[0] = 'R';
          if (Neutral) selector[0] = 'N';
          if (Drive) selector[0] = 'D'; 
          SpdSelect =  selector[0];
          }
        break;

     case 5: 
        myELM327.sendCommand("AT SH 7E2");     // Set Header for Vehicle Control Unit
        if (myELM327.queryPID("2102")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;          

          processPayload(payload, payloadLen, results);
          //AuxBattV = convertToInt(results.frames[3], 2, 2)* 0.001; //doesn't work...
          int AuxCurrByte1= convertToInt(results.frames[3], 4, 1);
          int AuxCurrByte2= convertToInt(results.frames[3], 5, 1);
          if (AuxCurrByte1 > 127){  // the most significant bit is the sign bit so need to calculate commplement value if true
            AuxBattC = -1 * (((255 - AuxCurrByte1) * 256) + (256 - AuxCurrByte2)) * 0.01;
          }
          else{
            AuxBattC = ((AuxCurrByte1 * 256) + AuxCurrByte2) * 0.01;
          }          
          AuxBattSoC = convertToInt(results.frames[3], 6, 1);
          }
        break;
        
     case 6: 
        myELM327.sendCommand("AT SH 7C6");       // Set Header for CLU Cluster Module
        if (myELM327.queryPID("22B002")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;          
          
          processPayload(payload, payloadLen, results);
          Odometer = convertToInt(results.frames[1], 4, 3);
          }
        break;

     case 7:  
       myELM327.sendCommand("AT SH 7B3");       //Set Header Aircon 
        if (myELM327.queryPID("220100")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;

          processPayload(payload, payloadLen, results);          
          INDOORtemp = (((convertToInt(results.frames[1], 3, 1)) * 0.5) - 40);
          OUTDOORtemp = (((convertToInt(results.frames[1], 4, 1)) * 0.5) - 40);
          }        
        break;

     case 8:  
        myELM327.sendCommand("AT SH 7D4");       //Set Speed Header 
        if (myELM327.queryPID("220101")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;

          processPayload(payload, payloadLen, results);
          Speed = (((convertToInt(results.frames[1], 7, 1)) << 8) + convertToInt(results.frames[2], 1, 1)) * 0.1;
          } 
        break;

      case 9:  
        myELM327.sendCommand("AT SH 7A0");       //Set BCM Header 
        if (myELM327.queryPID("22C00B")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;

          processPayload(payload, payloadLen, results);
          TireFL_P = convertToInt(results.frames[1], 2, 1) * 0.2;
          TireFL_T = convertToInt(results.frames[1], 3, 1) - 50;
          TireFR_P = convertToInt(results.frames[1], 6, 1) * 0.2;
          TireFR_T = convertToInt(results.frames[1], 7, 1) - 50;
          TireRL_P = convertToInt(results.frames[2], 7, 1) * 0.2;
          TireRL_T = convertToInt(results.frames[3], 1, 1) - 50;
          TireRR_P = convertToInt(results.frames[2], 3, 1) * 0.2;
          TireRR_T = convertToInt(results.frames[2], 4, 1) - 50;
          }        
        pid_counter = 0;
        break;
  }
  

  /////// Miscellaneous calculations /////////   
    
  Power = (BATTv * BATTc) * 0.001;

  if(!ResetOn){     // On power On, wait for current trip values to be re-initialized before executing the next lines of code
    TripOdo = Odometer - InitOdo;
    
    CurrTripOdo = Odometer - CurrInitOdo;    
    
    CurrOPtime = OPtimemins - CurrTimeInit;

    TripOPtime = CurrOPtime + PrevOPtimemins;
  
    UsedSoC = InitSoC - SoC;
  
    CurrUsedSoC = CurrInitSoC - SoC;
  
    EstFull_Ah = 100 * Net_Ah / UsedSoC;
  
    CellVdiff = MAXcellv - MINcellv;
    
    EstFull_kWh = 100 * Net_kWh / UsedSoC;

    if(PrevSoC != SoC){  // perform "used_kWh" and "left_kWh" when SoC changes
      if(InitRst){  // On Trip reset, initial kWh calculation
        Serial.print("1st Reset");
        reset_trip();
        kWh_corr = 0;
        PrevSoC = SoC;
        Prev_kWh = Net_kWh;
        used_kwh = calc_kwh(SoC, InitSoC);
        left_kwh = calc_kwh(0, SoC);        
        initscan = true;
        InitRst = false;
      }
      if(!InitRst){ // kWh calculation when the Initial reset is not active
        // After a Trip Reset, perform a new reset if SoC changed without a Net_kWh increase (in case SoC was just about to change when the reset was performed) or if SoC changed from 100 to 99 (did not go through 99.5)
        if(((Net_kWh < 0.3) & (PrevSoC > SoC)) | ((SoC > 98.5) & ((PrevSoC - SoC) > 0.5))){ 
          Serial.print("Net_kWh= ");Serial.println(Net_kWh);
          Serial.print("2nd Reset");
          reset_trip();
          kWh_corr = 0;
          used_kwh = calc_kwh(SoC, InitSoC);
          left_kwh = calc_kwh(0, SoC);
          PrevSoC = SoC;
          Prev_kWh = Net_kWh;
          kWh_update = true;
        }
        else if ((PrevSoC > SoC) & ((PrevSoC - SoC) < 1)){ // Normal kWh calculation when SoC decreases and exception if a 0 gitch in SoC data
          kWh_corr = 0;
          used_kwh = calc_kwh(SoC, InitSoC);
          left_kwh = calc_kwh(0, SoC);
          PrevSoC = SoC;
          Prev_kWh = Net_kWh;
          kWh_update = true;
        }
      }
        
    }
    else if((Prev_kWh < Net_kWh) & !kWh_update){  // since the SoC has only 0.5 kWh resolution, when the Net_kWh increases, a 0.1 kWh is added to the kWh calculation to interpolate until next SoC change.
      kWh_corr += 0.1;
      used_kwh = calc_kwh(PrevSoC, InitSoC) + kWh_corr;
      left_kwh = calc_kwh(0, PrevSoC) - kWh_corr;
      Prev_kWh = Net_kWh;
      corr_update = true;
    }
    else if((Prev_kWh > Net_kWh) & !kWh_update){    // since the SoC has only 0.5 kWh resolution, when the Net_kWh decreases, a 0.1 kWh is substracted to the kWh calculation to interpolate until next SoC change.
      kWh_corr -= 0.1;
      used_kwh = calc_kwh(PrevSoC, InitSoC) + kWh_corr;
      left_kwh = calc_kwh(0, PrevSoC) - kWh_corr;
      Prev_kWh = Net_kWh;
      corr_update = true;
    }
    
    if(sendIntervalOn){ // add condition so "kWh_corr" is not trigger before a cycle after a "kWh_update" when wifi is not connected
      if(kWh_update){
        Prev_kWh = Net_kWh;        
        kWh_update = false; // reset kWh_update so correction logic starts again         
      }            
      if(corr_update){
        corr_update = false;  // reset corr_update since it's not being recorded 
      }      
      sendIntervalOn = false;
    }    
  }  
        
    if(used_kwh >= 5){
      degrad_ratio = Net_kWh / used_kwh;      
    }
    else{
      degrad_ratio = old_lost;
    }
    EstFull_kWh = (356 * 180) / 1000 * degrad_ratio;
    EstLeft_kWh = left_kwh * degrad_ratio;
      
    RangeCalc();
    save_lost(SpdSelect);  
}

//--------------------------------------------------------------------------------------------
//                   Net Energy Calculation Function
//--------------------------------------------------------------------------------------------
 
/*//////Function to calculate Discharge Energy Since last reset //////////*/

float UpdateNetEnergy(){        
        if (InitCED == 0){ //if discharge value have been reinitiate to 0 then      
            InitCED = CED;  //initiate to current CED for initial CED value and            
            InitSoC = SoC;  //initiate to current CED for initial SoC value and
            CurrInitCED = CED;
            }
        if (InitCDC == 0){
            InitCDC = CDC;
            }
        if (InitCEC == 0){ //if charge value have been reinitiate to 0 then
            InitCEC = CEC;  //initiate to current CEC for initial CEC value            
            CurrInitCEC = CEC;
            }
        if (InitCCC == 0){
            InitCCC = CCC;            
            }
                     
        Discharg = CED - InitCED;
        Regen = CEC - InitCEC;
        Net_kWh = Discharg - Regen;     
        
        DischAh = CDC - InitCDC;        
        RegenAh = CCC - InitCCC;
        Net_Ah = DischAh - RegenAh;
        
        CurrTripDisc = CED - CurrInitCED;       
        CurrTripReg = CEC - CurrInitCEC;
        CurrNet_kWh = CurrTripDisc - CurrTripReg;        
}

//--------------------------------------------------------------------------------------------
//                   Kilometer Range Calculation Function
//--------------------------------------------------------------------------------------------

float RangeCalc(){
  
  MeanSpeed = (CurrTripOdo / CurrOPtime) * 60;
  TripkWh_100km = Net_kWh * 100 / TripOdo;
    
  if (CurrTripOdo > 10 && !ResetOn){  
    kWh_100km = CurrNet_kWh * 100 / CurrTripOdo;
  }
  else if (CurrTripOdo > 2 && !ResetOn){
    kWh_100km = (0.5 * (Net_kWh * 100 / TripOdo)) + (0.5 * old_kWh_100km);    
  }
  else{
    kWh_100km = old_kWh_100km;
  }
  
  if (kWh_100km > 1){
    Est_range =  (EstLeft_kWh / kWh_100km) * 100;
  }
  else{
    Est_range = 999;
  }
}

//--------------------------------------------------------------------------------------------
//                   Function to calculate energy between two SoC values
//--------------------------------------------------------------------------------------------

float calc_kwh(float min_SoC, float max_SoC){
  
  static int N = 25;
  interval = (max_SoC - min_SoC) / N;
  integral = 0,0;
  double x = 0;
  for (int i = 0; i < N; ++i){
    x = min_SoC + interval * i;
    //integral += ((0.001487 * x) + 0.5672);  //64kWh battery energy equation
    //integral += ((0.0018344 * x) + 0.55);  //64kWh battery energy equation    
    integral += ((0.0014 * x) + 0.5748);  //64kWh battery energy equation
    //integral += ((2E-7 * pow(x,3)) + (-2.4E-5 * pow(x,2)) + (0.002194 * x) + 0.562);  //64kWh battery energy equation
  }
  return_kwh = integral * interval;
  return return_kwh;
}

//----------------------------------------------------------------------------------------
//        Task on core 0 to Send data to Google Sheet via IFTTT web service Function                                            
//----------------------------------------------------------------------------------------

void makeIFTTTRequest(void * pvParameters){
  for(;;){
    if (send_enabled && send_data) {
      Serial.print("Connecting to "); 
      Serial.print(server);
      
      WiFiClient client;
      int retries = 5;
      while(!!!client.connect(server, 80) && (retries-- > 0)) {
        Serial.print(".");
      }
      Serial.println();
      if(!!!client.connected()) {
        Serial.println("Failed to connect...");
      }
      
      Serial.print("Request resource: "); 
      Serial.println(resource);
      
      float sensor_Values[nbParam];
      
      char column_name[ ][15]={"SoC","Power","BattMinT","Heater","Net_Ah","Net_kWh","AuxBattSoC","AuxBattV","Max_Pwr","Max_Reg","BmsSoC","MAXcellv","MINcellv","BATTv","BATTc","Speed","Odometer","CEC","CED","CDC","CCC","SOH","OPtimemins","OUTDOORtemp","INDOORtemp","kWh_update","corr_update","Calc_Used","Calc_Left","TripOPtime","CurrOPtime","MeanSpeed","TripkWh_100km","degrad_ratio","EstLeft_kWh","Energ_100km","Est_range","TireFL_P","TireFR_P","TireRL_P","TireRR_P","TireFL_T","TireFR_T","TireRL_T","TireRR_T"};;
      
      sensor_Values[0] = SoC;
      sensor_Values[1] = Power;
      sensor_Values[2] = BattMinT;
      sensor_Values[3] = Heater;
      sensor_Values[4] = Net_Ah;
      sensor_Values[5] = Net_kWh;
      sensor_Values[6] = AuxBattSoC;
      sensor_Values[7] = AuxBattV;
      sensor_Values[8] = Max_Pwr;
      sensor_Values[9] = Max_Reg;  
      sensor_Values[10] = BmsSoC;
      sensor_Values[11] = MAXcellv;
      sensor_Values[12] = MINcellv;
      sensor_Values[13] = BATTv;
      sensor_Values[14] = BATTc;
      sensor_Values[15] = Speed;
      sensor_Values[16] = Odometer;
      sensor_Values[17] = CEC;
      sensor_Values[18] = CED;
      sensor_Values[19] = CDC;
      sensor_Values[20] = CCC;
      sensor_Values[21] = SOH;  
      sensor_Values[22] = OPtimemins;
      sensor_Values[23] = OUTDOORtemp;
      sensor_Values[24] = INDOORtemp;
      sensor_Values[25] = kWh_update;
      sensor_Values[26] = corr_update;
      sensor_Values[27] = used_kwh;
      sensor_Values[28] = left_kwh;
      sensor_Values[29] = TripOPtime;
      sensor_Values[30] = CurrOPtime;      
      sensor_Values[31] = MeanSpeed;
      sensor_Values[32] = TripkWh_100km;
      sensor_Values[33] = degrad_ratio;
      sensor_Values[34] = EstLeft_kWh;
      sensor_Values[35] = kWh_100km;
      sensor_Values[36] = Est_range;
      sensor_Values[37] = TireFL_P;
      sensor_Values[38] = TireFR_P;
      sensor_Values[39] = TireRL_P;
      sensor_Values[40] = TireRR_P;
      sensor_Values[41] = TireFL_T;
      sensor_Values[42] = TireFR_T;
      sensor_Values[43] = TireRL_T;
      sensor_Values[44] = TireRR_T;
          
      
      String headerNames = "";
      String payload ="";
      
      int i=0;
      
      if(initscan){
          initscan = false;
          while(i!=nbParam) 
        {
          if(i==0){
            headerNames = String("{\"value1\":\"") + column_name[i];
            i++;
          }
          if(i==nbParam)
            break;
          headerNames = headerNames + "|||" + column_name[i];
          i++;    
        }        
      
          payload = headerNames;
          
        }
        
      else{
        while(i!=nbParam) 
        {
          if(i==0)
          {
            payload = String("{\"value1\":\"") + sensor_Values[i];
            i++;
          }
          if(i==nbParam)
          {
             break;
          }
          payload = payload + "|||" + sensor_Values[i];
          i++;    
        }
      }      
      
      String jsonObject = payload + "\"}";                          
                       
      client.println(String("POST ") + resource + " HTTP/1.1");
      client.println(String("Host: ") + server); 
      client.println("Connection: close\r\nContent-Type: application/json");
      client.print("Content-Length: ");
      client.println(jsonObject.length());
      client.println();
      client.println(jsonObject);
            
      int timeout = 5; // 50 * 100mS = 5 seconds            
      while(!!!client.available() && (timeout-- > 0)){
        delay(100);
      }
      if(!!!client.available()) {
        Serial.println("No response...");
      }
      while(client.available()){
        Serial.write(client.read());
      }
      
      Serial.println();
      Serial.println("closing connection");
      client.stop();

      send_data = false;
      
      if(kWh_update){ //add condition so "kWh_corr" is not trigger before a cycle after a "kWh_update"
        Prev_kWh = Net_kWh;        
        kWh_update = false;  // reset kWh_update after it has been recorded and so the correction logic start again       
      }            
      if(corr_update){  
        corr_update = false;  // reset corr_update after it has been recorded
      }
    }    
    vTaskDelay(10);
  }
}

//--------------------------------------------------------------------------------------------
//                        Button functions
//--------------------------------------------------------------------------------------------

void ButtonLoop() {      
        
    if (bouton.pressed()){
      InitMillis = millis();
      Serial.println("Button pressed");
    }    
    if (bouton.released()){
        if (millis() - InitMillis >= Longinterval){
          Serial.println("Button Long press");        
          if (screenNbr == 0){          
            InitRst = true;
            PrevSoC = 0;
            //initscan = true;          
            return;
          }
        }
        else{
          Serial.println("Button short press");
          Serial.print("screenNbr");Serial.print(screenNbr);          
          DrawBackground = true;
          if(screenNbr == (pagenumbers - 1)){
                screenNbr = 0;
          }
          else if(screenNbr < pagenumbers -1){
                screenNbr++;
          } 
        }
    }
    if (bouton2.pressed()){
      InitMillis2 = millis();
      //Serial.println("Button2 pressed");
    }    
    
    if (bouton2.released()){
        if (millis() - InitMillis2 >= Longinterval){
            Serial.println("Button2 Long press");
            if(!SetupOn) {
              SetupOn = true;
            }
            else{
              EEPROM.writeBool(40, winter);
              EEPROM.commit();
              SetupOn = false;
            }
          }            
        else{
            //Serial.println("Button2 short press");       
            if(SetupOn){
                if (winter){                  
                  winter = false;
                  Serial.println("winter: ");Serial.println(winter);
                  
                }
                else{
                  winter = true;
                  Serial.println("winter: ");Serial.println(winter);                   
                }
            }
            else{
              ledBacklight = 80;
              ledcWrite(pwmLedChannelTFT, ledBacklight);
             
            }
        }
    }     
}

/*////////////// Full Trip Reset ///////////////// */

void reset_trip() { //Overall trip reset. Automatic if the car has been recharged to the same level as previous charge or if left button is pressed for more then 3 secondes
  
    Serial.println("saving");
    InitOdo = Odometer;                 
    InitCED = CED;  //initiate to current CED for initial CED value and
    InitSoC = SoC;  //initiate to current CED for initial SoC value and          
    InitCEC = CEC;  //initiate to current CEC for initial CEC value and
    InitCDC = CDC;
    InitCCC = CCC;
    Net_kWh = 0;
    UsedSoC = 0;
    kWh_corr = 0;
    Discharg = 0;
    Regen = 0;
    Net_Ah = 0;
    DischAh = 0;
    RegenAh = 0;
    PrevOPtimemins = 0;
    EEPROM.writeFloat(0, Net_kWh);    //save initial CED to Flash memory
    EEPROM.writeFloat(4, InitCED);    //save initial CED to Flash memory
    EEPROM.writeFloat(8, InitCEC);    //save initial CEC to Flash memory  
    EEPROM.writeFloat(12, InitSoC);    //save initial SoC to Flash memory
    EEPROM.writeFloat(16, UsedSoC);    //save initial SoC to Flash memory
    EEPROM.writeFloat(20, InitOdo);    //save initial Odometer to Flash memory
    EEPROM.writeFloat(24, InitCDC);    //save initial Calculated CED to Flash memory
    EEPROM.writeFloat(28, InitCCC);    //save initial Calculated CED to Flash memory
    EEPROM.writeFloat(32, degrad_ratio);    //save actual batt energy lost in Flash memory
    EEPROM.writeFloat(36, kWh_100km);    //save actual kWh/100 in Flash memory
    EEPROM.writeFloat(44, PrevOPtimemins);    //save initial time to Flash memory
    EEPROM.writeFloat(48, kWh_corr);    //save cummulative kWh correction (between 2 SoC values) to Flash memory       
    EEPROM.commit();
    Serial.println("Values saved to EEPROM");
    CurrInitCED = CED;
    CurrInitCEC = CEC;
    CurrInitOdo = Odometer;
    CurrInitSoC = SoC;
    CurrTripReg = 0;
    CurrTripDisc = 0;    
    CurrTimeInit = OPtimemins;
}

/*////////////// Current Trip Reset ///////////////// */

void ResetCurrTrip(){ // when the car is turned On, current trip values are resetted.
  
    if (
      ResetOn && (SoC > 1) && (Odometer > 1) && (CED > 1) && (CEC > 1)){ // ResetOn condition might be enough, might need to update code...        
        CurrInitCED = CED;
        CurrInitCEC = CEC;
        CurrInitOdo = Odometer;
        CurrInitSoC = SoC;
        CurrTripReg = 0;
        CurrTripDisc = 0;
        CurrTimeInit = OPtimemins;
        Serial.println("Trip Reset");
        Prev_kWh = Net_kWh;
        used_kwh = calc_kwh(SoC, InitSoC) + kWh_corr;
        left_kwh = calc_kwh(0, SoC) - kWh_corr;
        PrevSoC = SoC;
        ResetOn = false;
        DrawBackground = true;
  }
}

/*//////Function to disable the VESS //////////*/

void setVessOff(char selector){  
        if (selector == 'D' && SelectOn){      
          //digitalWrite(LedPin, HIGH);
          digitalWrite(VESSoff, HIGH);   
          StartMillis = millis();
          SelectOn = false;
        }
        if ((millis() - StartMillis) >= ToggleDelay){     
          //digitalWrite(LedPin, LOW);
          digitalWrite(VESSoff, LOW);    
        }
        if (selector == 'P'){
          SelectOn = true;
        }
      }

/*//////Function to save some variable before turn off the car //////////*/

void save_lost(char selector){  
        if (selector == 'D' && !DriveOn){ 
          DriveOn = true;
        }        
        if (selector == 'P' && DriveOn){  // when the selector is set to Park, some values are saved to be used the next time the car is started
          DriveOn = false;
          EEPROM.writeFloat(32, degrad_ratio);
          Serial.println("new_lost saved to EEPROM");
          EEPROM.writeFloat(36, kWh_100km);    //save actual kWh/100 in Flash memory
          EEPROM.writeFloat(44, TripOPtime);  //save initial trip time to Flash memory
          EEPROM.writeFloat(48, kWh_corr);    //save cummulative kWh correction (between 2 SoC values) to Flash memory
          EEPROM.commit();
        }
      }


void SetupMode(){
    tft.fillScreen(TFT_BLACK);    
    tft.setTextColor(TFT_WHITE,TFT_BLUE);
    tft.setTextPadding(135);
    //tft.drawRect(0, 0, 135, 59, TFT_BLUE); // Blue rectangle
    tft.drawString("Winter Mode", tft.width() / 2, textLvl1, 1);
    if (winter){
        tft.setTextColor(TFT_BLACK,TFT_GREEN); 
        tft.fillRect(1, 20, 134, 118, TFT_GREEN);    
        tft.setTextPadding(130);
        tft.drawString("ON", tft.width() / 2, 80, 2);
    }
    else {
        tft.setTextColor(TFT_GREEN,TFT_BLACK); 
        tft.fillRect(1, 20, 134, 118, TFT_BLACK);    
        tft.setTextPadding(130);
        tft.drawString("OFF", tft.width() / 2, 80, 2);
    }
}


//--------------------------------------------------------------------------------------------
//                   Draw Boxe objects Definition
//--------------------------------------------------------------------------------------------


void draw_warningbox_lvl1(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(1, 18, 134, 58, TFT_RED);
}

void draw_warningbox_lvl2(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(1, 78, 134, 118, TFT_RED);
}

void draw_warningbox_lvl3(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(1, 138, 134, 178, TFT_RED);
}

void draw_warningbox_lvl4(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(1, 198, 134, 234, TFT_RED);
}

void draw_normalbox_lvl1(){
                         //tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 19, 133, 59, TFT_BLACK);
}

void draw_normalbox_lvl2(){
                         //tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 79, 133, 119, TFT_BLACK);
}

void draw_normalbox_lvl3(){
                         //tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 139, 133, 179, TFT_BLACK);
}

void draw_normalbox_lvl4(){
                         //tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 199, 133, 239, TFT_BLACK);
}

void draw_greenbox_lvl1(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(1, 18, 134, 58, TFT_GREEN);
}

void draw_greenbox_lvl2(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(1, 78, 134, 118, TFT_GREEN);
}

void draw_greenbox_lvl3(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(1, 138, 134, 178, TFT_GREEN);
}

void draw_greenbox_lvl4(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(1, 198, 134, 234, TFT_GREEN);
}

//--------------------------------------------------------------------------------------------
//                   Format Displays in Pages
//--------------------------------------------------------------------------------------------

void DisplayPage(){        
        if(DrawBackground){
          tft.fillScreen(TFT_BLACK);
          tft.setTextDatum(MC_DATUM);
          tft.setTextFont(1);
          tft.setTextSize(2);
          tft.setTextColor(TFT_WHITE,TFT_BLUE);
          tft.setTextPadding(135);            
          tft.drawString(title1, tft.width() / 2, textLvl1);
          tft.drawString(title2, tft.width() / 2, textLvl2);
          tft.drawString(title3, tft.width() / 2, textLvl3);
          tft.drawString(title4, tft.width() / 2, textLvl4);          
                 
          strcpy(prev_value1,"");
          strcpy(prev_value2,"");
          strcpy(prev_value3,"");
          strcpy(prev_value4,"");
          DrawBackground = false;
          tft.setTextFont(2);
          tft.setTextSize(3);          
        }         

        if(value1_float < 0){ 
          value1_float = abs(value1_float);
          negative_flag1 = true;
        }
        else{
          negative_flag1 = false;
        }
        if(value2_float < 0){ 
          value2_float = abs(value2_float);
          negative_flag2 = true;
        }
        else{
          negative_flag2 = false;
        }
        if(value3_float < 0){ 
          value3_float = abs(value3_float);
          negative_flag3 = true;
        }
        else{
          negative_flag3 = false;
        }
        if(value4_float < 0){ 
          value4_float = abs(value4_float);
          negative_flag4 = true;
        }
        else{
          negative_flag4 = false;
        }
        
        dtostrf(value1_float,3,nbr_decimal1,value1);
        dtostrf(value2_float,3,nbr_decimal2,value2);
        dtostrf(value3_float,3,nbr_decimal3,value3);
        dtostrf(value4_float,3,nbr_decimal4,value4);
        
        if(value1 != prev_value1){            
          tft.setTextColor(TFT_BLACK,TFT_BLACK);        
          tft.drawString(prev_value1, tft.width()/2, drawLvl1);
          if(negative_flag1){        
            tft.setTextColor(TFT_ORANGE,TFT_BLACK);        
            tft.drawString(value1, tft.width()/2, drawLvl1);
          }
          else{            
            tft.setTextColor(TFT_GREEN,TFT_BLACK);        
            tft.drawString(value1, tft.width()/2, drawLvl1);
          }
          strcpy(prev_value1,value1);
        }
        if(value2 != prev_value2){         
          tft.setTextColor(TFT_BLACK,TFT_BLACK);
          tft.drawString(prev_value2, tft.width()/2, drawLvl2);     
          if(negative_flag2){     
            tft.setTextColor(TFT_ORANGE,TFT_BLACK);        
            tft.drawString(value2, tft.width()/2, drawLvl2);
          }
          else{
            tft.setTextColor(TFT_GREEN,TFT_BLACK);
            tft.drawString(value2, tft.width()/2, drawLvl2);            
          }
          strcpy(prev_value2,value2);
        }
        if(value3 != prev_value3){          
          tft.setTextColor(TFT_BLACK,TFT_BLACK);
          tft.drawString(prev_value3, tft.width()/2, drawLvl3);
          if(negative_flag3){          
            tft.setTextColor(TFT_ORANGE,TFT_BLACK);        
            tft.drawString(value3, tft.width()/2, drawLvl3);
          }
          else{
            tft.setTextColor(TFT_GREEN,TFT_BLACK);
            tft.drawString(value3, tft.width()/2, drawLvl3);            
          }
          strcpy(prev_value3,value3);
        }
        if(value4 != prev_value4){          
          tft.setTextColor(TFT_BLACK,TFT_BLACK);
          tft.drawString(prev_value4, tft.width()/2, drawLvl4);
          if(negative_flag4){          
            tft.setTextColor(TFT_ORANGE,TFT_BLACK);        
            tft.drawString(value4, tft.width()/2, drawLvl4);
          }         
          else{
            tft.setTextColor(TFT_GREEN,TFT_BLACK);
            tft.drawString(value4, tft.width()/2, drawLvl4);            
          }
          strcpy(prev_value4,value4);
        }                    
        
}
//-------------------------------------------------------------------------------------
//             Start of Pages content definition         
//-------------------------------------------------------------------------------------

/*///////////////// Display Page 1 //////////////////////*/
void page1(){ 
        
        strcpy(title1,"TripOdo");
        strcpy(title2,"UsedSoC");
        strcpy(title3,"Net_kWh");
        strcpy(title4,"EstLeft_kWh");
        value1_float = TripOdo;
        value2_float = UsedSoC;
        value3_float = Net_kWh;
        value4_float = EstLeft_kWh;
        nbr_decimal1 = 0;
        nbr_decimal2 = 1;
        nbr_decimal3 = 1;
        nbr_decimal4 = 1;        

        DisplayPage();               
}
/*///////////////// End of Display Page 1 //////////////////////*/

/*///////////////// Display Page 2 //////////////////////*/
void page2(){
  
        strcpy(title1,"kWh/100km");
        strcpy(title2,"Est_range");
        strcpy(title3,"CurrOPtime");
        strcpy(title4,"Full_Ah");        
        value1_float = kWh_100km;
        value2_float = Est_range;
        value3_float = CurrOPtime;
        value4_float = EstFull_Ah;
        nbr_decimal1 = 1;
        nbr_decimal2 = 0;
        nbr_decimal3 = 1;
        nbr_decimal4 = 0;

        DisplayPage();       
}
/*///////////////// End of Display Page 2 //////////////////////*/

/*///////////////// Display Page 3 //////////////////////*/
void page3(){

        strcpy(title1,"Calc_Used");        
        strcpy(title2,"Net_kWh");
        strcpy(title3,"Calc_Left");
        strcpy(title4,"Full_kWh");  
        value1_float = used_kwh;
        value2_float = Net_kWh;
        value3_float = left_kwh;
        value4_float = EstFull_kWh;
        nbr_decimal1 = 1;
        nbr_decimal2 = 1;
        nbr_decimal3 = 1;
        nbr_decimal4 = 1;

        DisplayPage();                            
}
/*///////////////// End of Display Page 3 //////////////////////*/

/*///////////////// Display Page 4 //////////////////////*/
void page4(){
  
        strcpy(title1,"BattMinT");
        strcpy(title2,"Heater");
        strcpy(title3,"Power");
        strcpy(title4,"BattMaxT");        
        value1_float = BattMinT;
        value2_float = Heater;
        value3_float = Power;
        value4_float = BattMaxT;
        nbr_decimal1 = 1;
        nbr_decimal2 = 1;
        if(value3_float >= 100){
          nbr_decimal3 = 0;
        }
        else{
          nbr_decimal3 = 1;
        }
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        } 

        DisplayPage();        
}
/*///////////////// End of Display Page 4 //////////////////////*/

/*///////////////// Display Page 5 //////////////////////*/
void page5(){
          
        strcpy(title1,"Max_Pwr");
        strcpy(title2,"Power");
        strcpy(title3,"BattMinT");
        strcpy(title4,"SoC");        
        value1_float = Max_Pwr;
        value2_float = Power;
        value3_float = BattMinT;
        value4_float = SoC;
        nbr_decimal1 = 0;
        if(value2_float >= 100){
          nbr_decimal2 = 0;
        }
        else{
          nbr_decimal2 = 1;
        }        
        nbr_decimal3 = 1;
        nbr_decimal4 = 1;

        DisplayPage();        
}
/*///////////////// End of Display Page 5 //////////////////////*/

/*///////////////// Display Page 6 //////////////////////*/
void page6(){
                 
        strcpy(title1,"Trip_kWh");
        strcpy(title2,"TripOdo");
        strcpy(title3,"TripSoC");
        strcpy(title4,"AuxBattSoC");        
        value1_float = CurrNet_kWh;
        value2_float = CurrTripOdo;
        value3_float = CurrUsedSoC;
        value4_float = AuxBattSoC;
        nbr_decimal1 = 1;
        nbr_decimal2 = 0;
        nbr_decimal3 = 1;        
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        }        

        DisplayPage();            
}
/*///////////////// End of Display Page 6 //////////////////////*/

/*///////////////// Display Page 7 //////////////////////*/
void page7(){
        
        strcpy(title1,"BmsSoC");
        strcpy(title2,"MAXcellv");
        strcpy(title3,"CellVdiff");
        strcpy(title4,"SOH");        
        value1_float = BmsSoC;
        value2_float = MAXcellv;
        value3_float = CellVdiff;
        value4_float = SOH;
        nbr_decimal1 = 1;
        nbr_decimal2 = 2;
        nbr_decimal3 = 2;
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        }

        DisplayPage();        
}
/*///////////////// End of Display Page 7 //////////////////////*/                 
        


/*///////////////////////////////////////////////////////////////////////*/
/*                     START OF LOOP                                     */ 
/*///////////////////////////////////////////////////////////////////////*/

void loop() {
  
  ButtonLoop();

  currentTimer = millis();  
  if (currentTimer - previousTimer >= sendInterval) {    
      send_data = true; // This will trigger logic to send data to Google sheet
      previousTimer = currentTimer;     
    if (!send_enabled){
      sendIntervalOn = true;      
    }
  } 
               
  /*/////// Read each OBDII PIDs /////////////////*/

  read_data();
  
  /*/////// Display Page Number /////////////////*/
  
  if(!SetupOn){
      switch (screenNbr){  // select page to display                
               case 0: page1(); break;
               case 1: page2(); break;
               case 2: page3(); break;
               case 3: page4(); break;
               case 4: page5(); break;
               case 5: page6(); break;
               case 6: page7(); break;                                                                                        
               }
      }
  
  else{ /*/////// Display Setup Page/////////////////*/
      SetupMode();
      }
                     
  ResetCurrTrip();
  
  //setVessOff(SpdSelect);  //This will momentarely set an output ON to turned off the VESS //  
}
