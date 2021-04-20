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
#define textLvl1 10            // y coordinates for text
#define textLvl2 70
#define textLvl3 130
#define textLvl4 190
#define drawLvl1 40            // and numbers
#define drawLvl2 100
#define drawLvl3 160
#define drawLvl4 220            // TTGO 135x240 TFT display

#define BUTTON_PIN  0
#define BUTTON_2_PIN  35

Button bouton(BUTTON_PIN);
Button bouton2(BUTTON_2_PIN);

#define BATTtemp_Warning 40.0       // RED color above this value (Celsius)
#define SOCpercent_Warning 10       // RED color below this value (Percent)
#define AUXSOCpercent_Warning 60    // RED color below this value (Percent)
#define BATTv_LOW_Warning 320       // Main Battery Low Voltage warning (Volts)
#define BATTv_HIGH_Warning 410      // Main Battery High Voltage warning (Volts)
#define AUXBATTv_LOW_Warning 11.8   // Main Battery Low Voltage warning (Volts)
#define AUXBATTv_HIGH_Warning 14.5  // Main Battery High Voltage warning (Volts)
#define pagenumbers 7               // number of pages to display

int ledBacklight = 80; // Initial TFT backlight intensity on a scale of 0 to 255. Initial value is 80.

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
float AuxBattSOC;      
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
float CalcCEC;
float CalcCED;
float BmsSOC;
float Max_Pwr;
float Max_Reg;
float SOC;
float SOH;  
float Heater;
float COOLtemp;
float OUTDOORtemp;
float INDOORtemp;
char SpdSelect;
float Odometer;
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
float Power;
float CurrEnergHr = 0;
float CurrInitOdo = 0;
float CurrInitCEC = 0;
float CurrInitCED = 0;
float CurrInitSOC= 0;
float CurrTripOdo;
float CurrNet_kWh;
float CurrUsedSOC;
float CurrTripDisc;
float CurrTripReg;
float Net_kWh = 0;
float UsedSOC = 0;
float Net_Ah = 0;
float DischAh = 0;
float RegenAh = 0;
int TripOdo;
int InitOdo = 0;
float InitOPtimemins;
float TripOPtime;
float CurrTimeInit;
float CurrOPtime;
float InitSOC = 0;
float InitCEC = 0;
float InitCED = 0;
float InitCCC = 0;
float InitCDC = 0;
float Regen = 0;
float Discharg = 0;
float LastSOC = 0;
float EstFull_kWh;
float EstFull_Ah;
float left_kwh;
float used_kwh;
float lost_ratio;
float new_lost;
float old_lost;
float EstLeft_kWh;
unsigned long elap_time; 
float MeanSpeed;
float Time_100km;
float Pwr_100km;
float Est_range;
bool DriveOn = false;
bool winter = false;
bool SetupOn = false;
bool StartWifi = true;
bool initscan = false;

/*////// Variables for Google Sheet data transfer ////////////*/
bool send_enabled = false;
bool send_data = false;
int nbParam = 32;    //number of parameters to send to Google Sheet
unsigned long sendInterval = 5000;
unsigned long currentTimer = 0;
unsigned long previousTimer = 0;

const char* resource = "/trigger/konaEv_readings/with/key/dqNCA93rEfn0CAeqkVRXvl";

// Maker Webhooks IFTTT
const char* server = "maker.ifttt.com";

/*////// Variables for OBD data timing ////////////*/
const long interval = 100; // interval to update OBD data (milliseconds)
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

  //pinMode(VESSoff, OUTPUT); // enable output pin that activate a relay to temporary disable the VESS

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
    //Task1code, /* Function to implement the task */
    //"Task1", /* Name of the task */
    //10000,  /* Stack size in words */
    //NULL,  /* Task input parameter */
    //0,  /* Priority of the task */
    //&Task1,  /* Task handle. */
    //0); /* Core where the task should run */
    //delay(500);

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
  InitSOC = EEPROM.readFloat(12);
  UsedSOC = EEPROM.readFloat(16);
  InitOdo = EEPROM.readFloat(20);
  InitCDC = EEPROM.readFloat(24);
  InitCCC = EEPROM.readFloat(28);
  old_lost = EEPROM.readFloat(32);
  InitOPtimemins = EEPROM.readFloat(36);
  winter = EEPROM.readBool(40);

/*////////////////////////////////////////////////////////////////*/   
/*              Open serial monitor communications                */
/*////////////////////////////////////////////////////////////////*/

  Serial.begin(9600);  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    
  Serial.println("Serial Monitor - STARTED");
    
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
  
}   

/*////////////////////////////////////////////////////////////////////////*/
/*                         END OF SETUP                                   */
/*////////////////////////////////////////////////////////////////////////*/

//----------------------------------------------------------------------------------------
//               Send data to Google Sheet via IFTTT web service Function                                            
//----------------------------------------------------------------------------------------

void makeIFTTTRequest() {
  
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
 
  char column_name[ ][30]={"SOC","Power","BattMinT","Heater","Net_Ah","Net_kWh","AuxBattSOC","AuxBattV","Max_Pwr","Max_Reg","BmsSOC","MAXcellv","MINcellv","BATTv","BATTc","Speed","Odometer","CEC","CED","CDC","CCC","SOH","OPtimemins","OUTDOORtemp","INDOORtemp","Calc_Used","Calc_Left","CurrEnergHr","MeanSpeed","Time_100km","Pwd_100km","Est_range"};;
  //char column_name[ ][30]={"SOC","Power","BattMinT","Heater","Net_Ah","Net_kWh","AuxBattSOC","AuxBattV","Max_Pwr","Max_Reg","BmsSOC","MAXcellv","MINcellv","BATTv","BATTc","Speed","Odometer","CEC","CED","CDC","CCC","SOH","OPtimemins","OUTDOORtemp","INDOORtemp"};;
  
  sensor_Values[0] = SOC;
  sensor_Values[1] = Power;
  sensor_Values[2] = BattMinT;
  sensor_Values[3] = Heater;
  sensor_Values[4] = Net_Ah;
  sensor_Values[5] = Net_kWh;
  sensor_Values[6] = AuxBattSOC;
  sensor_Values[7] = AuxBattV;
  sensor_Values[8] = Max_Pwr;
  sensor_Values[9] = Max_Reg;  
  sensor_Values[10] = BmsSOC;
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
  sensor_Values[25] = used_kwh;
  sensor_Values[26] = left_kwh;
  sensor_Values[27] = CurrEnergHr;
  sensor_Values[28] = MeanSpeed;
  sensor_Values[29] = Time_100km;
  sensor_Values[30] = Pwr_100km;
  sensor_Values[31] = Est_range;

  String headerNames = "";
  String payload ="";

  int i=0;
  //Serial.print("initscan: ");Serial.println(initscan);
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
    Serial.print("headerNames: ");Serial.println(headerNames);
  
      payload = headerNames;
      //payload = String("{\"value1\":\"") + "SOC" + "|||" + "Power" + "|||" + "BattMinT" + "|||" + "Heater" + "|||" + "Net_Ah" + "|||" + "Net_kWh" + "|||" + "AuxBattSOC" + "|||" + "AuxBattV" + "|||" + "Max_Pwr" + "|||" + "Max_Reg" + "|||" + "BmsSOC" + "|||" + "MAXcellv" + "|||" + "MINcellv" + "|||" + "BATTv" + "|||" + "BATTc" + "|||" + "Speed" + "|||" + "Odometer" + "|||" + "CEC" + "|||" + "CED" + "|||" + "CDC" + "|||" + "CCC";
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
  Serial.print("payload: ");Serial.println(payload);

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
  send_data = false;
  
  Serial.println();
  Serial.println("closing connection");
  client.stop(); 
}
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
          BmsSOC = convertToInt(results.frames[1], 2, 1) * 0.5;
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
          SOC = convertToInt(results.frames[5], 1, 1) * 0.5;
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
          AuxBattSOC = convertToInt(results.frames[3], 6, 1);
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
        pid_counter = 0;
        break;    
  }
  

  /////// Miscellaneous calculations /////////   
    
  Power = (BATTv * BATTc) * 0.001;

  TripOdo = Odometer - InitOdo;

  CurrTripOdo = Odometer - CurrInitOdo;

  TripOPtime = OPtimemins - InitOPtimemins;
  
  CurrOPtime = OPtimemins - CurrTimeInit;

  UsedSOC = InitSOC - SOC;

  CurrUsedSOC = CurrInitSOC - SOC;

  //EstFull_Ah = 100 * Net_Ah / UsedSOC;

  CellVdiff = MAXcellv - MINcellv;
  EstFull_kWh = 100 * Net_kWh / UsedSOC;
  calc_used_kwh();
  calc_left_kwh();
  
  if(used_kwh > 0){
    lost_ratio = (0,9 * old_lost) + (0,1 * (Net_kWh / used_kwh));
    lost_ratio = Net_kWh / used_kwh;
  }
  else{
    lost_ratio = 1;
  }
  EstFull_kWh = 64 * lost_ratio;
  EstLeft_kWh = left_kwh * lost_ratio;
  new_lost =  lost_ratio;

  
  energy();
  //save_lost(SpdSelect);
  
}

//--------------------------------------------------------------------------------------------
//                   Energy Calculation Function
//--------------------------------------------------------------------------------------------

/*//////64kWh battery energy equation //////////*/
double f(double x){
    (0.00165 * x) + 0.56;
  }
  
float energy(){
  CurrEnergHr = CurrNet_kWh * 60 / CurrOPtime;  
  
  MeanSpeed = (CurrTripOdo / CurrOPtime) * 60;
  if (MeanSpeed > 0){
    Time_100km = 100 / MeanSpeed;
  }
  else{
    Time_100km = 100 / 99999;
  }
  Pwr_100km = CurrEnergHr * Time_100km;
  if (Pwr_100km > 1){
    Est_range =  (EstLeft_kWh / Pwr_100km) * 100;
  }
  else{
    Est_range = 0.1;
  }
}

void calc_used_kwh(){
  double integral;
  double interval;
  int N = 50;
  interval = (InitSOC - SOC) / N;
  integral = 0;
  float x = 0;
  for (int i = 0; i < N; ++i){
    x = SOC + interval * i;    
    integral += ((0.00165 * x) + 0.56);    
  }
  used_kwh = integral * interval;
  Serial.print("used_kwh= ");Serial.println(used_kwh);
}

void calc_left_kwh(){
  double integral;
  double interval;
  int N = 50;
  interval = (SOC - 0) / N;
  integral = 0;
  float x = 0;
  for (int i = 0; i < N; ++i){
    x = 0 + interval * i;    
    integral += ((0.00165 * x) + 0.56);    
  }
  left_kwh = integral * interval;
  Serial.print("left_kwh= ");Serial.println(left_kwh);
}

//--------------------------------------------------------------------------------------------
//                   Task to calculate kWh that will be run on Core 0
//--------------------------------------------------------------------------------------------

void Task1code( void * pvParameters ){
  double integral;
  double interval;
  int N = 50;

  for(;;){
  interval = (InitSOC - SOC) / N;
  integral = 0;
  float x = 0;
  for (int i = 0; i < N; ++i){
    x = SOC + interval * i;    
    integral += ((0.00165 * x) + 0.56);    
  }
  used_kwh = integral * interval;
  Serial.print("used_kwh: ");Serial.println(used_kwh);
  }
}

void Task2code( void * pvParameters ){
  double integral;
  double interval;
  int N = 50;

  for(;;){
  interval = (SOC - 0) / N;
  integral = 0;
  float x = 0;
  for (int i = 0; i < N; ++i){
    x = 0 + interval * i;    
    integral += ((0.00165 * x) + 0.56);    
  }
  left_kwh = integral * interval;
  Serial.print("left_kwh: ");Serial.println(left_kwh);
  }
}

//--------------------------------------------------------------------------------------------
//                   Net Energy Calculation Function
//--------------------------------------------------------------------------------------------
 
/*//////Function to calculate Discharge Energy Since last reset //////////*/

float UpdateNetEnergy(){        
        if (InitCED == 0){ //if discharge value have been reinitiate to 0 then      
            InitCED = CED;  //initiate to current CED for initial CED value and            
            InitSOC = SOC;  //initiate to current CED for initial SOC value and
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
//                   End of Net Energy Calculation
//--------------------------------------------------------------------------------------------


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
            reset_trip();
            initscan = true;          
            return;
          }
        }
        else{
          Serial.println("Button short press");
          Serial.print("screenNbr");Serial.print(screenNbr);
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
              EEPROM.writeBool(36, winter);
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

void reset_trip() {
  
    //Serial.println("saving");
    InitOdo = Odometer;                 
    InitCED = CED;  //initiate to current CED for initial CED value and
    InitSOC = SOC;  //initiate to current CED for initial SOC value and          
    InitCEC = CEC;  //initiate to current CEC for initial CEC value and
    InitCDC = CDC;
    InitCCC = CCC;
    Net_kWh = 0;
    UsedSOC = 0;
    Discharg = 0;
    Regen = 0;
    Net_Ah = 0;
    DischAh = 0;
    RegenAh = 0;
    EEPROM.writeFloat(0, Net_kWh);    //save initial CED to Flash memory
    EEPROM.writeFloat(4, InitCED);    //save initial CED to Flash memory
    EEPROM.writeFloat(8, InitCEC);    //save initial CEC to Flash memory  
    EEPROM.writeFloat(12, InitSOC);    //save initial SOC to Flash memory
    EEPROM.writeFloat(16, UsedSOC);    //save initial SOC to Flash memory
    EEPROM.writeFloat(20, InitOdo);    //save initial Odometer to Flash memory
    EEPROM.writeFloat(24, InitCDC);    //save initial Calculated CED to Flash memory
    EEPROM.writeFloat(28, InitCCC);    //save initial Calculated CED to Flash memory
    EEPROM.writeFloat(32, 1);    //Reset MeanPower to 0 in Flash memory
    EEPROM.writeFloat(36, InitOPtimemins);    //Reset MeanPower to 0 in Flash memory
    EEPROM.commit();
    //Serial.println("Values saved to EEPROM");
}

/*////////////// Current Trip Reset ///////////////// */

void ResetCurrTrip(){
  
    if (BMS_ign && ResetOn && (SOC > 1) && (Odometer > 1) && (CED > 1) && (CEC > 1)){
        CurrInitCED = CED;
        CurrInitCEC = CEC;
        CurrInitOdo = Odometer;
        CurrInitSOC = SOC;
        CurrTripReg = 0;
        CurrTripDisc = 0;
        CurrTimeInit = OPtimemins;
        Serial.println("Trip Reset");
        ResetOn = false;
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

/*//////Function to save current lost //////////*/

void save_lost(char selector){  
        if (selector == 'D' && !DriveOn){ 
          DriveOn = true;
        }        
        if (selector == 'P' && DriveOn){
          DriveOn = false;
          EEPROM.writeFloat(32, new_lost);
          Serial.println("new_lost saved to EEPROM");
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
                         tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 18, 134, 58, TFT_BLACK);
}

void draw_normalbox_lvl2(){
                         tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 78, 134, 118, TFT_BLACK);
}

void draw_normalbox_lvl3(){
                         tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 138, 134, 178, TFT_BLACK);
}

void draw_normalbox_lvl4(){
                         tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 198, 134, 234, TFT_BLACK);
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

void DisplayFloatPID(int pagePosition, char *text, float PID, int decimal, int LoWarning, int LoAlarm, int HiWarning, int HiAlarm){       

    switch (pagePosition){

      case 1:
            tft.setTextColor(TFT_WHITE,TFT_BLUE);
            tft.setTextPadding(135);
            //tft.drawRect(0, 0, 135, 59, TFT_BLUE); // Blue rectangle
            tft.drawString(text, tft.width() / 2, textLvl1, 1);
            //check warning levels
            //if(PID < warning) draw_warningbox_lvl1();
            //else  draw_normalbox_lvl1();
            draw_normalbox_lvl1();
            //tft.setTextPadding( tft.textWidth("888.8", 2) );
            tft.setTextPadding(130);
            tft.drawFloat(PID, decimal, tft.width()/2, drawLvl1, 2);                 
            break;

      case 2:
            tft.setTextColor(TFT_WHITE,TFT_BLUE);
            tft.setTextPadding(135);
            //tft.drawRect(0, 60, 135, 119, TFT_BLUE); // Blue rectangle
            tft.drawString(text, tft.width() / 2, textLvl2, 1);
            //check warning levels
            //if(PID < warning) draw_warningbox_lvl2();
            //else  draw_normalbox_lvl2();
            draw_normalbox_lvl2();
            //tft.setTextPadding( tft.textWidth("888.8", 2) );
            tft.setTextPadding(130);
            tft.drawFloat(PID, decimal, tft.width()/2, drawLvl2, 2);                 
            break;

      case 3:
            tft.setTextColor(TFT_WHITE,TFT_BLUE);
            tft.setTextPadding(135);
            //tft.drawRect(0, 120, 135, 179, TFT_BLUE); // Blue rectangle
            tft.drawString(text, tft.width() / 2, textLvl3, 1);
            //check warning levels
            //if(PID < warning) draw_warningbox_lvl3();
            //else  draw_normalbox_lvl3();
            draw_normalbox_lvl3();
            //tft.setTextPadding( tft.textWidth("888.8", 2) );
            tft.setTextPadding(130);
            tft.drawFloat(PID, decimal, tft.width()/2, drawLvl3, 2);                 
            break;

      case 4:
            tft.setTextColor(TFT_WHITE,TFT_BLUE);
            tft.setTextPadding(135);
            //tft.drawRect(0, 180, 135, 235, TFT_BLUE); // Blue rectangle
            tft.drawString(text, tft.width() / 2, textLvl4, 1);
            //check warning levels
            //if(PID < warning) draw_warningbox_lvl4();
            //else  draw_normalbox_lvl4();
            draw_normalbox_lvl4();
            //tft.setTextPadding( tft.textWidth("888.8", 2) );
            tft.setTextPadding(130);
            tft.drawFloat(PID, decimal, tft.width()/2, drawLvl4, 2);                 
            break;
    }
}

void DisplayNumberPID(int pagePosition, float PID, const char* text, int warning, int alarm){                                                           
                              
        tft.setTextColor(TFT_WHITE,TFT_BLUE);        

        switch (pagePosition){

          case 1:
                tft.drawString(text, tft.width() / 2, textLvl1, 1);
                //check warning levels
                if(PID < warning) draw_warningbox_lvl1();
                else  draw_normalbox_lvl1();
                tft.setTextPadding( tft.textWidth("88888", 2) );
                tft.drawNumber( PID, tft.width()/2, drawLvl1, 2);                 
                break;

          case2:
                tft.drawString(text, tft.width() / 2, textLvl2, 2);
                //check warning levels
                if(PID < warning) draw_warningbox_lvl2();
                else  draw_normalbox_lvl2();
                tft.setTextPadding( tft.textWidth("88888", 3) );
                tft.drawNumber( PID, tft.width()/2, drawLvl2, 3);                 
                break;

          case 3:
                tft.drawString(text, tft.width() / 2, textLvl3, 2);
                //check warning levels
                if(PID < warning) draw_warningbox_lvl3();
                else  draw_normalbox_lvl3();
                tft.setTextPadding( tft.textWidth("88888", 3) );
                tft.drawNumber( PID, tft.width()/2, drawLvl3, 3);                 
                break;

          case 4:
                tft.drawString(text, tft.width() / 2, textLvl4, 2);
                //check warning levels
                if(PID < warning) draw_warningbox_lvl4();
                else  draw_normalbox_lvl4();
                tft.setTextPadding( tft.textWidth("88888", 3) );
                tft.drawNumber( PID, tft.width()/2, drawLvl4, 3);                 
                break;
        }
}


//-------------------------------------------------------------------------------------
//             Start of Pages content definition         
//-------------------------------------------------------------------------------------

/*///////////////// Display Page 1 //////////////////////*/
void page1(){    
                      
        DisplayFloatPID(1, "TripOdo", TripOdo, 0, 0, 0, 0, 0);
        DisplayFloatPID(2, "UsedSOC", UsedSOC, 1, 0, 0, 0, 0);
        DisplayFloatPID(3, "Net_kWh", Net_kWh, 1, 0, 0, 0, 0);
        DisplayFloatPID(4, "EstLeft_kWh", EstLeft_kWh, 1, 0, 0, 0, 0);         
}
/*///////////////// End of Display Page 1 //////////////////////*/

/*///////////////// Display Page 2 //////////////////////*/
void page2(){
          
        DisplayFloatPID(1, "CurrEnergHr", CurrEnergHr, 1, 0, 0, 0, 0);
        DisplayFloatPID(2, "Est_range", Est_range, 1, 0, 0, 0, 0);
        DisplayFloatPID(3, "CurrOPtime", CurrOPtime, 1, 0, 0, 0, 0);
        DisplayFloatPID(4, "Full_Ah", EstFull_Ah, 1, 0, 0, 0, 0);
}
/*///////////////// End of Display Page 2 //////////////////////*/

/*///////////////// Display Page 3 //////////////////////*/
void page3(){
          
        DisplayFloatPID(1, "Calc_Used", used_kwh, 1, 0, 0, 0, 0);
        DisplayFloatPID(2, "Calc_Left", left_kwh, 1, 0, 0, 0, 0);        
        DisplayFloatPID(3, "Net_kWh", Net_kWh, 1, 0, 0, 0, 0);
        DisplayFloatPID(4, "Full_kWh", EstFull_kWh, 1, 0, 0, 0, 0);                     
}
/*///////////////// End of Display Page 3 //////////////////////*/

/*///////////////// Display Page 4 //////////////////////*/
void page4(){
  
        DisplayFloatPID(1, "BattMinT", BattMinT, 1, 0, 0, 0, 00);
        DisplayFloatPID(2, "Heater", Heater, 1, 0, 0, 0, 0);
        DisplayFloatPID(3, "Power", Power, 1, 0, 0, 0, 0);
        DisplayFloatPID(4, "SOC", SOC, 1, 0, 0, 0, 0);
}
/*///////////////// End of Display Page 4 //////////////////////*/

/*///////////////// Display Page 5 //////////////////////*/
void page5(){
          
        DisplayFloatPID(1, "Max_Pwr", Max_Pwr, 1, 0, 0, 0, 0);
        DisplayFloatPID(2, "Power", Power, 1, 0, 0, 0, 0);
        DisplayFloatPID(3, "BattMinT", BattMinT, 1, 0, 0, 0, 0);
        DisplayFloatPID(4, "SOC", SOC, 1, 0, 0, 0, 0);       
       
}
/*///////////////// End of Display Page 5 //////////////////////*/

/*///////////////// Display Page 6 //////////////////////*/
void page6(){
                 
        DisplayFloatPID(1, "Trip_kWh", CurrNet_kWh, 1, 0, 0, 0, 0);
        DisplayFloatPID(2, "TripOdo", CurrTripOdo, 1, 0, 0, 0, 0);
        DisplayFloatPID(3, "TripSOC", CurrUsedSOC, 1, 0, 0, 0, 0);
        DisplayFloatPID(4, "AuxBattSOC", AuxBattSOC, 1, 0, 0, 0, 0);     
}
/*///////////////// End of Display Page 6 //////////////////////*/

/*///////////////// Display Page 7 //////////////////////*/
void page7(){
        
        DisplayFloatPID(1, "BmsSOC", BmsSOC, 1, 0, 0, 0, 0);        
        DisplayFloatPID(2, "MAXcellv", MAXcellv, 2, 0, 0, 0, 0);
        DisplayFloatPID(3, "CellVdiff", CellVdiff, 2, 0, 0, 0, 0);
        DisplayFloatPID(4, "SOH", SOH, 1, 0, 0, 0, 0);
}
/*///////////////// End of Display Page 7 //////////////////////*/

/*///////////////// Display Page 8 //////////////////////*/
void page8(){
        
        DisplayFloatPID(1, "InitCED", InitCED, 1, 0, 0, 0, 0);        
        DisplayFloatPID(2, "InitCEC", InitCEC, 1, 0, 0, 0, 0);
        DisplayFloatPID(3, "CED", CED, 1, 0, 0, 0, 0);
        DisplayFloatPID(4, "CEC", CEC, 1, 0, 0, 0, 0);
}
/*///////////////// End of Display Page 8 //////////////////////*/

/*///////////////// Display Page 9 //////////////////////*/
void page9(){
        
        DisplayFloatPID(1, "InitCDC", InitCDC, 1, 0, 0, 0, 0);        
        DisplayFloatPID(2, "InitCCC", InitCCC, 1, 0, 0, 0, 0);
        DisplayFloatPID(3, "CDC", CDC, 1, 0, 0, 0, 0);
        DisplayFloatPID(4, "CCC", CCC, 1, 0, 0, 0, 0);
}
/*///////////////// End of Display Page 9 //////////////////////*/

/*///////////////////////////////////////////////////////////////////////*/
/*                     START OF LOOP                                     */ 
/*///////////////////////////////////////////////////////////////////////*/

void loop() {
  
  ButtonLoop();

  currentTimer = millis();  
  if ((currentTimer - previousTimer >= sendInterval) && send_enabled) {    
    send_data = true;
    previousTimer = currentTimer;
  }
               
  /*/////// Read each OBDII PIDs with 100ms interval /////////////////*/

  read_data();
  /*
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {   // 100ms timer
      
      previousMillis = currentMillis;  // save the last time      
      
      read_data();  // Read Datas from OBD 
      }
*/
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
               case 7: page8(); break;
               case 8: page9(); break;                                                           
               }
      }
  /*/////// Display Setup Page/////////////////*/
  else{
      SetupMode();
      }

  /*/////// Send Data to Google Sheet /////////*/ 
  
  if (send_enabled && send_data) {
        if (WiFi.status() != WL_CONNECTED) { 
            //send_data = false;
            send_enabled = false;       
        }        
        else
        {
          makeIFTTTRequest();          
        }
    }
                     
  ResetCurrTrip();
  
  //setVessOff(SpdSelect);  //This will momentarely set an output ON to turned off the VESS //  
}
