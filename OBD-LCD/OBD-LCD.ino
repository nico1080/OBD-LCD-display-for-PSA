//Sketch by Nico1080
//This skecth was tested on a DS4 with ep6fdtx engine (MEVD17_4_4 for engine and DSG_UDS for tyre pressure)
//It can read almost all values accesible on diagbox, you just have to find correct adress (see database)
//Many thank to:
//Infizer for the help in reading diagbox file  ( https://www.drive2.ru/users/infizer/ , https://telecoder.ru/index.php )
//A good read on https://www.208gti.fr/showthread.php?4044-Diagbox-telemetry
//Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it)
//
//The function of this sktech:

//Display various data from OBD port on LCD (you need correct adress from diagbox)
//Various screens are available
//LCD backlight brightness is adjusted depending on sate of car  lightening (day/night/black panel mode). Each screen can have specific value
//To change screen you need to press the "Return" button on wheel (bottom right)
//At startup it display the init (0) screen for 10s (typre pressure oil level etc)
//Then it display the temprature (1) screen
//If you do a short press on "Return) it will switch to screen (2) (air flow) then 3 (battery) then 4 (power) and go back to 0 (init)
//
//I also implemented "screen sequence " but I don't realy use it
//if you do a long press you will go to screen 100 (sequence 0-->1) a short press will make you change screen in this sequence 100-->101-->100
//Same thing for sequence 2 (screen 200 & 201)
//A last long press will make you go back seq 0


//Hardware:
//- Arduino
//- MCP2515 can module
//- TFT_ILI9341 3.2 LCD display

//Connection

//Arduino PIN -->  TFT_ILI9341
//gnd gnd
//5v  vcc
//D13 SCL
//D11 SDA
//D7  /  RST  RES
//D8  DC
//D9  CS
//D6  BL   --> Backlight PWM
//
//Arduino PIN -->  MCP2515
//D2  INT
//D13 SCK
//D11 SI
//D12 SO
//D10 CS
//GND GND
//5v  VCC

//Connect CAN to OBD plug (PSA specific: CAN High= pin 3   CANLow= pin8)
//PIN 3: CAN-BUS Diagnostic High
//PIN 8: CAN-BUS Diagnostic Low


/////////////////////
//    Libraries    //
/////////////////////

//#include <EEPROM.h>
#include <SPI.h>
#include <mcp2515.h> // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

//#include <TFT_ST7735.h> // Hardware-specific library        https://github.com/Bodmer/TFT_ST7735
//TFT_ST7735 tft = TFT_ST7735();  // Invoke library, pins defined in User_Setup.h


#include <TFT_ILI9341.h> // Hardware-specific library      https://github.com/Bodmer/TFT_ILI9341
TFT_ILI9341 tft = TFT_ILI9341();       // Invoke custom library   --> modify 7seg to add "-" char and comment unused font to save space


/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 10   //pin connected to arduino
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_500KBPS // Diagnostic CAN bus - High Speed
#define CAN_FREQ MCP_8MHZ // Switch to 16MHZ if you have a 16Mhz module

int led_pin = 6; //Initializing Backlight PWM LED Pin



////////////////////
// Initialization //
////////////////////

MCP2515 CAN0(CS_PIN_CAN0); // OBD CAN bus

// CAN-BUS Messages
struct can_frame canMsgSnd;
struct can_frame canMsgRcv;

////////////////////
//   Variables    //
////////////////////

// My variables
bool debugCAN0 = !true;   //Debug for NAC/cirocco CAN bus
bool SerialEnabled = true;
bool PrintFPS = !true;    //Not actual FPS, but time to print display
long timerFPSCan;
long timerFPSDisplay;


int  ScreenSelect;
int LastScreenSelect;
int  SeqSelect = 0;
int LastSeqSelect;
bool ClearScreen = true;

char  Brightness;
char buf [7];
//==========================  Default Brightness setting ================
int BrightnessDay = 155;
int BrightnessNight = 40;
int BrightnessBlackpanel = 3;

float FLPressure ;
float FRPressure ;
float RLPressure ;
float RRPressure ;

long timerScreen;


int FLTemperature;
int FRTemperature;
int RLTemperature;
int RRTemperature;

float OilPressure ;
int OilTemperature;
int OilLevel;
int WaterTemperature;
float Text;
//float Torque = 300;
//int RPM =500;
//int gear;


//int VCIstate;
bool ESCbutton = false;
bool LastESCbutton;
long timerVCI;
long timerESCbutton;
bool ChangeScreen = false;
bool ChangeSeq = false;

bool initdone = false;

// CAN request is about 10ms or so. Debug parameter is optional : WaterTemperature = CanRequest(0x6A8, 0x688, 0xd4, 0x0A, 2, true); or WaterTemperature = CanRequest(0x6A8, 0x688, 0xd4, 0x0A, 2);
int CanRequest(int TxID, int  RxID, int adress1, int adress2, int Lenght, bool debug = false) {
  int result;
  long timercan;
  timercan = millis();


  canMsgSnd.can_id = TxID;
  canMsgSnd.can_dlc = 0x04 ;
  canMsgSnd.data[0] = 0x03;
  canMsgSnd.data[1] = 0x22;
  canMsgSnd.data[2] = adress1;
  canMsgSnd.data[3] = adress2;

  CAN0.sendMessage( & canMsgSnd);
  result = 0;
  if (debug) {
    Serial.print("Can ID asked:    ");
    Serial.print(TxID, HEX );
    Serial.print(":DLC=");
    Serial.print(0x04, HEX);
    char tmp[3];
    for (int i = 0 ; i <= 3; i++) {
      Serial.print(":");
      snprintf(tmp, 3, "%02X", canMsgSnd.data[i]);
      Serial.print(tmp);
    }
    Serial.println("");
  }

  do {
    // statement block
    if (CAN0.readMessage( & canMsgRcv) == MCP2515::ERROR_OK) {
      int id = canMsgRcv.can_id;
      int len = canMsgRcv.can_dlc;

      //if (id == RxID && len == (Lenght + 4) && canMsgRcv.data[0] == (Lenght + 3) && canMsgRcv.data[1] == 0x62 && canMsgRcv.data[2] == adress1 && canMsgRcv.data[3] == adress2) {
      if (id == RxID && len == (Lenght + 4) && canMsgRcv.data[1] == 0x62 && canMsgRcv.data[2] == adress1 && canMsgRcv.data[3] == adress2) {
        if (debug) {
          Serial.print("Can ID received: ");
          Serial.print((id), HEX);
          Serial.print(":DLC=");
          Serial.print((len), HEX);
          char tmp[3];
          for (int i = 0 ; i <= Lenght + 3; i++) {
            Serial.print(":");
            snprintf(tmp, 3, "%02X", canMsgRcv.data[i]);
            Serial.print(tmp);
          }
          Serial.println("");
        }

        if (Lenght == 2) {
          result = 256 * canMsgRcv.data[4] + canMsgRcv.data[5];
        }
        if (Lenght == 1) {
          result = canMsgRcv.data[4];
        }
      }
      return result;   //Break While loop and directly return result
    }
  } while (millis() - timercan < 50);  //50ms CAN timeout
  return result;  //return default value (0)

}

// Lin request is about 80ms or so. Debug parameter is optional
int Linrequest(int TxID, int  RxID, int LINadress, int adress1, int adress2, int Lenght, bool debug = false) {
  int result;
  long timercan;
  timercan = millis();


  canMsgSnd.can_id = TxID;
  canMsgSnd.can_dlc = 0x05 ;
  canMsgSnd.data[0] = LINadress;
  canMsgSnd.data[1] = 0x03;
  canMsgSnd.data[2] = 0x22;
  canMsgSnd.data[3] = adress1;
  canMsgSnd.data[4] = adress2;

  CAN0.sendMessage( & canMsgSnd);
  result = 0;

  if (debug) {
    Serial.print("LIN ID asked:    ");
    Serial.print(TxID, HEX );
    Serial.print(":DLC=");
    Serial.print(0x05, HEX);
    char tmp[3];
    for (int i = 0 ; i <= 4; i++) {
      Serial.print(":");
      snprintf(tmp, 3, "%02X", canMsgSnd.data[i]);
      Serial.print(tmp);
    }
    Serial.println("");
  }


  do {
    // statement block
    if (CAN0.readMessage( & canMsgRcv) == MCP2515::ERROR_OK) {
      int id = canMsgRcv.can_id;
      int len = canMsgRcv.can_dlc;

      if (id == RxID && len == (Lenght + 5) && canMsgRcv.data[0] == LINadress && canMsgRcv.data[2] == 0x62 && canMsgRcv.data[3] == adress1 && canMsgRcv.data[4] == adress2) {
        //if (true) {
        if (debug) {
          Serial.print("Lin ID received: ");
          Serial.print((id), HEX);
          Serial.print(":DLC=");
          Serial.print((len), HEX);
          char tmp[3];
          for (int i = 0 ; i <= Lenght + 4; i++) {
            Serial.print(":");
            snprintf(tmp, 3, "%02X", canMsgRcv.data[i]);
            Serial.print(tmp);
          }
          Serial.println("");
        }

        if (Lenght == 2) {
          result = 256 * canMsgRcv.data[5] + canMsgRcv.data[6];
        }
        if (Lenght == 1) {
          result = canMsgRcv.data[5];
        }
      }
      return result;   //Break While loop and directly return result
    }
  } while (millis() - timercan < 100);  //50ms CAN timeout
  return result;  //return default value (0)

}


void DisplayFPS (int ScreenSelect, long timerFPSCan, long timerScreen) {
  tft.setTextColor(TFT_RED, TFT_BLACK);
  //tft.setTextColor(TFT_RED);
  tft.setCursor(98, 230, 1);  tft.setTextSize(1); tft.print(ScreenSelect);   //print screen name
  tft.print(":FPS "); tft.print(timerFPSCan);
  long timerFPSDisplay = millis() - timerScreen;
  Serial.print("FPS ");   Serial.print(timerFPSCan); Serial.print(" + "); Serial.print(timerFPSDisplay - timerFPSCan); Serial.print(" = "); Serial.println(timerFPSDisplay);
  //Serial.println(); Serial.println();
  tft.print("+"); tft.print(timerFPSDisplay - timerFPSCan); tft.print("="); tft.print(timerFPSDisplay); tft.print("   ");
}


void PWMBacklight (char  Brightness, int PWMDay = BrightnessDay, int PWMNight = BrightnessNight , int PWMBlack = BrightnessBlackpanel) { //  PWMBacklight (Brightness) for default brightness; or PWMBacklight (  Brightness,255,128,40); for specific screen brightness (day, night, blackpanel)
  switch (Brightness) {
    case 'B':
      analogWrite(led_pin, PWMBlack);
      //Serial.print("backlight is BrightnessBlackpanel "); Serial.println(BrightnessBlackpanel);
      break;
    case 'N':
      analogWrite(led_pin, PWMNight);
      //Serial.print("backlight is BrightnessNight "); Serial.println(BrightnessNight);
      break;
    case 'D':
      analogWrite(led_pin, PWMDay);
      //Serial.print("backlight is BrightnessDay "); Serial.println(BrightnessDay);
      break;
    default:
      analogWrite(led_pin, PWMDay);
      //Serial.print("backlight is BrightnessDay unvalid value "); Serial.println(BrightnessDay);
      break;
  }
}


void ChooseColor(int value, int lowT, int midT, int highT ) { //set color depending of for thresold blue (navy) green, orange red
  if (value < lowT) {
    tft.setTextColor(0x03DF, TFT_BLACK);
  }
  else if (value < midT) {
    tft.setTextColor(TFT_DARKGREEN, TFT_BLACK);
  }
  else if (value < highT) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  }
  else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
}

void setup() {
  pinMode(led_pin, OUTPUT);
  analogWrite(led_pin, 0); //turn off backlight for init
  Serial.begin(SERIAL_SPEED);

  tft.init();
  tft.setRotation(1);
  Serial.println(F("LCD Initialized"));
  tft.fillScreen(TFT_BLACK);

  Serial.println("Initialization CAN");


  CAN0.reset();
  CAN0.setBitrate(CAN_SPEED, CAN_FREQ);
  //CAN0.setNormalMode();
  while (CAN0.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
    Serial.println("CAN0 Initialization wait");
  }
  Serial.println("CAN0 Initialized");
}

void loop() {

  //Init screen
  if (millis() < 10 * 1000) { //Put initing screen for 5sec
    LastScreenSelect = ScreenSelect; //store last screen before checking screen
    //ClearScreen = true;
    ScreenSelect = 0;  //screen for init
  }
  else if (!initdone) {
    initdone = true;
    ClearScreen = true;
    LastScreenSelect = ScreenSelect; //store last screen before checking screen
    ScreenSelect = 1; //screen after init
  }

  if ( (millis() - timerVCI > 200)) {    //read VCI state every XXXms  (Lin request take about 80ms) & put in in ESCbutton
    timerVCI = millis();
    LastESCbutton = ESCbutton;
    ESCbutton = bitRead(Linrequest(0x6C8, 0x628, 0x48, 0xD4, 0x0B, 1), 6); //Read ESCbutton
    //Serial.print("ESC is "); Serial.print(ESCbutton); Serial.print(" timer ");//Serial.print((millis()-timerESCbutton));
    //Serial.print(" ChangeScreen "); Serial.println(ChangeScreen);

    if (ESCbutton != LastESCbutton) {  //button change
      //Serial.print("timer ESC: "); Serial.println(millis()-timerESCbutton);
      if (LastESCbutton && (millis() - timerESCbutton) > 900) {
        ChangeSeq = true;
        //Serial.print(" long press ESC pushed for:"); Serial.println(millis()-timerESCbutton);
        //Serial.print(" bool ChangeSeq: "); Serial.println(ChangeSeq);
      }
      //      else if (LastESCbutton) {
      //        Serial.print(" short press ESC pushed for:"); Serial.println(millis()-timerESCbutton);
      //      }
      timerESCbutton = millis();
    }

    if ( LastESCbutton && (ESCbutton != LastESCbutton)) { // if was pushed & not pushed anymore)
      //Serial.print("timer ESC release: "); Serial.println(millis()-timerESCbutton);
      Serial.print("ESC release & bool ChangeSeq: "); Serial.println(ChangeSeq);
      //ChangeScreen = true;
      //timerESCbutton= millis();
      // if ( ChangeScreen) {
      //ChangeScreen = false;
      LastScreenSelect = ScreenSelect; //store last screen before checking screen
      ClearScreen = true;

      //change the screen  (sequence?)

      if (ChangeSeq) {
        switch (SeqSelect) {  //switch statement for sequence 0
          case 2:               //if last seq (3) go to seq 0 (loop)
            SeqSelect = 0; // return to xx screen
            break;
          default:         //-->increment
            SeqSelect = SeqSelect + 1;
            break;
        }
        ChangeSeq = false;
        if (SeqSelect == 0) {
          ScreenSelect = 1;
        }
        if (SeqSelect == 1) {
          ScreenSelect = 100;
        }
        if (SeqSelect == 2) {
          ScreenSelect = 200;
        }
      }
      else {
        if (SeqSelect == 0) {
          switch (ScreenSelect) {  //switch stalement for sequence 0
            case 0:               //if init screen (0) go to screen x
              ScreenSelect = 1;
              break;
            case 4:               //if last screen (3) go to seq 0 (loop)
              ScreenSelect = 0; // return to xx screen
              break;
            default:         //for other screen -->increment
              ScreenSelect = ScreenSelect + 1;
              break;
          }
        }

        if (SeqSelect == 1) {
          switch (ScreenSelect) {  //switch stalement for sequence 0
            case 101:               //if last screen (3) go to seq 0 (loop)
              ScreenSelect = 100; // return to xx screen
              break;
            default:         //for other screen -->increment
              ScreenSelect = ScreenSelect + 1;
              break;
          }
        }

        if (SeqSelect == 2) {
          switch (ScreenSelect) {  //switch stalement for sequence 0
            case 201:               //if last screen (3) go to seq 0 (loop)
              ScreenSelect = 200; // return to xx screen
              break;
            default:         //for other screen -->increment
              ScreenSelect = ScreenSelect + 1;
              break;
          }
        }
      }
    }
  }

  //Detect brigthness  values are D N or B for "Day" "Night" or "Blackpanel"

  if (!bitRead(CanRequest(0x752, 0x652, 0xD9, 0x2A, 1), 4)) {      //Brightness = 'D'; // for "Day" //Brightness = 'N'; // for "Night" //  Brightness= 'B'; // for "Blackpanel"
    Brightness = 'D'; // for "Day"
  }
  else {
    Brightness = 'N'; // for "Night"
  }
  if (bitRead(CanRequest(0x752, 0x652, 0xD9, 0x28, 1), 0)) {
    Brightness = 'B'; // for "Blackpanel"
  }




  ///init screen tyre pressure
  if ((ScreenSelect == 0) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    FLPressure = float(CanRequest(0x6AF, 0x68F, 0xD6, 0x10, 2)) / 1000;
    FRPressure = float(CanRequest(0x6AF, 0x68F, 0xD6, 0x0F, 2)) / 1000;
    RLPressure = float(CanRequest(0x6AF, 0x68F, 0xD6, 0x12, 2)) / 1000;
    RRPressure = float(CanRequest(0x6AF, 0x68F, 0xD6, 0x11, 2)) / 1000;

    //Serial.print("FL/FR/RL/RR pressure is   "); Serial.print(FLPressure, 3); Serial.print("   "); Serial.print(FRPressure, 3); Serial.print("   "); Serial.print(RLPressure, 3); Serial.print("   "); Serial.println(RRPressure, 3);

    FLTemperature = CanRequest(0x6AF, 0x68F, 0xd6, 0x0E, 1) - 50;
    FRTemperature = CanRequest(0x6AF, 0x68F, 0xd6, 0x0D, 1) - 50;
    RLTemperature = CanRequest(0x6AF, 0x68F, 0xd6, 0x0C, 1) - 50;
    RRTemperature = CanRequest(0x6AF, 0x68F, 0xD6, 0x0B, 1) - 50;

    //    FLPressure = 2.601 ;
    //    FRPressure = 0.900 ;
    //    RLPressure = 1.515;
    //    RRPressure = 2.099;
    //
    //    FLTemperature = FLTemperature+10;
    //    FRTemperature = -40;
    //    RLTemperature = 10;
    //    RRTemperature = -5;

    //Serial.print("FL/FR/RL/RR temperature is "); Serial.print(FLTemperature); Serial.print("   "); Serial.print(FRTemperature); Serial.print("   "); Serial.print(RLTemperature); Serial.print("   "); Serial.println(RRTemperature);
    //Serial.println();

    //OilTemperature = CanRequest(0x752, 0x652, 0xDB, 0x83, 1) - 40;     //BSI
    OilLevel = CanRequest(0x752, 0x652, 0xDB, 0x84, 1 );
    float BatteryV = float(CanRequest(0x752, 0x652, 0xDA, 0x46, 2)) / 1000;
    //OilLevel =OilLevel+1;
    float Text = float(CanRequest(0x752, 0x652, 0xD9, 0x12, 1) * 0.5) - 40;
    int BatLoad = CanRequest(0x752, 0x652, 0xDA, 0x21, 1);

    //Serial.print("Oil Pressure is  "); Serial.print(OilPressure, 3); Serial.print(" Bar. Oil Temperature is "); Serial.print(OilTemperature); Serial.print(" °C. Oil level is "); Serial.print(OilLevel); Serial.println("  %   ");
    //Serial.println();


    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
    PWMBacklight (Brightness);   //  PWMBacklight (Brightness) for default brightness; or PWMBacklight (  Brightness,255,128,40); for specific screen brightness (day, night, blackpanel)

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_BLACK);
      ClearScreen = false;
    }
    //tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

    tft.setCursor(5 , 10  , 7);  tft.setTextSize(1); tft.print(FLPressure, 2); tft.setCursor(110, 45, 1); tft.print(" bar    ");
    tft.setCursor(5 + 25, 10 + 53 , 1);  tft.setTextSize(2); tft.print(FLTemperature); tft.print((char)0xF7); tft.print("C  ");

    tft.setCursor(185, 10  , 7);  tft.setTextSize(1); tft.print(FRPressure, 2); tft.setCursor(290, 45, 1); tft.print(" bar");
    tft.setCursor(210, 10 + 53 , 1);  tft.setTextSize(2); tft.print(FRTemperature); tft.print((char)0xF7); tft.print("C  ");

    tft.setCursor(5 , 170, 7);  tft.setTextSize(1); tft.print(RLPressure, 2); tft.setCursor(110, 170 + 35 , 1); tft.print(" bar");
    tft.setCursor(5 + 25, 170 + 53, 1);  tft.setTextSize(2); tft.print(RLTemperature); tft.print((char)0xF7); tft.print("C  ");

    tft.setCursor(185, 170, 7);  tft.setTextSize(1); tft.print(RRPressure, 2); tft.setCursor(290, 170 + 35, 1); tft.print(" bar");
    tft.setCursor(210, 170 + 53, 1);  tft.setTextSize(2); tft.print(RRTemperature); tft.print((char)0xF7); tft.print("C  ");

    tft.setCursor(5, 100, 1);  tft.setTextSize(2); tft.print("Bat: "); tft.print(BatteryV, 3); tft.print("V    Load: "); tft.print(BatLoad); tft.print("%   ");
    tft.setCursor(5, 120, 1);  tft.setTextSize(2); tft.print("T-ext: "); tft.print(Text, 1); tft.print((char)0xF7); tft.print("C  ");
    if (OilLevel != 255) {
      tft.setCursor(5, 140, 1);  tft.setTextSize(2); tft.print("Oil: "); tft.print(OilLevel); tft.print("%   ");
      //tft.drawRect(125-1,142-1,185+2,13+2,TFT_YELLOW);

      if (OilLevel <= 10) {
        tft.setTextColor(TFT_LIGHTGREY, TFT_RED);
        tft.setCursor(5, 140, 1);  tft.print("Oil: "); tft.print(OilLevel); tft.print("% Low   ");
        tft.fillRect(150, 140, 185 + 2, 16, TFT_RED);
        //tft.fillRect(125,142,185*OilLevel/100,13,TFT_RED);
      }
      else if (OilLevel <= 25) {
        tft.drawRect(125 - 1, 142 - 1, 185 + 2, 13 + 2, TFT_RED);
        tft.fillRect(125, 142, 185 * OilLevel / 100, 13, TFT_RED);
      }
      else if (OilLevel <= 40) {
        tft.drawRect(125 - 1, 142 - 1, 185 + 2, 13 + 2, TFT_ORANGE);
        tft.fillRect(125, 142, 185 * OilLevel / 100, 13, TFT_ORANGE);
      }
      else {
        tft.drawRect(125 - 1, 142 - 1, 185 + 2, 13 + 2, TFT_GREEN);
        tft.fillRect(125, 142, 185 * OilLevel / 100, 13, TFT_GREEN);
      }
    }

    else {
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      tft.setCursor(5, 140, 1);  tft.print("Oil Level: ___"); //tft.print(OilLevel); tft.print("%   ");
      //tft.drawRect(199,139,117,17,TFT_YELLOW);
      //tft.fillRect(200,140,115*OilLevel/100,15,TFT_GREEN);
    }
    //tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);




    //Print screen name/FPS
    if (PrintFPS) {
      DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }

    tft.fillEllipse(200, 200, 2, 2, 0x0000);   //Fake item to avoid cursor blinking (dirty solution)
  }

  /// screen1  water oil
  if ((ScreenSelect == 1) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    //Read some stuff on CAN (optionnal print on serial port)

    OilPressure = (float(CanRequest(0x6A8, 0x688, 0xD4, 0x29, 2)) * 0.00076294) - 0.25;
    OilTemperature = CanRequest(0x752, 0x652, 0xDB, 0x83, 1) - 40;   //BSI
    WaterTemperature = CanRequest(0x6A8, 0x688, 0xD4, 0x0A, 2);
    //int BlackPanelHex = CanRequest(0x752, 0x652, 0xD9, 0x28, 1);
    //bool BlackPanelBool = bitRead(CanRequest(0x752, 0x652, 0xD9, 0x28, 1), 0);
    //int LightHex = CanRequest(0x752, 0x652, 0xD9, 0x2A, 1);
    //LightHex = 0x00;
    //bool LightBool = bitRead(CanRequest(0x752, 0x652, 0xD9, 0x2A, 1), 4);

    float BatteryV = float(CanRequest(0x752, 0x652, 0xDA, 0x46, 2)) / 1000;
    float BatteryC = float(CanRequest(0x752, 0x652, 0xDA, 0x45, 1)) / 10; //Voltage setting
    Text = float(CanRequest(0x752, 0x652, 0xD9, 0x12, 1) * 0.5) - 40;

    int WaterSET = CanRequest(0x6A8, 0x688, 0xD4, 0x0D, 2);  //Water setting
    int gear = CanRequest(0x6A8, 0x688, 0xD4, 0x09, 1);
    //float gear = float(CanRequest(0x6A8, 0x688, 0xD4, 0x09, 1))*0.0006;

    float Torque = float(CanRequest(0x6A8, 0x688, 0xD7, 0x28, 2)) / 16;





    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
    PWMBacklight (Brightness);

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_BLACK);
      tft.drawFastVLine(160, 0, 240, TFT_LIGHTGREY);
      tft.drawFastHLine(0, 80, 320, TFT_LIGHTGREY);
      tft.drawFastHLine(0, 160, 320, TFT_LIGHTGREY);

      //water
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(160 + 3, 3, 1);  tft.setTextSize(2); tft.print("Water:");
      tft.setCursor(160 + 100, 25, 1); tft.setTextSize(2); tft.print((char)0xF7);

      //Oil
      tft.setCursor(0 + 3, 3, 1);  tft.setTextSize(2); tft.print("Oil:");
      tft.setCursor(0 + 100, 25, 1); tft.setTextSize(2); tft.print((char)0xF7);

      //Battery
      tft.setCursor(0 + 3, 80 + 3, 1);  tft.setTextSize(2); tft.print("Bat:");
      tft.setCursor(0 + 143, 80 + 50, 1); tft.setTextSize(2); tft.print("V");

      //torque
      tft.setCursor(160 + 3, 80 + 3, 1);  tft.setTextSize(2); tft.print("Poil:");
      tft.setCursor(160 + 124, 80 + 50, 1); tft.setTextSize(2); tft.print("b");

      //T ext
      tft.setCursor(0 + 3, 160 + 3, 1);  tft.setTextSize(2); tft.print("Text:");
      tft.setCursor(0 + 143, 160 + 30, 1); tft.setTextSize(2); tft.print((char)0xF7);

      //Gear
      tft.setCursor(160 + 3, 160 + 3, 1);  tft.setTextSize(2); tft.print("Gear:");
      //tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(0 + 125, 160+30, 1); tft.setTextSize(2); tft.print((char)0xF7);


      ClearScreen = false;
    }

    //Display some stuff on LCD


    //Water
    //WaterTemperature = WaterTemperature+10; WaterSET = 105;

    itoa(WaterTemperature, buf, 10);

    //tft.setCursor(160 + 100, 10, 7);  //tft.setTextSize(1); tft.print(WaterTemperature);
    //choose color to display:
    ChooseColor(WaterTemperature, 80, 110, 115 );  //set color depending of for thresold blue (navy) green, orange red
    tft.setTextSize(1); tft.drawRightString(buf, 160 + 100, 25, 7);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(160 + 100, 55, 1); tft.setTextSize(2); tft.print(WaterSET); tft.print("  ");

    //OilTemperature
    //OilTemperature = OilTemperature+10;
    itoa(OilTemperature, buf, 10);
    //dtostrf(OilTemperature, 5, 0, buf);

    //tft.setCursor(160 + 100, 10, 7);  //tft.setTextSize(1); tft.print(WaterTemperature);
    //choose color to display:
    ChooseColor(OilTemperature, 80, 120, 150 );  //set color depending of for thresold blue (navy) green, orange red
    tft.setTextSize(1); tft.drawRightString(buf, 0 + 100, 25, 7);



    //Torque
    //Torque = 200;
    dtostrf(OilPressure, 5, 2, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0, 80 + 27, 4); // tft.setTextSize(2); tft.print(Torque,2);
    tft.setTextSize(1); tft.drawRightString(buf, 160 + 124, 80 + 25, 7);


    //Battery
    //BatteryV = 12.235; BatteryC = 12.1;
    dtostrf(BatteryV, 5, 2, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0, 80 + 27, 4); // tft.setTextSize(2); tft.print(BatteryV,2);
    tft.setTextSize(1); tft.drawRightString(buf, 0 + 140, 80 + 25, 7);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(0 + 110, 80 + 5, 1); tft.setTextSize(2); tft.print(BatteryC, 1); //tft.print("  ");


    //T ext
    //Text=10.4;
    dtostrf(Text, 7, 1, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0 + 10, 160+27, 1);  tft.setTextSize(2); tft.print(buf);//tft.print(Text,1);
    tft.setTextSize(1); tft.drawRightString(buf, 0 + 140, 160 + 25, 7);

    //Gear
    //gear=gear-1;
    if (gear >= 1 && gear <= 6) {
      tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setCursor(160 + 60, 160 + 25, 7);  tft.setTextSize(1); tft.print(gear);
    }
    else {
      tft.setTextColor(TFT_CYAN, TFT_BLACK);tft.setCursor(160 + 60, 160 + 25, 7); tft.setTextSize(1);
      tft.print("-");
    }




    //    tft.setCursor(3, 60, 1);  tft.setTextSize(1); tft.print("Oil P  "); tft.print(OilPressure, 2); tft.print(" bar    ");
    //    tft.setCursor(3, 70, 1);  tft.setTextSize(1); tft.print("Oil Temp "); tft.print(OilTemperature); tft.print((char)0xF7); tft.print("C  ");
    //        //tft.setCursor(3, 90, 1);  tft.setTextSize(1); tft.print("ESC is "); tft.print(ESCbutton); //tft.print(" VCI ");//tft.print(VCIstate);
    //    //tft.setCursor(3, 100, 1);  tft.setTextSize(1); tft.print("ChangeScreen "); tft.print(ChangeScreen);
    //    //tft.setCursor(3, 110, 1);  tft.setTextSize(1); tft.print("BlackP: "); tft.print(BlackPanelHex, HEX); tft.print(" H/B: "); tft.print(BlackPanelBool); tft.print("  ");
    //    //tft.setCursor(3, 120, 1);  tft.setTextSize(1); tft.print("Light:  "); tft.print(LightHex, HEX); tft.print(" H/B: "); tft.print(LightBool); tft.print("  ");
    //    //tft.setCursor(3, 130, 1);  tft.setTextSize(1); tft.print("Brightness  "); tft.print(Brightness); tft.print("    ");
    //    tft.setCursor(3 ,40 , 1);  tft.setTextSize(1); tft.print("BAT  "); tft.print(BatteryV, 3); tft.print("V / "); tft.print(BatteryC, 1); tft.print("V   ");
    //    tft.setCursor(3, 30, 1);  tft.setTextSize(1); tft.print("Text "); tft.print(Text,1); tft.print((char)0xF7); tft.print("C  ");

    //Print screen name/FPS
    if (PrintFPS) {
      DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }

    tft.fillCircle(318, 238, 1, 0x0000);   //Fake item to avoid cursor blinking (dirty solution)
  }

  /// screen2  Engine Air
  if ((ScreenSelect == 2) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    //Read some stuff on CAN (optionnal print on serial port)

    float Airflow = float(CanRequest(0x6A8, 0x688, 0xD4, 0x89, 2)) * 0.0152587888888889;
    float AirflowSet = float(CanRequest(0x6A8, 0x688, 0xD4, 0x8E, 2)) * 0.0152587888888889;
    int Tintake = CanRequest(0x6A8, 0x688, 0xD4, 0x14, 2);
    float PTurbo = float(CanRequest(0x6A8, 0x688, 0xD4, 0x7E, 2)) / 12500;
    float PturboSet = float(CanRequest(0x6A8, 0x688, 0xD4, 0x8D, 2)) / 12500;
    float Pmap = float(CanRequest(0x6A8, 0x688, 0xD4, 0xD9, 2)) * 0.078125 / 100;
    float PmapSet = float(CanRequest(0x6A8, 0x688, 0xD4, 0xDA, 2)) * 0.08 / 1000;
    float Tturbo = float(CanRequest(0x6A8, 0x688, 0xD4, 0x7F, 2)) * 0.0234375 - 273.15;
    float Tmap = float(CanRequest(0x6A8, 0x688, 0xD4, 0x11, 1)) * 0.75 - 48;




    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
    PWMBacklight (Brightness);

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_BLACK);
      tft.drawFastVLine(160, 0, 240, TFT_LIGHTGREY);
      tft.drawFastHLine(0, 80, 320, TFT_LIGHTGREY);
      tft.drawFastHLine(0, 160, 320, TFT_LIGHTGREY);

      //Intake temp
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(160 + 3, 3, 1);  tft.setTextSize(2); tft.print("Intake temp:");
      tft.setCursor(160 + 100, 25, 1); tft.setTextSize(2); tft.print((char)0xF7);

      //Airflow
      tft.setCursor(0 + 3, 3, 1);  tft.setTextSize(2); tft.print("Airflow:");
      tft.setCursor(0 + 100, 50, 1); tft.setTextSize(2); tft.print("g/s");

      //PTurbo
      tft.setCursor(0 + 3, 80 + 3, 1);  tft.setTextSize(2); tft.print("PTurbo:");
      tft.setCursor(0 + 123, 80 + 50, 1); tft.setTextSize(2); tft.print("bar");

      //TTurbo
      tft.setCursor(160 + 3, 80 + 3, 1);  tft.setTextSize(2); tft.print("Tturbo:");
      tft.setCursor(160 + 145, 80 + 25, 1); tft.setTextSize(2); tft.print((char)0xF7);

      //Pmap
      tft.setCursor(0 + 3, 160 + 3, 1);  tft.setTextSize(2); tft.print("Pmap:");
      tft.setCursor(0 + 123, 160 + 50, 1); tft.setTextSize(2); tft.print("bar");

      //Tmap
      tft.setCursor(160 + 3, 160 + 3, 1);  tft.setTextSize(2); tft.print("Tmap:");
      tft.setCursor(160 + 145, 160 + 25, 1); tft.setTextSize(2); tft.print((char)0xF7);


      ClearScreen = false;
    }

    //Display some stuff on LCD

    char buf [7];

    //Airflow
    //Airflow = Airflow+150; AirflowSet=150;
    dtostrf(Airflow, 3, 0, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextSize(1); tft.drawRightString(buf, 0 + 95, 25, 7);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(0 + 110, 0 + 5, 1); tft.setTextSize(2); tft.print(AirflowSet, 0); //tft.print("  ");

    //Tintake
    //Tintake = Tintake-20;
    itoa(Tintake, buf, 10);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(1); tft.drawRightString(buf, 160 + 100, 25, 7);

    //PTurbo
    //PTurbo = 2.235;  PturboSet=2.500;
    dtostrf(PTurbo, 5, 2, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0, 80 + 27, 4); // tft.setTextSize(2); tft.print(BatteryV,2);
    tft.setTextSize(1); tft.drawRightString(buf, 0 + 122, 80 + 25, 7);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(0 + 110, 80 + 5, 1); tft.setTextSize(2); tft.print(PturboSet, 2); //tft.print("  ");

    //Pmap
    //Pmap= Pmap+0; PmapSet=PmapSet;
    dtostrf(Pmap, 5, 2, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0 + 10, 160+27, 1);  tft.setTextSize(2); tft.print(buf);//tft.print(Text,1);
    tft.setTextSize(1); tft.drawRightString(buf, 0 + 122, 160 + 25, 7);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(0 + 110, 160 + 5, 1); tft.setTextSize(2); tft.print(PmapSet, 2); //tft.print("  ");

    //Tturbo
    //Tturbo = Tturbo+400;
    dtostrf(Tturbo, 3, 1, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextSize(1); tft.drawRightString(buf, 160 + 145, 80 + 25, 7);

    //Tmap
    //Tmap = Tmap+80;
    dtostrf(Tmap, 3, 1, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextSize(1); tft.drawRightString(buf, 160 + 145, 160 + 25, 7);



    //    tft.setCursor(3, 60, 1);  tft.setTextSize(1); tft.print("Oil P  "); tft.print(OilPressure, 2); tft.print(" bar    ");
    //    tft.setCursor(3, 70, 1);  tft.setTextSize(1); tft.print("Oil Temp "); tft.print(OilTemperature); tft.print((char)0xF7); tft.print("C  ");
    //        //tft.setCursor(3, 90, 1);  tft.setTextSize(1); tft.print("ESC is "); tft.print(ESCbutton); //tft.print(" VCI ");//tft.print(VCIstate);
    //    //tft.setCursor(3, 100, 1);  tft.setTextSize(1); tft.print("ChangeScreen "); tft.print(ChangeScreen);
    //    //tft.setCursor(3, 110, 1);  tft.setTextSize(1); tft.print("BlackP: "); tft.print(BlackPanelHex, HEX); tft.print(" H/B: "); tft.print(BlackPanelBool); tft.print("  ");
    //    //tft.setCursor(3, 120, 1);  tft.setTextSize(1); tft.print("Light:  "); tft.print(LightHex, HEX); tft.print(" H/B: "); tft.print(LightBool); tft.print("  ");
    //    //tft.setCursor(3, 130, 1);  tft.setTextSize(1); tft.print("Brightness  "); tft.print(Brightness); tft.print("    ");
    //    tft.setCursor(3 ,40 , 1);  tft.setTextSize(1); tft.print("BAT  "); tft.print(BatteryV, 3); tft.print("V / "); tft.print(BatteryC, 1); tft.print("V   ");
    //    tft.setCursor(3, 30, 1);  tft.setTextSize(1); tft.print("Text "); tft.print(Text,1); tft.print((char)0xF7); tft.print("C  ");

    //Print screen name/FPS
    if (PrintFPS) {
      DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }

    tft.fillCircle(318, 238, 1, 0x0000);   //Fake item to avoid cursor blinking (dirty solution)
  }

  // screen3  //battery
  if ((ScreenSelect == 3) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    //Read some stuff on CAN (optionnal print on serial port)

    float BatteryV = float(CanRequest(0x752, 0x652, 0xDA, 0x46, 2)) / 1000;
    float BatteryC = float(CanRequest(0x752, 0x652, 0xDA, 0x45, 1)) / 10; //Voltage setting
    int BatTemp = CanRequest(0x752, 0x652, 0xDA, 0x47, 1) - 40;
    int BatLoad = CanRequest(0x752, 0x652, 0xDA, 0x21, 1);
    float BatResit = float(CanRequest(0x752, 0x652, 0xDA, 0x4C, 2)) / 100;
    float BatLeakage = float(CanRequest(0x752, 0x652, 0xDA, 0x49, 2)) * 0.005;

    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
    PWMBacklight (Brightness);

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_BLACK);
      ClearScreen = false;
    }

    //Display some stuff on LCD
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(3 , 20 , 1);  tft.setTextSize(1); tft.print("BAT  "); tft.print(BatteryV, 3); tft.print("V / "); tft.print(BatteryC, 1); tft.print("V   ");
    tft.setCursor(3 , 30 , 1);  tft.setTextSize(1); tft.print("Temprature "); tft.print(BatTemp); tft.print((char)0xF7); tft.print("C  ");
    tft.setCursor(3 , 40 , 1);  tft.setTextSize(1); tft.print("BatLoad "); tft.print(BatLoad); tft.print("%  ");
    tft.setCursor(3 , 50 , 1);  tft.setTextSize(1); tft.print("BatResit "); tft.print(BatResit, 2); tft.print(" mOhm  ");
    tft.setCursor(3 , 60 , 1);  tft.setTextSize(1); tft.print("BatLeakage "); tft.print(BatLeakage, 3); tft.print(" ?  ");



    //Print screen name/FPS
    if (PrintFPS) {
      DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }

    tft.fillEllipse(50, 50, 2, 2, 0x0000);   //Fake item to avoid cursor blinking (dirty solution)
  }


  // screen4  //fuel
  if ((ScreenSelect == 4) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    //Read some stuff on CAN (optionnal print on serial port)
    float FuelLevel = float(CanRequest(0x752, 0x652, 0xD8, 0xC4, 2)) / 100;
    float FuelInst = float(CanRequest(0x752, 0x652, 0xD8, 0xC7, 2)) / 10;
    float FuelTrip1 = float(CanRequest(0x752, 0x652, 0xD8, 0xC8, 2)) / 10;
    //float FuelTrip2 = float(CanRequest(0x752, 0x652, 0xD8, 0xC9, 2))/10;
    //float FuelFast = float(CanRequest(0x752, 0x652, 0xD8, 0xCA, 2))/5;
    int RemainKm = CanRequest(0x752, 0x652, 0xD8, 0xE1, 2);
    //    int EngineState = CanRequest(0x6A8, 0x688, 0xD4, 0x12, 1);
    //    float AFR = float(CanRequest(0x6A8, 0x688, 0xD6, 0x35, 2))*0.00024414062;
    float Torque = float(CanRequest(0x6A8, 0x688, 0xD7, 0x28, 2))/16;
    //    float TorqueAir = float(CanRequest(0x6A8, 0x688, 0xD4, 0xD2, 2))/10;
    //    float TorqueDriver = float(CanRequest(0x6A8, 0x688, 0xD4, 0x07, 2))/128;
    //    float TorqueAvance = float(CanRequest(0x6A8, 0x688, 0xD6, 0x2A, 2))/16;
    int RPM = CanRequest(0x6A8, 0x688, 0xD4, 0x00, 2);
    //Torque=Torque-25;
//    RPM = RPM + 100;
    float Power = Torque * RPM / 9.5488 / 1000;







    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
    PWMBacklight (Brightness);

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_BLACK);
      ClearScreen = false;
      tft.drawFastVLine(160, 0, 160, TFT_LIGHTGREY);
      tft.drawFastHLine(0, 80, 320, TFT_LIGHTGREY);
      tft.drawFastHLine(0, 160, 320, TFT_LIGHTGREY);

      //Fuel inst
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(0 + 3, 3, 1);  tft.setTextSize(2); tft.print("Fuel Inst:");
      tft.setCursor(0 + 120, 62, 1); tft.setTextSize(1); tft.print("L/100");

      //Fuel trip1
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(0 + 3, 80 + 3, 1);  tft.setTextSize(2); tft.print("Fuel Trip1:");
      tft.setCursor(0 + 120, 80 + 62, 1); tft.setTextSize(1); tft.print("L/100");

      //Fuel level
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(160 + 3, 3, 1);  tft.setTextSize(2); tft.print("Fuel:");
      tft.setCursor(160 + 130, 62, 1); tft.setTextSize(2); tft.print("L");

      //Remain Km
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(160 + 3, 80 + 3, 1);  tft.setTextSize(2); tft.print("Remain km:");
      tft.setCursor(160 + 130, 80 + 62, 1); tft.setTextSize(2); tft.print("km");

      //Power torque
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.setCursor(0 + + 3, 160 + 3, 1);  tft.setTextSize(2); tft.print("Torque & Power:");
      tft.setCursor(0 + 60, 160 + 45, 1); tft.setTextSize(2); tft.print("Nm x ");
      tft.setCursor(0 + 165, 160 + 45, 1); tft.setTextSize(2); tft.print("RPM = ");
      tft.setCursor(0 + 290, 160 + 45, 1); tft.setTextSize(2); tft.print("Kw");
      tft.setCursor(0 + 290, 160 + 15, 1); tft.setTextSize(2); tft.print("HP");
    }

    //Display some stuff on LCD

    // fuel inst
    //FuelFast=5.6;
        if (FuelInst >= 0 && FuelInst <= 100) {
      dtostrf(FuelInst, 5, 1, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0 + 10, 160+27, 1);  tft.setTextSize(2); tft.print(buf);//tft.print(Text,1);
    tft.setTextSize(1); tft.drawRightString(buf, 0 + 120, 0 + 25, 7);
    }
    else {
      dtostrf(FuelInst, 4, 1, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0 + 10, 160+27, 1);  tft.setTextSize(2); tft.print(buf);//tft.print(Text,1);
    tft.setTextSize(1); tft.drawRightString("---", 0 + 120, 0 + 25, 7);
    }



    // FuelTrip1
    //FuelTrip1=15.6;
    dtostrf(FuelTrip1, 3, 1, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0 + 10, 160+27, 1);  tft.setTextSize(2); tft.print(buf);//tft.print(Text,1);
    tft.setTextSize(1); tft.drawRightString(buf, 0 + 120, 80 + 25, 7);


    // fuel level
    //FuelLevel=50;
    dtostrf(FuelLevel, 3, 1, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0 + 10, 160+27, 1);  tft.setTextSize(2); tft.print(buf);//tft.print(Text,1);
    tft.setTextSize(1); tft.drawRightString(buf, 160 + 130, 0 + 25, 7);

    // RemainKm
    //RemainKm=5000;
    dtostrf(RemainKm, 4, 0, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); //tft.setCursor(0 + 10, 160+27, 1);  tft.setTextSize(2); tft.print(buf);//tft.print(Text,1);
    tft.setTextSize(1); tft.drawRightString(buf, 160 + 130, 80 + 25, 7);


    //Power
    dtostrf(Torque, 3, 0, buf);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(3); tft.drawRightString(buf, 0 + 5, 160 + 40, 1);
    tft.setCursor(0 + 165, 160 + 45, 1);  tft.setTextSize(2);
    dtostrf(RPM, 4, 0, buf); tft.drawRightString(buf, 0 + 115, 160 + 45, 1);
    tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextSize(3);
    dtostrf(Power, 3, 0, buf); tft.drawRightString(buf, 0 + 230, 160 + 40, 1);
    dtostrf((Power / 0.736), 3, 0, buf); tft.drawRightString(buf, 0 + 230, 160 + 10, 1);




    //    tft.setCursor(3 , 0 , 1);  tft.setTextSize(2); tft.print("FuelLevel  "); tft.print(FuelLevel, 1); tft.print(" L   ");
    //    tft.setCursor(3 , 20 , 1);  tft.setTextSize(2); tft.print("FuelInst "); tft.print(FuelInst,1); tft.print(" L/100  ");
    //    tft.setCursor(3 , 40 , 1);  tft.setTextSize(2); tft.print("FuelTrip1 "); tft.print(FuelTrip1,1); tft.print(" L/100  ");
    //    tft.setCursor(3 , 60 , 1);  tft.setTextSize(2); tft.print("FuelTrip2 "); tft.print(FuelTrip2,1); tft.print(" L/100  ");
    //    tft.setCursor(3 , 80 , 1);  tft.setTextSize(2); tft.print("FuelFast "); tft.print(FuelFast,1); tft.print(" L/100  ");
    //    tft.setCursor(3 , 100 , 1);  tft.setTextSize(2); tft.print("RemainKm "); tft.print(RemainKm); tft.print(" km  ");
    //    tft.setCursor(3 , 120 , 1);  tft.setTextSize(2); tft.print("EngineState "); tft.print(EngineState);// tft.print(" km  ");
    //    tft.setCursor(3 , 140 , 1);  tft.setTextSize(2); tft.print("AFR "); tft.print(AFR,3); tft.print(" ???  ");
    //
    //    //RPM=6000; Torque=285;
    //    tft.setCursor(3 , 160 , 1);  tft.setTextSize(2); tft.print("T: "); tft.print(Torque,0); tft.print("x");tft.print(RPM); tft.print(" ="); tft.print(Torque*RPM/ 9.5488/1000,0);tft.print(" Kw   ");
    //    tft.setCursor(3 , 180 , 1);  tft.setTextSize(2); tft.print("Tair: "); tft.print(TorqueAir,0); tft.print("x");tft.print(RPM); tft.print(" ="); tft.print(TorqueAir*RPM/ 9.5488/1000,0);tft.print(" Kw   ");
    //    tft.setCursor(3 , 200 , 1);  tft.setTextSize(2); tft.print("Tdriv: "); tft.print(TorqueDriver,0); tft.print("x");tft.print(RPM); tft.print(" ="); tft.print(TorqueDriver*RPM/ 9.5488/1000,0);tft.print(" Kw   ");
    //    tft.setCursor(3 , 220 , 1);  tft.setTextSize(2); tft.print("Tadv: "); tft.print(TorqueAvance,0); tft.print("x");tft.print(RPM); tft.print(" ="); tft.print(TorqueAvance*RPM/ 9.5488/1000,0);tft.print(" Kw   ");
    //




    //Print screen name/FPS
    if (PrintFPS) {
      DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }

    tft.fillEllipse(318, 0, 2, 2, 0x0000);   //Fake item to avoid cursor blinking (dirty solution)
  }

  // screen100  (
  if ((ScreenSelect == 100) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    //Read some stuff on CAN (optionnal print on serial port)

    //xxxxxx

    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
    PWMBacklight (Brightness);

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_BLUE);
      ClearScreen = false;
    }

    //Display some stuff on LCD
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(3, 60, 1);  tft.setTextSize(1); tft.print("screen 100");



    //Print screen name/FPS
    if (PrintFPS) {
      DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }

  }

  // screen101
  if ((ScreenSelect == 101) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    //Read some stuff on CAN (optionnal print on serial port)

    //xxxxxx

    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
    PWMBacklight (Brightness);

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_BLUE);
      ClearScreen = false;
    }

    //Display some stuff on LCD
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(3, 60, 1);  tft.setTextSize(1); tft.print("101");



    //Print screen name/FPS
    if (PrintFPS) {
      DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }

  }

  // screen200  (
  if ((ScreenSelect == 200) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    //Read some stuff on CAN (optionnal print on serial port)

    //xxxxxx

    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
    PWMBacklight (Brightness);

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_YELLOW);
      ClearScreen = false;
    }

    //Display some stuff on LCD
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(3, 60, 1);  tft.setTextSize(1); tft.print("screen 200");



    //Print screen name/FPS
    if (PrintFPS) {
      DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }
  }

  // screen201
  if ((ScreenSelect == 201) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    //Read some stuff on CAN (optionnal print on serial port)

    //xxxxxx

    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
    PWMBacklight (Brightness);

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_YELLOW);
      ClearScreen = false;
    }

    //Display some stuff on LCD
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(3, 60, 1);  tft.setTextSize(1); tft.print("screen 201");



    //Print screen name/FPS
    if (PrintFPS) {
      DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }
  }


}

/*
   ============    Generic screen  ===============
   Name can be any number (integer), best to keep short name for debug

  // screen3
  if ((ScreenSelect == 3) && (millis() - timerScreen > 1000)) {    //screen name + refresh rate
    timerScreen = millis();
    //Serial.print("Screen "); Serial.println(ScreenSelect);

    //Read some stuff on CAN (optionnal print on serial port)

    //xxxxxx

    if (PrintFPS) {
      timerFPSCan = millis() - timerScreen;
    }

    // Set Backlight (setting can be overide for each screen)
  PWMBacklight (Brightness);

    if ( ClearScreen) { //Clear screen if needed  & reset ClearScreen  Optionnal draw background here
      tft.fillScreen(TFT_BLACK);
      ClearScreen = false;
    }

    //Display some stuff on LCD
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(3, 60, 1);  tft.setTextSize(1); tft.print("xxxx"); tft.print(" yyy    ");



    //Print screen name/FPS
    if (PrintFPS) {
        DisplayFPS ( ScreenSelect,  timerFPSCan,  timerScreen);
    }

    tft.fillEllipse(50, 50, 2, 2, 0x0000);   //Fake item to avoid cursor blinking (dirty solution)
  }


*/
