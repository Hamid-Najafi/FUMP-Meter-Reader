/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "lmic_util.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/mcpwm.h"
const char* OBISCode =""; 
unsigned char DATA1[240];
byte ETX_Byte=0x03;
char ETX=(char) ETX_Byte;
byte STX_Byte=0x02;
char STX=(char) STX_Byte;
byte ACK_Byte=0x06;
char ACK=(char) ACK_Byte;
char Ack_Data[]={ACK,'0','5','0','\r','\n'};
byte data[]={0x2F,0x3F,0x21,0x0D,0x0A};
int baudRate=300;
String OBIS="";
int period = 3000;
int DATACount=0;
String DATA;
unsigned long time_now = 0;
#define PROBE Serial2
#define JoinedLED 32
#define UplinkLED 33
#define ReadyLED 21
#define DEBUG 1
#ifdef DEBUG
#define debugln(x) Serial.println(x)
#else
#define debugln(x) 
#endif
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xdb, 0x7a, 0xcb, 0x50, 0xf3, 0x14, 0x87, 0xb7 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {0xaa, 0x41, 0xa9, 0xe4, 0x25, 0xe0, 0x66, 0x8d, 0xc9, 0x28, 0xc3, 0x87, 0xe9, 0x26, 0x2f, 0x6e };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;
static osjob_t ProbeJob;
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;
bool stringComplete=false;
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 13,
    .dio = {26, 22, 25},
};

void onEvent (ev_t ev) {
    debugln(os_getTime());
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            debugln(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            debugln(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            debugln(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            debugln(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            debugln(F("EV_JOINING"));
            break;
        case EV_JOINED:
            debugln(F("EV_JOINED"));
            digitalWrite(JoinedLED,LOW);
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            debugln(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            debugln(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            debugln(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            debugln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              debugln(F("Received ack"));
            if (LMIC.dataLen) {
              debugln(F("Received "));
              debugln(LMIC.dataLen);
              debugln(F(" bytes of payload"));
            }
            
            break;
        case EV_LOST_TSYNC:
            debugln(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            debugln(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            debugln(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            debugln(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            debugln(F("EV_LINK_ALIVE"));
            break;
         default:
            debugln(F("Unknown event"));
            break;
    }
}

static void ProbeTransmit (osjob_t* j){
  char inChar;
  int temp;
  int count=0;
  bool ModelFlag=false;
  bool DataFlag=false;
  String OBISCodetemp;
  const char *ss="";
  debugln("Probe transmit job started");
  OBIS.clear();
  PROBE.write(data,5); 
  //Serial.write(data,5); 
  int i=0;
  debugln("Sent to meter");
  ledcAttachPin(ReadyLED, 0);
  while(1){
    if(PROBE.available()){
      ledcWrite(0, 127);
      delay(1200);
      String s=PROBE.readStringUntil(ETX);
      OBIS+=s;
      count++;
      ss=s.c_str();
      
      if(ss[0]=='/'||ss[1]=='/'||ss[2]=='/'){
          debugln("Model recieved");
          ModelFlag=true;
          OBISCode=s.c_str();
          
      }
      if(ss[0]==STX){
          debugln("Data recieved");
         OBISCode=s.c_str();
          
          DataFlag=true;
      }
      if(count<2){
        if(!ModelFlag){
          debugln("Restarting Transmit");
          //PROBE.write(data,5);
        }
      }
      Serial.print(OBIS);
      if(ModelFlag&&DataFlag){
          OBISCode=OBISCodetemp.c_str();
          // OBISCode = ( char*) OBISCodetemp.c_str(); // cast from string to unsigned char*  
          
          os_setTimedCallback(j, os_getTime()+sec2osticks(60), ProbeTransmit);
          os_setCallback(&sendjob,do_send);
          // send to lora
          break;
      }
      
       // memset(OBISCode,'\0',sizeof(OBISCode));
        //PROBE.write(data,5);
             
       
    }
  }  
 
}

void do_send(osjob_t* j){
  debugln("Lora  send job started");
   ledcDetachPin(ReadyLED);
   DATA.clear();
   int OBISLEN=0;
   OBISLEN=OBIS.length();
   static uint8_t payload[51];
   byte tempLow;
   byte tempHigh;
   /////////////////////////////
   int AbsEnergyPos=0;
   String AbsEnergyOBISCode="1-0:15.8.0.255(" ;
   String AbsEnergyString;
   float AbsEnergy;
   uint16_t AbsEnergyInt;
   /////////////////////////////
   int VoltageL1Pos=0;
   String VoltageL1OBISCode="1-0:32.7.0.255(" ;
   String VoltageL1String;
   float VoltageL1;
   uint16_t VoltageL1Int;
   ////////////////////////////
   int VoltageL2Pos=0;
   String VoltageL2OBISCode="1-0:52.7.0.255(" ;
   String VoltageL2String;
   float VoltageL2;
   uint16_t VoltageL2Int;
   ///////////////////////////
   int VoltageL3Pos=0;
   String VoltageL3OBISCode="1-0:72.7.0.255(" ;
   String VoltageL3String;
   float VoltageL3;
   uint16_t VoltageL3Int;
   //////////////////////////
   int CurrentL1Pos=0;
   String CurrentL1OBISCode="1-0:31.7.0.255(" ;
   String CurrentL1String;
   float CurrentL1;
   uint16_t CurrentL1Int;
   /////////////////////////
   int CurrentL2Pos=0;
   String CurrentL2OBISCode="1-0:51.7.0.255(" ;
   String CurrentL2String;
   float CurrentL2;
   uint16_t CurrentL2Int;  
   ////////////////////////
   int CurrentL3Pos=0;
   String CurrentL3OBISCode="1-0:71.7.0.255(" ;
   String CurrentL3String;
   float CurrentL3;
   uint16_t CurrentL3Int;   
   ////////////////////////   
   VoltageL1Pos=OBIS.indexOf(VoltageL1OBISCode)+VoltageL1OBISCode.length();
   VoltageL1String=OBIS.substring(VoltageL1Pos,VoltageL1Pos+6);
   debugln(VoltageL1String);
   VoltageL1=VoltageL1String.toFloat();
   debugln(VoltageL1);
   VoltageL1=VoltageL1/1000;
   VoltageL1Int= LMIC_f2uflt16(VoltageL1);
   debugln(VoltageL1Int);
   tempLow = lowByte(VoltageL1Int);
   tempHigh = highByte(VoltageL1Int);
   payload[0]=tempLow;
   payload[1]=tempHigh;
   ///////////////////////
   VoltageL2Pos=OBIS.indexOf(VoltageL2OBISCode)+VoltageL2OBISCode.length();
   VoltageL2String=OBIS.substring(VoltageL2Pos,VoltageL2Pos+6);
   debugln(VoltageL2String);
   VoltageL2=VoltageL2String.toFloat();
   
   VoltageL2=VoltageL2/1000;
   debugln(VoltageL2);
   VoltageL2Int= LMIC_f2uflt16(VoltageL2);
   debugln(VoltageL2Int);
   tempLow = lowByte(VoltageL2Int);
   tempHigh = highByte(VoltageL2Int);
   payload[2]=tempLow;
   payload[3]=tempHigh;
   //////////////////////
   VoltageL3Pos=OBIS.indexOf(VoltageL3OBISCode)+VoltageL3OBISCode.length();
   VoltageL3String=OBIS.substring(VoltageL3Pos,VoltageL3Pos+6);
   debugln(VoltageL3String);
   VoltageL3=VoltageL3String.toFloat();
   debugln(VoltageL3);
   VoltageL3=VoltageL3/1000;
   VoltageL3Int= LMIC_f2uflt16(VoltageL3);
   debugln(VoltageL1Int);
   tempLow = lowByte(VoltageL3Int);
   tempHigh = highByte(VoltageL3Int);
   payload[4]=tempLow;
   payload[5]=tempHigh;
   /////////////////////
   CurrentL1Pos=OBIS.indexOf(CurrentL1OBISCode)+CurrentL1OBISCode.length();
   CurrentL1String=OBIS.substring(CurrentL1Pos,CurrentL1Pos+6);
   debugln(CurrentL1String);
   CurrentL1=CurrentL1String.toFloat();
   debugln(CurrentL1);
   CurrentL1=CurrentL1/1000;
   CurrentL1Int= LMIC_f2uflt16(CurrentL1);
   debugln(CurrentL1Int);
   tempLow = lowByte(CurrentL1Int);
   tempHigh = highByte(CurrentL1Int);
   payload[6]=tempLow;
   payload[7]=tempHigh;
   /////////////////////
   CurrentL2Pos=OBIS.indexOf(CurrentL2OBISCode)+CurrentL2OBISCode.length();
   CurrentL2String=OBIS.substring(CurrentL2Pos,CurrentL2Pos+6);
   debugln(CurrentL2String);
   CurrentL2=CurrentL2String.toFloat();
   debugln(CurrentL2);
   CurrentL2=CurrentL2/1000;
   CurrentL2Int= LMIC_f2uflt16(CurrentL2);
   debugln(CurrentL2Int);
   tempLow = lowByte(CurrentL2Int);
   tempHigh = highByte(CurrentL2Int);
   payload[8]=tempLow;
   payload[9]=tempHigh;
   /////////////////////
   CurrentL3Pos=OBIS.indexOf(CurrentL3OBISCode)+CurrentL3OBISCode.length();
   CurrentL3String=OBIS.substring(CurrentL3Pos,CurrentL3Pos+6);
   debugln(CurrentL3String);
   CurrentL3=CurrentL3String.toFloat();
   debugln(CurrentL3);
   CurrentL3=CurrentL3/1000;
   CurrentL3Int= LMIC_f2uflt16(CurrentL3);
   debugln(CurrentL3Int);
   tempLow = lowByte(CurrentL3Int);
   tempHigh = highByte(CurrentL3Int);
   payload[10]=tempLow;
   payload[11]=tempHigh;
   /////////////////////
   AbsEnergyPos=OBIS.indexOf(AbsEnergyOBISCode)+AbsEnergyOBISCode.length();
   AbsEnergyString=OBIS.substring(AbsEnergyPos,AbsEnergyPos+9);
   debugln(AbsEnergyString);
   AbsEnergy=AbsEnergyString.toFloat();
   debugln(AbsEnergy);
   AbsEnergy=AbsEnergy/50000;
   debugln(AbsEnergy);
   AbsEnergyInt= LMIC_f2uflt16(AbsEnergy);
   debugln(AbsEnergyInt);
   tempLow = lowByte(AbsEnergyInt);
   tempHigh = highByte(AbsEnergyInt);
   payload[12]=tempLow;
   payload[13]=tempHigh;
   //////////////////////////////
    debugln("Printing Payload");
//    for (size_t i = 0; i < sizeof(payload); i++)
//    {
//        debugln(payload[i],HEX);
//    }
    
    // Check if there is not a current TX/RX job running
    if(LMIC.opmode & OP_TXRXPEND) {
        debugln(F("OP_TXRXPEND, not sending"));
    } else {
          LMIC_setTxData2(1,payload,14, 0);
        debugln(F("Packet queued"));
        delay(1500);
        
        
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    ledcAttachPin(ReadyLED, 0);
    ledcSetup(0, 1, 8);
    ledcWrite(0, 255);

    Serial.begin(115200);
    // Rx = 2 , Tx = 4
    PROBE.begin(baudRate,SERIAL_7E1,2,4); 
    debugln(F("Starting"));
    pinMode(JoinedLED,OUTPUT);
    pinMode(UplinkLED,OUTPUT);
    pinMode(ReadyLED,OUTPUT);
    digitalWrite(ReadyLED,HIGH);
    digitalWrite(JoinedLED,HIGH);
    digitalWrite(UplinkLED,HIGH);
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

//     LMIC init();
   os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
   LMIC_reset();
    //LMIC_startJoining ();
    
    // Start job (sending automatically starts OTAA too)
    ProbeTransmit(&ProbeJob);
}

void loop() {
    os_runloop_once();
}
