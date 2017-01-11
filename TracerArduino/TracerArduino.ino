#include <SPI.h> //used for LoRa radio
#include "Lora_Library.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//10DOF Sensor Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>

/**AHRS Variables**/
// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

// Update this with the correct SLP for accurate altitude measurements
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/**GENERAL VARIABLES**/
enum FlightMode {PREFLIGHT, LAUNCH, LAND};

FlightMode currentFlightMode = LAUNCH;

#define PIN_CAMERA 11 //pin to set high for camera recording
#define dt .01 //sec sample rate for complimentary filter calculations

bool gpsEnabled = true; //WARNING: Changing this bool currently does not disable sensors correctly!
bool dofEnabled = true; //WARNING: Changing this bool currently does not disable sensors correctly!

float sendInterval = 500; //time in milliseconds between data logs and sends
float timer = 0; //DO NOT CHANGE THIS

void setup() {
  Serial.begin(115200); //start Serial output for debugging

  pinMode(PIN_CAMERA, OUTPUT); //set the camera pin to output
  digitalWrite(PIN_CAMERA, LOW);
  
  /**SETUP LORA**/
  loraSetup();

  /**SETUP GPS**/
  if(gpsEnabled)
  {
    //TODO Implement when new GPS unit received
  }

  /**SETUP ACCEL/GYRO**/
  if(dofEnabled)
  {
    // Initialize the sensors.
    accel.begin();
    mag.begin();
    bmp.begin();
  }

  timer = millis();
}

void loop() {
  sensors_vec_t   orientation;
  String loraMessage = "";
  String toSend = "";
  float roll, pitch, temperature, altitude;
  
  // Calculate orientation using AHRS library
  if (ahrs.getOrientation(&orientation)){
    roll = orientation.roll;
    pitch = orientation.pitch;
  } else {
    //SENSOR ERROR
    //Serial.println("");
  }

  // Calculate the altitude using the barometric pressure sensor
  sensors_event_t bmp_event;
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure){
    //Get ambient temperature in Celsius
    bmp.getTemperature(&temperature);

    //Get altitude
    altitude = bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature); 
  } else {
    //SENSOR ERROR
    //Serial.println("");
  }

  if(millis() - timer > sendInterval){
    timer = millis(); //Reset timer
    
    switch(currentFlightMode){
      
      case PREFLIGHT:
        //Send data to Raspi
        //TODO Figure out formatting for data to send to Connor
        
        //Setup LoRa to wait for launch signal
        loraMessage = receiveLora();
  
        if(loraMessage == "L"){
          currentFlightMode = LAUNCH;
        }
      break;
  
      case LAUNCH:
        digitalWrite(PIN_CAMERA, HIGH);

        //Prepare string to send
        toSend = "KD2LUP"; //Prefix messages with callsign
        toSend = "";
        toSend = toSend + "N"; //Hardcode 
        toSend = toSend + "0.00000"; //Hardcode latitude 0
        toSend = toSend + "N"; //Hardcode 
        toSend = toSend + "0.00000"; //Hardcode longitude 0
        toSend = toSend + "N"; //Hardcode
        toSend = toSend + "0.00000"; //Hardcode altitude
        toSend = toSend + "Y"; //Hardcode
        toSend = toSend + pitch;
        toSend = toSend + "Y";
        toSend = toSend + roll;
        
        //sendLora(toSend);
        Serial.println(toSend);
  
      break;
  
      case LAND:
        digitalWrite(PIN_CAMERA, LOW); //tell camera to stop recording

      break;
  
      default:
        //This should NEVER happen!
        Serial.println("[ERROR] Invalid Flight Mode");
      break;
  
  
      
    }
  }
  
}

/**LORA METHODS**/
void loraSetup()
{
  Program_Setup();

  Serial.println("[LOG] Starting LoRa setup");
  lora_ResetDev();      //Reset the device
  lora_Setup();       //Do the initial LoRa Setup
  lora_SetFreq(434.400);    //Set the LoRa
  //lora_Print();             //Print the LoRa registers
  //lora_Tone(1000, 1000, 5);             //Transmit an FM tone
  lora_SetModem(lora_BW41_7, lora_SF8, lora_CR4_5, lora_Explicit, lora_LowDoptOFF);   //Setup the LoRa modem parameters
  //lora_PrintModem();                    //Print the modem parameters
  //Serial.println();
  //lora_FillTX();      //Fill TXbuff with test data
  //lora_TXBuffPrint(0);      //Print the TX buffer as ASCII to screen

  //Send LoRa setup complete message
  sendLora("[LOG] LoRa Radio System Initialized");

  Serial.println("[LOG] LoRa setup complete");
}

void sendLora(String message)
{

    //byte TXTemp[256];
    message.getBytes(lora_TXBUFF, 256);
    lora_TXEnd = message.length() * sizeof(char);  // so a single byte packet ends up with lora_TXend of 0
    lora_TXStart = 0;

    /*for(int i = 0; i < sizeof(message) ; i++)

    {
      lora_TXEnd++;
      lora_TXBUFF[lora_TXEnd] = TXTemp[i];
    }*/
    Serial.print(lora_TXStart);
    Serial.print(" ");
    Serial.print(lora_TXEnd);
    Serial.println();
    //lora_TXBuffPrint(0);
    lora_Send(lora_TXStart, lora_TXEnd, 32, 255, 1, 10, 17);
}

String receiveLora(){
    lora_Listen(0);
    byte lora_LData;
    byte lcount;
    byte lora_LLoop;
    
    if(lora_FRXOK == 1){
      for(lora_LLoop = lora_RXStart; lora_LLoop <= lora_RXEnd; lora_LLoop++)
      {
        lcount++;
        lora_LData = lora_RXBUFF[lora_LLoop];
      }

      String message = String((char *)lora_LData);
      return message; 
    }
}
