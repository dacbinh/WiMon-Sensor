
#include <SPI.h>
#include <LiquidCrystal_I2C.h>       //lcd  libraries
#include <Wire.h>
#include "MAX30105.h"           //MAX3010x library //Heart rate  calculating algorithm
#include <RF24.h>        // radio frequency device lib
#include <ESP8266WiFi.h>
#include "spo2_algorithm.h"
#include <string>
#include <algorithm>
#include <stdlib.h>

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];
#else
uint32_t irBuffer[100];
uint32_t redBuffer[100];
#endif
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
uint64_t this_device_mac;
uint8_t address[][6] = {"1Node", "2Node"} ;
uint8_t channel; //default channel

#define THIS_DEVICE_ROLE 1 // 1 for transmiter 0 for reciever
#define WIMON_PAYLOAD_SIZE 22

bool role = THIS_DEVICE_ROLE;

// //for i2c protocol
#define SCL_PIN 20
#define SDA_PIN 19

//for SPI protocol
#define SCLK_PIN 5
#define MISO_PIN 6
#define MOSI_PIN 7
#define CS_PIN 17
#define CSN_PIN 16
//for lcd screen
#define SCREEN_ADDRESS 0x27
#define SCREEN_WIDTH  16 
#define SCREEN_HEIGHT 2 
// //others
#define MAX_BRIGHTNESS 255
//set pin # (or -1 if sharing Arduino  reset pin)
struct WiMonPacket {
	uint8_t channel; // The channel can be changed from 0-255 to use with different WiMon terminals
	uint16_t temp_C; // temperature is stored as a fixed point real number with 3 decimal places (ie: 35000 translates to 35.000*C)
	uint8_t spo2; // SPO2 is stored as a fixed point real number with 1 decimal place when the oxygen saturation is below 100%, whether saturation is 100% is determined by the followed bit
	bool is_spo2_valid;
  bool is_hr_valid;
	uint16_t hr;// the heartrate is stored as a fixed point real number with 2 decimal place with range 0-655.36BPM  (15002 translates to 150.02BPM)
	uint64_t device_MAC; // This value is generated on startup for handshaking purposes with the terminal and ensuring data integrity
};

struct Handshake{
  bool hanshake_enable;
  uint64_t mac_addr;
};
LiquidCrystal_I2C  lcd(SCREEN_ADDRESS,SCREEN_WIDTH, SCREEN_HEIGHT) ; //Declaring the display name (lcd)
MAX30105 particleSensor;
RF24 radio(CSN_PIN, CS_PIN);


void setup()
{
  radio.begin();
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  Wire.begin();
  
  // // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    // while (1);
  }


  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 
  //Configure sensor with these settings
      char temp[18];  
      WiFi.macAddress().toCharArray(temp,sizeof(temp));
      this_device_mac = convert_mac(temp);
}

void loop()
{
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
      /*
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
    */
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample


    }
      // END NOTE*
    //assigning values to prepare for transmitting
       // convert ESP "lite" String into char array 
      //this version transmit and recieve with ackpayload
      radio.setPALevel(RF24_PA_HIGH);
      radio.enableAckPayload();
      radio.enableDynamicAck();
      radio.openReadingPipe(1,address[channel]);
      radio.openWritingPipe(address[channel]);
      //radio for sensor
      radio.startListening();
      delay(500);
      uint8_t pipe_num;
          if(radio.available(&pipe_num) && radio.getDynamicPayloadSize() < WIMON_PAYLOAD_SIZE){
            Handshake pairing;
            radio.read(&pairing, sizeof(pairing));
            if(pairing.hanshake_enable && pairing.mac_addr == this_device_mac){
              radio.stopListening();
          WiMonPacket send;
          send.device_MAC = this_device_mac;
          send.channel = channel;
          send.temp_C = cast_temp(particleSensor.readTemperature());
          send.spo2 = cast_spo(spo2);
          send.is_spo2_valid = (bool)validSPO2;
          send.hr = heartRate;
          send.is_hr_valid = (bool)validHeartRate;
          uint8_t serialize[&send, sizeof(send)]
          radio.write(&send, sizeof(send));
            }           
          }
    
    //END THE RF24 TX-RX PART



    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  
  }

}
///functions

//start transmit and stop recieving, clear any ack that is left in FIFO
//warning: THIS CAN ONLY BE USED IF radio.begin() was success


//cast temperature (C) in to format required by terminal in struct (ex: 37000 for 37.000 C)
uint16_t cast_temp(float temp){
  return (uint16_t)(temp * 1000);
}
//cast spo2 into format required by terminal in struct (ex: 8600 for 86.00%)
uint8_t cast_spo(int32_t spo){
  return (uint8_t)(spo *100);
}
//cast heart rate into format required by terminal in struct (ex: 602 for 60.2 bpm)
uint16_t cast_hr(int32_t hr){
  return (uint16_t)(hr * 10);
}
//hanshaking protocol

//this function use for terminal to send request



uint64_t convert_mac(std::string mac) {
  // Remove colons
  mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());

  // Convert to uint64_t
  return strtoul(mac.c_str(), NULL, 16);
}


