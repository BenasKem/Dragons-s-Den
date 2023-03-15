#include <Arduino.h>
#include "HIH8000_I2C.h"

void fetchMeas();
bool trigMeas();



HIH8000_I2C hihSensor = HIH8000_I2C(0x27);

HIH8000_I2C sensors[1];

bool getReading = true;
bool trigSuccess = false;
byte I2CStatus = 0;
char serialReadBuffer[2];

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(100000);
  Serial.begin(9600);
  trigSuccess = trigMeas();
}

void loop() {
  if (getReading)
  {
    if (trigSuccess) {
      fetchMeas();
    }
    
    trigSuccess = trigMeas();
    
    delay(5000);
  }
}

bool trigMeas() {
  I2CStatus = hihSensor.triggerMeasurement();
  
  switch (I2CStatus)
  {
    case HIH8000_I2C_comStatus_OK:
      return true;
      break;
      
    case HIH8000_I2C_comStatus_LONGDATA:
       Serial.println("Data too long to fit in I2C transmit buffer");
      break;

    case HIH8000_I2C_comStatus_NACKADD:
      Serial.println("Cannot find a sensor with the given address");
      break;
      
    case HIH8000_I2C_comStatus_NACKDATA:
      Serial.println("Sensor stopped data transmission");
      break;
      break;

    case HIH8000_I2C_comStatus_ADDRESS:
      Serial.println("Address of sensor has not been set in the HIH8000_I2C class");
      break;

    case HIH8000_I2C_comStatus_OTHER:
    default:
      Serial.println("Something went wrong...");
      break;
  }

  return false;
}

void fetchMeas() {
  I2CStatus = hihSensor.fetchMeasurement();

  if (I2CStatus == HIH8000_I2C_comStatus_OK) {
    switch (hihSensor.getStatus())
    {
      case 0:
        Serial.println("Relative humidity: " + String(hihSensor.getHumidity(), 2) + "%");
        Serial.println("Temperature: " + String(hihSensor.getTemperature(), 2) + " C");
        break;

      case 1:
        Serial.println("***NOTE: Stale data***");
        Serial.println("Relative humidity: " + String(hihSensor.getHumidity(), 2) + "%");
        Serial.println("Temperature: " + String(hihSensor.getTemperature(), 2) + " C");
        break;
        
      case 2:
        Serial.println("Sensor in command mode");
        break;

      case 3:
        Serial.println("Sensor in diagnostic mode");
        break;

      default:
        Serial.println("Sensor returned an invalid status");
        break;
    }
  } else {
    switch (I2CStatus)
    {
      case HIH8000_I2C_comStatus_BYTECOUNT:
        Serial.println("Received different amount of bytes than requested");
        break;

      case HIH8000_I2C_comStatus_ADDRESS:
        Serial.println("Address of sensor has not been set in the HIH8000_I2C class");
        break;
        
      default:
        Serial.println("Something went wrong...");
        break;
    }
  }
}

void serialEvent()
{
  Serial.readBytes(serialReadBuffer, 1);

  if (serialReadBuffer[0] == 'g')
  {
    getReading = true;
  }
  else if (serialReadBuffer[0] == 's')
  {
    getReading = false;
  }
}

void ScanSensors()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}