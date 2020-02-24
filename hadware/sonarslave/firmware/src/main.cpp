#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HCSR04.h>
#include <cmath>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS PB10

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// arrays to hold device address
DeviceAddress insideThermometer;

TwoWire i2c(PB7,PB6);

#define s_cnt 3
#define buff_cnt 5
#define LED PC13

UltraSonicDistanceSensor sonars[s_cnt] = {
  UltraSonicDistanceSensor(PB13,PB12),
  UltraSonicDistanceSensor(PB14,PA11),
  UltraSonicDistanceSensor(PB15,PA12)
};



uint64_t counter = 0;
uint32_t values_mm[s_cnt];
uint32_t stdev_mm[s_cnt];
uint32_t sonar_buffer_counter[s_cnt];
uint32_t sonar_buffer[s_cnt][buff_cnt];
uint32_t temp_deviation[buff_cnt];
uint8_t cmd = 1;
float temp;
bool led_on = false;

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void onRecive(int howMany) {
  if (howMany > 0) {
  uint8_t * data = (uint8_t*)malloc(howMany);
  i2c.readBytes(data,howMany);
  cmd = data[0];
  free(data);
  }
  Serial.println("CMD RECIVED");
}

void onRequest(void) {
  switch (cmd)
  {
  case 1:
      i2c.write((const char*) values_mm, sizeof(uint32_t)*3);
    break;
  case 2:
      i2c.write((const char*) stdev_mm, sizeof(uint32_t)*3);
  default:
    i2c.write((const char*) values_mm, sizeof(uint32_t)*3);
    break;
  }
  Serial.println("RESPONSE SEND");
}

float getTemperature(){
  sensors.requestTemperatures();

  Serial.print(values_mm[0]);
  Serial.print(" ");
  Serial.print(values_mm[1]);
  Serial.print(" ");
  Serial.println(values_mm[2]);

  float tempC = sensors.getTempC(insideThermometer);
  Serial.print("Temp C: ");
  Serial.println(tempC);
  return tempC;
}

void setup() {
   //put your setup code here, to run once:
  Serial.begin(9600);
  
  i2c.begin(21);
  i2c.onRequest(&onRequest);
  i2c.onReceive(&onRecive);

  pinMode(LED, OUTPUT);

  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();
  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

  //reset all buffers
  for (size_t i = 0; i < s_cnt; i++)
  {
    sonar_buffer_counter[i] = 0;
    values_mm[i] = 0;
    for (size_t j = 0; j < buff_cnt; j++)
    {
      sonar_buffer[i][j]=0;
    }
    
  }
  
  temp = getTemperature();
}

void loop() {
  if (counter % 500 == 0 ) {
    temp = getTemperature();
  }  
  int current = counter % 3;

  uint32_t value_um = sonars[current].measureDistanceCm(temp) * 10000;
  

  if (value_um > 1) {
    //insert new value into buffer
    uint32_t buffer_slot = sonar_buffer_counter[current];
    uint32_t  * buffer = sonar_buffer[current];
    buffer[buffer_slot] = value_um;
    sonar_buffer_counter[current] = (sonar_buffer_counter[current]+1) % buff_cnt;

    uint64_t sum = 0;
    for (size_t i = 0; i < buff_cnt; i++) {sum += buffer[i];}
    uint32_t mean = sum / buff_cnt;

    uint64_t vari_sum = 0;
    uint64_t max_vari = 0;
    uint64_t max_idx = 0;
    for (size_t i = 0; i < buff_cnt; i++)
    { 
      uint64_t var = sq(mean -  buffer[i]);
      if (var > max_vari) {
        max_vari = var;
        max_idx = i;
      }
      vari_sum += var; 
    }
    uint64_t vari = vari_sum /buff_cnt;

    if(vari > sq(20000)) {
      sum -= buffer[max_idx];
      mean = sum / (buff_cnt-1);
      vari_sum = 0;
      for (size_t i = 0; i < buff_cnt; i++)
      { 
        if(i == max_idx) {continue;}
        vari_sum += sq(mean -  buffer[i]);
      }
      vari = vari_sum / (buff_cnt-1);
    }
    values_mm[current] =  mean;
    stdev_mm[current] = sqrt(vari);

  } else {
    // no valid echo
  }

    Serial.print(values_mm[0]/1000);
    Serial.print(" \t ");
    Serial.print(values_mm[1]/1000);
    Serial.print(" \t ");
    Serial.println(values_mm[2]/1000);

  
  if (led_on) {
    digitalWrite(LED,LOW);
    led_on = false;
  } else {
    digitalWrite(LED,HIGH);
    led_on = true;
  }

  counter++;
}

