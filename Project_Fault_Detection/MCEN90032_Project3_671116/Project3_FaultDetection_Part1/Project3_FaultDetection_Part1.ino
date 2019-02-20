//Code written by Akshay Kumar, 671116, for MCEN90032 Sensor Systems Project 3, Part 1
//Last editted 21st October 2018

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bmp; // I2C


// Accelerometers variables
int analogPinZ1 = A0;// 1st Acceleromter Pin
int analogPinZ2 = A1;// 2nd Acceleromter Pin
double value_z1 = 0;                
double value_z2 = 0;                 
double G_z1     = 0;                 
double G_z2     = 0;   
double normalised_acc1 = 0;
double sensorValuez1 = 0; 
double normalised_acc2 = 0;
double sensorValuez2 = 0; 
int G= 1;
int z1_min = 271;             
int z1_max = 402;             
int z2_min = 268;             
int z2_max = 400;             


// Pressure Sensor Variables
double height_now;
double save;
double pressurenow;
double relative_height;
int counter=0;
double norm_acc1;
double norm_acc2;
double save_acc1=0;
double save_acc2=0;


// Setting up timer variables
double T = 0.01;
double Current_Time;
double Prev_Time = 0;
double Time_Interval = T;


            
// Other variables
int fix_error1 = 0;
int fix_error2 = 0;
void setup() {

  Serial.begin(9600);
  

  if (!bmp.begin()) {  
    Serial.println(("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
}

void loop() {

  Current_Time = millis();
  if (Current_Time - Prev_Time >= Time_Interval)
  {
    Prev_Time = Current_Time;
    // Accelerometers collect data
    value_z1 = analogRead(analogPinZ1);
    value_z2 = analogRead(analogPinZ2);
  
    // Calibration of Accelerometer from Project 1 (Function modified for robustness)
    sensorValuez1 = map2_float_v3(value_z1, z1_min, z1_max, -G, G);
    sensorValuez2 = map2_float_v3(value_z2, z2_min, z1_max, -G, G);
    // Normalisation of acceleration data
    normalised_acc1 = sensorValuez1 - G;
    normalised_acc2 = sensorValuez2 - G;

    // Fault detected algorithm for when Accelerometer 1 has its input pin removed
    if (normalised_acc1-save_acc1 > 1)
    {
      Serial.print("Warning... A fault has been detected in Accelerometer 1. Fix the error");
      Serial.print("\n");
      fix_error1 = 1;
      while(1){}
    }
    // Fault detected algorithm for when Accelerometer 1 has its power pin removed
    else if(abs(normalised_acc1)>  4){
      Serial.print("Warning... Power loss in Accelerometer 1");
      Serial.print("\n");
      while(1){}
      
    }
    // Prints normally if no fault detected
    else{
      Serial.print(normalised_acc1);
      Serial.print("\t");
    }

    // Fault detected algorithm for when Accelerometer 2 has its input pin removed
    if (normalised_acc2-save_acc2 > 1)
    {
      Serial.print("Warning... A fault has been detected in Accelerometer 2. Fix the error");
      Serial.print("\n");
      while(1){}
      
    }
    // Fault detected algorithm for when Accelerometer 2 has its power pin removed
    else if (normalised_acc2 < -6){
      Serial.print("Warning... Power loss in Accelerometer 2");
      Serial.print("\n");
      while(1){}
      
    }
    // Prints normally if no fault detected
    else{
      Serial.print(normalised_acc2);
      Serial.print("\t");
    }
    
    // Saves the very last accelerations temporarily for fault checking
    save_acc1 = normalised_acc1;
    save_acc2 = normalised_acc2;
    // Pressure sensor readings
    bmp.readTemperature();
    pressurenow = bmp.readPressure()/100;
    height_now = bmp.readAltitude(1022.4);
    // Establishing a baseline for the pressure sensor to measure from
    if (counter == 0){
      save = height_now;
      counter = 1;
    }
    // Determining the relative height from an initial baseline
    relative_height = height_now - save;
    // Fault detection algorithm for when Pressure sensor has its input pin removed
    if (pressurenow < 0 ){
      Serial.print("Problem with Pressure sensor. Please fix");
      Serial.print("\n");
    }
    else {
      Serial.print(bmp.readPressure()/100);
    Serial.print("\t");
    }
    Serial.println();

  }
}

// Calibration function as per Project 1 (Modified for improved robustness)
double map2_float_v3(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


