//Code written by Akshay Kumar, 671116, for MCEN90032 Sensor Systems Project 3, Part 3i
//Last editted 21st October 2018

#include <Matrix.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bmp;               

// Accelerometers variables
double total_acc;
int analogPinZ1 = A0;// 1st Acceleromter Pin
int analogPinZ2 = A1;// 2nd Acceleromter Pin
double value_z1 = 0;                 
double value_z2 = 0;                   
double sensorValuez1 = 0;                 
double sensorValuez2 = 0;                 
double normalised_acc1 = 0;
double normalised_acc2 = 0;
int z1_min = 271;             
int z1_max = 402;             
int z2_min = 268;             
int z2_max = 400; 
int G = 1;   
double save_acc1=0;
double save_acc2=0;

// Pressure sensor variables
double height_now;
double save;
int counter = 0;
double relative_height;
double pressure = 0;

// Setting up timer variables
double Current_Time;
double Prev_Time = 0;
double T = 0.01;
double Time_Interval = T;


// Other variables used            
double dist;
double factors = 0.333;

void setup()
{
  Serial.begin(9600);
  if (!bmp.begin()) {  
  Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  while (1);
  }
}

void loop()
{ 
  Current_Time = millis();
  if (Current_Time - Prev_Time >= Time_Interval)
  {
    Prev_Time = Current_Time;
    
   // Accelerometer collects data
    value_z1 = analogRead(analogPinZ1);
    value_z2 = analogRead(analogPinZ2);

    // Calibration of Accelerometer from Project 1 (Function modified for robustness)
    sensorValuez1 = map2_double_v3(value_z1, z1_min, z1_max, -G, G);
    sensorValuez2 = map2_double_v3(value_z2, z2_min, z2_max, -G, G);
    // Normalisation of acceleration data
    normalised_acc1 = sensorValuez1 - G;
    normalised_acc2 = sensorValuez2 - G;

     // Fault detected algorithm for when Accelerometer 1 has its input pin removed
    if (normalised_acc1-save_acc1 > 1){
      Serial.print("Warning... A fault has been detected in Accelerometer 1. Fix the error");
      Serial.print("\n");
    }
    // Fault detected algorithm for when Accelerometer 2 has its input pin removed
    if (normalised_acc2-save_acc2 > 1) {
      Serial.print("Warning... A fault has been detected in Accelerometer 2. Fix the error");
      Serial.print("\n");
      
    }
    // The data fusion between the 2 accelerometer accelerations is taken to be the average between them
    total_acc = (normalised_acc1 + normalised_acc2)/2.0;
    
    
   // Pressure sensor readings
    bmp.readTemperature();
    pressure = bmp.readPressure()/100;
    height_now = bmp.readAltitude(1027);
    if (counter == 0){
      save = height_now;
      counter = 1;
    }
    relative_height = height_now - save;
    // Fault detection algorithm for when Pressure sensor has its input pin removed
    if (pressure < 0)
    {
      Serial.print("Pressure sensor failed");
      Serial.println();
      while(1){}
    }

    // The proposed height estimation algorithm 
    total_acc = factors*normalised_acc1 + factors*normalised_acc2;
    dist =total_acc*T*T/2 + relative_height*factors;
    
    // Plotting all the data
    Serial.print(total_acc);    // Acceleration
    Serial.print("\t");
    Serial.print(relative_height);    //Pressure sensor height
    Serial.print("\t");

    // Display the position output
    Serial.print(dist);   //Estimation of height 
    Serial.print("\n");
  }
}

// Calibration function as per Project 1 (Modified for improved robustness)
double map2_double_v3(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

