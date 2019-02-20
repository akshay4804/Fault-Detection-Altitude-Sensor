//Code written by Akshay Kumar, 671116, for MCEN90032 Sensor Systems Project 3, Part 3iii
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
double save_acc1=0;
double save_acc2=0;

int z1_min = 271;             
int z1_max = 402;             
int z2_min = 268;             
int z2_max = 400; 
int G = 1;        

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

// Kalman filter variables
double var_pressure = 1;
double beta = 100;
double beta_v = -sqrt(2/beta)*var_pressure*(exp(-beta*T)-1);

// Initialise matrix arrays
double array1[3][3] = {{1, T, 0}, {0, 1, 0}, {0, 0, exp(-beta*T)}};
double array2[3][1] = {{T*T/2.0}, {T}, {0}};
double array3[1][3] = {1, 0, 1};
double array4[3][1] = {{0}, {0}, {0}};
double array5[3][3] = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
double array6[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
double array7[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, beta_v}};
double array8[3][1] = {{var_pressure}, {var_pressure}, {var_pressure}};

Matrix<double> A(3,3, (double*)array1);
Matrix<double> B(3,1, (double*)array2);
Matrix<double> C(1,3, (double*)array3);
Matrix<double> x_k(3,1,(double*)array4);
Matrix<double> P(3,3, (double*)array5);
Matrix<double> noise_1(3,3, (double*)array7);
Matrix<double> noise_2(3,1, (double*)array8);
Matrix<double> I(3,3, (double*)array6);
Matrix<double> X(3,1); 
Matrix<double> x_bar(3,1); 
Matrix<double> K(3,3); 
Matrix<double> P_bar(3,3);
Matrix<double> Q = (B*(Matrix<double>::transpose(B)))*((noise_1*noise_2)*(Matrix<double>::transpose((noise_1*noise_2))));
double R = var_pressure*var_pressure;

void setup()
{
  Serial.begin(9600);
  if (!bmp.begin()) {  
  Serial.println(("Could not find a valid BMP280 sensor, check wiring!"));
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
    
    // Apply the calibration to the sensor reading and then normalise the data:
    sensorValuez1 = map2_double_v3(value_z1, z1_min, z1_max, -G, G);
    sensorValuez2 = map2_double_v3(value_z2, z2_min, z2_max, -G, G);
    // Normalise the pressure sensor readings
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
    
    // Time Update (prediction)
    x_bar = A*x_k + B*total_acc;    // Project the state ahead of time
    P_bar = A*P*(Matrix<double>::transpose(A)) + Q;   // Project the error covariance ahead of time
  
    // Measurement Update (correction)
    K = P_bar*(Matrix<double>::transpose(C))*(Matrix<double>::inv((C*P_bar*Matrix<double>::transpose(C) + R))); // Kalman Gain calculation
    x_k = x_bar + K*(relative_height - C*x_bar);  // Update the estimation of the position and velocity of the arduino system
    P = (I - K*C)*P_bar;  // Updating the error covariance

    // Plotting all the data
    Serial.print(total_acc);    // Acceleration
    Serial.print("\t");
    Serial.print(relative_height);    //Pressure sensor height
    Serial.print("\t");

    // Display the position output
    Serial.print(x_k._entity[0][0]);    // The Kalman filter position
    Serial.print("\n");

  }
}
// Calibration function as per Project 1 (Modified for improved robustness)
double map2_double_v3(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

