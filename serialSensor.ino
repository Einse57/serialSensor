// Example Arduino101 based sensor system with logging connection handshake and serial messaging
// The purpose of this program is to take and offboard readings at high rate to a serial receiver

#include <BMI160.h>
#include <CurieImu.h>
#include <Wire.h>

// Pin Definitions
// Arduino ADCs consist of a 6 into 1 multiplexer with a single sample and hold circuit in the middle and one 10 bit ADC
// Make sure all lines are pulled either up or down
// If lines are left floating, all ADC readings will follow a connected ADC line since it will be the only thing charging or 
// discharging the sample and hold circuitry
int adcLine3 = A3;   
int adcLine2  = A2;  
int adcLine1  = A1;   
int adcLine0  = A0;   

// Store sensor readings in a central location
// [adcLine0MSB, adcLine0LSB, adcLine1MSB, adcLine1LSB, ...]
byte adcValues[13];

// Store IMU readings in a central location
byte imuValues[23];

// Read back timing info
byte timing[8];

// Allocate a temp value to hold ADC reading as it is separated into bytes
unsigned int tempvalue = 0;

// IMU variables
int accel[3];       // Multiply by 0.00390625 to obtain g reading (3.9mg/LSb conversion factor - datasheet page 17)
int magnetom[3];    // Multiply by 0.92 mGauss/LSb conversion (datasheet pg 13)
int gyro[3];        // Divide by 14.375 deg/sec to obtain deg/sec reading

// Timing variables
unsigned long curr_time = 0;

// Timing variables used for adc rate locked loop
unsigned long adc_interval = 15625;  // 64 Hz, 1/64 = 0.015625 seconds = 15625 microseconds  (sent by reader program)
unsigned long adc_read = 0;

// Timing variables for imu rate locked loop
unsigned long imu_interval = 20000;  // 50 Hz, 1/50 = 0.02 seconds = 20000 microseconds  
unsigned long imu_read = 0;

// Used only for receiving rate parameter
unsigned long temp_rate = 0;

void setup() 
{ 
  // Start serial
  Serial.begin(115200);

  // Start the Curie's IMU (Accel/Gyro only)
  CurieImu.initialize();
  
  // Set analog ref voltage to AREF external source (may jumper to Uno 3.3V supply)
  //analogReference(EXTERNAL);
  
  // Set ADC lines as inputs, assume there are 10k pull down resistors on all lines of interest
  pinMode(adcLine3,  INPUT); 
  pinMode(adcLine2,  INPUT);
  pinMode(adcLine1,  INPUT);
  pinMode(adcLine0,  INPUT);
  
  // TEST CODE: 
  // Used for timing validation - init and set to known value
  // Note that a digitalWrite may take from 5-17us to complete
  //pinMode(4, OUTPUT);
  //digitalWrite(4, LOW);
  
  // Init IMU sensors
  delay(50);  // Give sensors enough time to start

  // Send handshake byte and wait for connection from the serial receiver
  while (1)
  {
    Serial.write(0xAA);
    delay(300);
    // Receive handshake byte and make sure it's valid
    if ((Serial.available() == 9) && (Serial.read() == 0xBB))
    {
      // Next we expect resmed rate information (4 bytes) to be sent     
      // Had to break up operations because Arduino was failing on shifting + or'ing at the same line
      temp_rate = Serial.read();
      adc_interval = temp_rate << 24;
      
      temp_rate = Serial.read();
      temp_rate = temp_rate << 16;
      adc_interval = adc_interval | temp_rate;
      
      temp_rate = Serial.read();
      temp_rate = temp_rate << 8;
      adc_interval = adc_interval | temp_rate;
      
      temp_rate = Serial.read();
      adc_interval = adc_interval | temp_rate;
      
      // Read back received rate
      //Serial.print("ResMed Interval: ");
      //Serial.println(adc_interval, DEC);
      
      // Next we expect imu rate information (4 bytes) to be sent     
      temp_rate = Serial.read();
      imu_interval = temp_rate << 24;
      
      temp_rate = Serial.read();
      temp_rate = temp_rate << 16;
      imu_interval = imu_interval | temp_rate;
      
      temp_rate = Serial.read();
      temp_rate = temp_rate << 8;
      imu_interval = imu_interval | temp_rate;
      
      temp_rate = Serial.read();
      imu_interval = imu_interval | temp_rate;
   
      // Read back received rate
      //Serial.print("IMU Interval: ");
      //Serial.println(imu_interval, DEC);
            
      // Read back received rates
      timing[7] = adc_interval >> 24;
      timing[6] = adc_interval >> 16;
      timing[5] = adc_interval >> 8;
      timing[4] = adc_interval;
      
      timing[3] = imu_interval >> 24;
      timing[2] = imu_interval >> 16;
      timing[1] = imu_interval >> 8;
      timing[0] = imu_interval;
      
      Serial.write(timing, 8);
      
      break;
    }
  }
}

void loop() 
{

// Get current system time in microseconds
curr_time = micros();

/*
* Sensor read and transmit rate locked loop
* Handles timing rollovers due to micros (rollover after 71 minutes)
* Implements abs(current time - last reading) without complication

* This loop was measured using an oscilloscope and toggling digital pin 
* Using this method the loop took 680 us to complete
*/

if ((unsigned long)((long)curr_time - (long)adc_read) >= adc_interval)
{

// TEST CODE: 
// Raise the start of work signal pin
//digitalWrite(4, HIGH);

// read the sensor
tempvalue = analogRead(adcLine3);
adcValues[12] = tempvalue >> 8;
adcValues[11] = tempvalue;

tempvalue = analogRead(adcLine2);
adcValues[10] = tempvalue  >> 8;
adcValues[9] = tempvalue;

tempvalue = analogRead(adcLine1);
adcValues[8] = tempvalue >> 8;
adcValues[7] = tempvalue;

tempvalue = analogRead(adcLine0);
adcValues[6] = tempvalue >> 8;
adcValues[5] = tempvalue;

// send timestamp
adcValues[4] = curr_time >> 24;
adcValues[3] = curr_time >> 16;
adcValues[2] = curr_time >> 8;
adcValues[1] = curr_time;

// send Msg ID for sensor values
adcValues[0] = 0xCC;

// write results to serial
Serial.write(adcValues, 13);

// update timekeeping
adc_read = curr_time;

// TEST CODE: 
// Lower the end of work signal pin
//digitalWrite(4, LOW);
}

// read the imu
if ((unsigned long)((long)curr_time - (long)imu_read) >= imu_interval)
{

  gyro[2] = CurieImu.getRotationZ();
  gyro[1] = CurieImu.getRotationY();
  gyro[0] = CurieImu.getRotationX();

  accel[2] = CurieImu.getAccelerationZ();
  accel[1] = CurieImu.getAccelerationY();
  accel[0] = CurieImu.getAccelerationX();
    
  // Gyro_z
  imuValues[22] = gyro[2] >> 8;
  imuValues[21] = gyro[2];
  // Gyro_y
  imuValues[20] = gyro[1] >> 8;
  imuValues[19] = gyro[1];
  // Gyro_x
  imuValues[18] = gyro[0] >> 8;
  imuValues[17] = gyro[0];
 
  // Mag_z
  imuValues[16] = 0;
  imuValues[15] = 0;
  // Mag_y
  imuValues[14] = 0;
  imuValues[13] = 0;
  // Mag_x
  imuValues[12] = 0;
  imuValues[11] = 0;

  // Acc_z
  imuValues[10] = accel[2] >> 8;
  imuValues[9] = accel[2];
  // Acc_y
  imuValues[8] = accel[1] >> 8;
  imuValues[7] = accel[1];
  // Acc_x
  imuValues[6] = accel[0] >> 8;
  imuValues[5] = accel[0];

  // send timestamp
  imuValues[4] = curr_time >> 24;
  imuValues[3] = curr_time >> 16;
  imuValues[2] = curr_time >> 8;
  imuValues[1] = curr_time;

  // send Msg ID for IMU values
  imuValues[0] = 0xDD;
  
  // write data
  Serial.write(imuValues, 23);
  
  // update timekeeping
  imu_read = curr_time; 
}

// end of main()
} 

