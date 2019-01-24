/* Code for getting measurement values from 
* Two laser range sensors 
* MPU9250 including magnetometer for the direction and gyroscope 
* By: Loc Nguyen
* Original Source: https://github.com/stevenvo/mpuarduino/blob/master/mpuarduino.ino
*                  "vl53l0x_dual_sensor_test" from class material on CCLE
 * Description: Basic code to test the functionality of the
 * MPU9250 and range sensors. Refer to the startup guide on CCLE for 
 * more information. NOTE: this is meant to help confirm 
 * communication with the sensor, calibration is still required
 */
#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PORT 14
#define SCL_PORT 12
VL53L0X sensor;
VL53L0X sensor2;

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define SDA_PORT 14
#define SCL_PORT 12


float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float MPU9250gyroBias[3] = {0, 0, 0}, MPU9250accelBias[3] = {0, 0, 0}, MPU9250magBias[3] = {0, 0, 0};

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();

    
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin(SDA_PORT,SCL_PORT);
  Serial.begin(115200);

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);

  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);
  Wire.begin(SDA_PORT,SCL_PORT);

  Serial.begin (115200);

  digitalWrite(D3, HIGH);
  delay(150);
  Serial.println("00");
  
  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");
  
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  delay(3000);

}

long int cpt=0;
// Main loop, read and display data
void loop()
{
  
  // _______________
  // ::: Counter :::
  
  // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print ("\t");
  
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Create 16 bits values from 8 bits data
  
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

  
  // :::  Magnetometer and Gyroscope ::: 

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=(Mag[3]<<8 | Mag[2]);
  int16_t my=(Mag[1]<<8 | Mag[0]);
  int16_t mz=(Mag[5]<<8 | Mag[4]);
  
  // Shift mx and my to origin
  mx = mx - 76;
  my = my  + 122;
  
  float heading = atan2(my, mx);

  // Once you have your heading, you must then add your 'Declination Angle',
  // which is the 'Error' of the magnetic field in your location. Mine is 0.0404 
  // Find yours here: http://www.magnetic-declination.com/
  
  // From above website, our declination at LA is 0.208218.
  float declinationAngle = 0.208218;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
     heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/PI; 
  
  Serial.print ("Magnetometer readings:"); 
  Serial.print ("\tMx:");
  Serial.print (mx); 
  Serial.print ("\tMy:");
  Serial.print (my);
  Serial.print ("\tMz:");
  Serial.print (mz);  
  Serial.println ("\t\n");

  Serial.print("\rHeading:\t");
  Serial.print(heading);
  Serial.print(" Radians   \t");
  Serial.print(headingDegrees);
  Serial.println(" Degrees   \n");

  Serial.print ("Gyrosocpe readings:"); 
  Serial.print ("\tGx:");
  Serial.print (gx); 
  Serial.print ("\tGy:");
  Serial.print (gy);
  Serial.print ("\tGz:");
  Serial.print (gz);  
  Serial.println ("\t\n");



  Serial.print("Lidar 1 range(mm): ");
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.print("  Lidar 2 range(mm): ");
  Serial.println(sensor2.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }

  // End of line
  delay(500); 


  
}
