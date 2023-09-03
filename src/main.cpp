#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>
#include <MPU6050_light.h>
#include <Servo.h>

const float AltitudeUncertainty = 1;
const int8_t Servo1Offset = 15, Servo2Offset = 5, Servo3Offset = 5, Servo4Offset = -2;

Adafruit_BMP280 BMPSensor;
File FileHandler;
MPU6050 Gyroscrope(Wire);
Servo Servo1; //.
Servo Servo2; //.
Servo Servo3; //.
Servo Servo4; //.. control servos
Servo Servo5; //recovery system servo
String Data = "";
uint64_t timer = 0;

float AngleRoll = 0, AnglePitch = 0, AngleYaw = 0;

float StartingAltitude = -1000;
float Altitude = 0;
float MaxAltitude = 0;
float DeltaAltitude = 0;
float DeltaMaxAltitude = 0;
float Temperature = 0;
float Pressure = 0;
bool FilePresent = true;
bool Launched = false;
uint64_t MainStartTime = 0;

void SetupSensors()
{
  //sets up i2c coms
  Wire.setClock(400000);

  //sets up gyro
  Gyroscrope.begin();
  delay(1000);
  Gyroscrope.calcOffsets();
  delay(1000);

  //Sets of bmp sensor and relavent variables
  BMPSensor.begin();
  StartingAltitude = BMPSensor.readAltitude();

  //sets up serial coms
  Serial.begin(57600);
  Serial.print("Starting Altitude/m: "); Serial.println(StartingAltitude);
  delay(1000);

  //sets up file for writing too
  pinMode(10, OUTPUT);
  Serial.print("Initializing SD card...");

  if (SD.begin(10)) 
  {
    Serial.println("initialization success!");
  }
  else
  {
    Serial.println("initialization failed!");
  }

  FileHandler = SD.open("DATA.txt", FILE_WRITE | O_TRUNC);
  if (FileHandler)
  {
    FileHandler.print("Starting Altitude/m: "); FileHandler.println(StartingAltitude);
    Serial.println("Starting alt written to file");
  }
  else
  {
    FilePresent = false;
    Serial.println("Can't write to file");
  }
  FileHandler.close();
}

void SetupServos()
{
  //sets up the servos
  Servo1.attach(8);
  Servo2.attach(7);
  Servo3.attach(6);
  Servo4.attach(5);
  Servo5.attach(9); 
  Servo5.write(0);
  delay(1000);

  // quickly tests each servo to make sure visibly that their power source is sufficient
  Servo1.write(0 - Servo1Offset);
  Servo2.write(0 + Servo2Offset);
  Servo3.write(0 - Servo3Offset);
  Servo4.write(0 + Servo4Offset);
  delay(250);
  Servo1.write(180 - Servo1Offset);
  Servo2.write(180 + Servo2Offset);
  Servo3.write(180 - Servo3Offset);
  Servo4.write(180 + Servo4Offset);
  delay(250);
  Servo1.write(90 - Servo1Offset);
  Servo2.write(90 + Servo2Offset);
  Servo3.write(90 - Servo3Offset);
  Servo4.write(90 + Servo4Offset);
  delay(250);
}

void setup() 
{
  SetupSensors();
  SetupServos();

  MainStartTime = millis(); 
}

void RetrieveGyroData()
{
  //updates gyro and gets angles
  Gyroscrope.update();
  AngleRoll = Gyroscrope.getAngleZ();
  AnglePitch = Gyroscrope.getAngleY();
  AngleYaw = Gyroscrope.getAngleX();
}

void RetrieveBMPSensorData()
{
  //retrieves sensor data
  Temperature = BMPSensor.readTemperature();
  Pressure = BMPSensor.readPressure();
  Altitude = BMPSensor.readAltitude();
  DeltaAltitude = abs(Altitude - StartingAltitude);
}

void DeployRecoverySystem()
{
  //moves an internal servo that will push the nose cone of dragging the shoot out
  Servo5.write(180);
}

void LogData()
{
  //writes all relavent variables to a file
  if ((millis() - timer) > 200)
  {
    FileHandler = SD.open("DATA.txt", FILE_WRITE);
    if (FileHandler)
    {
      Serial.println(" Data was written ");
      FileHandler.print("Time/ms: "); FileHandler.print(millis()); FileHandler.print(" ");
      FileHandler.print("Alt/m: "); FileHandler.print(Altitude); FileHandler.print(" ");
      FileHandler.print("Delta Alt/m: "); FileHandler.print(DeltaAltitude); FileHandler.print(" ");
      FileHandler.print("Max Alt/m: "); FileHandler.print(MaxAltitude); FileHandler.print(" ");
      FileHandler.print("Delta Max Alt/m: "); FileHandler.print(DeltaMaxAltitude); FileHandler.print(" ");
      FileHandler.print("Temp/'C: "); FileHandler.print(Temperature); FileHandler.print(" ");
      FileHandler.print("Pre/hPa: "); FileHandler.print(Pressure); FileHandler.print(" ");
      FileHandler.print("Yaw/' "); FileHandler.print(AngleYaw); FileHandler.print(" ");
      FileHandler.print("Pitch/': "); FileHandler.print(AnglePitch); FileHandler.print(" ");
      FileHandler.print("Roll/': "); FileHandler.print(AngleRoll); FileHandler.println(" ");
    }
    else
    {
      Serial.print(" Failed to write data ");
      FilePresent = false;
    }
    FileHandler.close();
    timer = millis();  
  }
}

void OutputOnSerial()
{

  Serial.print("accX/g "); Serial.print(Gyroscrope.getAccX()); Serial.print(" ");
  Serial.print("accY/g "); Serial.print(Gyroscrope.getAccY()); Serial.print(" ");
  Serial.print("accZ/g "); Serial.print(Gyroscrope.getAccZ()); Serial.print(" ");
  Serial.print("Time/ms: "); Serial.print(millis()); Serial.print(" ");
  Serial.print("Alt/m: "); Serial.print(Altitude); Serial.print(" ");
  Serial.print("Delta Alt/m: "); Serial.print(DeltaAltitude); Serial.print(" ");
  Serial.print("Max Alt/m: "); Serial.print(MaxAltitude); Serial.print(" ");
  Serial.print("Delta Max Alt/m: "); Serial.print(DeltaMaxAltitude); Serial.print(" ");
  Serial.print("Temp/'C: "); Serial.print(Temperature); Serial.print(" ");
  Serial.print("Pre/hPa: "); Serial.print(Pressure); Serial.print(" ");
  Serial.print("Yaw/' "); Serial.print(AngleYaw); Serial.print(" ");
  Serial.print("Pitch/': "); Serial.print(AnglePitch); Serial.print(" ");
  Serial.print("Roll/': "); Serial.print(AngleRoll); Serial.println(" ");
}

void loop() 
{
  RetrieveGyroData();
  if (Launched == true)
  {
    Servo1.write(map(AnglePitch, -180, 180, 0, 180) - Servo1Offset);
    Servo2.write(map(AnglePitch, -180, 180, 180, 0) + Servo2Offset);
    Servo3.write(map(AngleYaw, -180, 180, 180, 0) - Servo3Offset);
    Servo4.write(map(AngleYaw, -180, 180, 0, 180) + Servo4Offset);

    if (Altitude > MaxAltitude)
    {
      MaxAltitude = Altitude;
      DeltaMaxAltitude = MaxAltitude - StartingAltitude;
    }
    //checks to see if the altitude is decreasing
    if ((DeltaAltitude < (DeltaMaxAltitude - AltitudeUncertainty)) || ((millis() - MainStartTime) > 3500))
    {
      DeployRecoverySystem();
    }

    RetrieveBMPSensorData();
    if (FilePresent == true)
    {
      LogData();
    }
  }
  else
  {
    if ((abs(Gyroscrope.getAccZ()) > 1.2))// || (abs(Gyroscrope.getAccX()) < 0.9) || (abs(Gyroscrope.getAccY()) < 0.9))
    {
      //Gyroscrope.calcOffsets();
      Launched = true;
      MainStartTime = millis();
    }
  }

  OutputOnSerial();

}
