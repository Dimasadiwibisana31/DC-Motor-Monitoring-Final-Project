#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_INA219.h>
#include <SoftwareSerial.h>
#include <DS3231.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>

SoftwareSerial esp32(2, 3); // RX,TX B6,B7

// MPU6050 Processing Vibration
MPU6050 mpu;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// INA219 Processing
const int INA_addr = 0x40;
Adafruit_INA219 ina219(INA_addr);
float voltage = 0;
float current = 0;
float power = 0;

//LM393 Processing

// RTC Processing
DS3231 myRTC(SDA, SCL);
Time time;
bool isTimeSetup = false;
String getTimeConvert(); // Y-m-d H:i:s
String print2digits(int number);

// Micro SD Processing
File myFile;
bool isHeaderSetup = false;
int inputSDCard = 4; // A4

void setup() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(115200);
  esp32.begin(15200);
  ina219.begin();
  myRTC.begin();

  // Setup RTC Module
  //rtc.setDOW(WEDNESDAY);       // Set Day-of-Week to SUNDAY
  //rtc.setTime(15, 57, 43);    // Set the time to 12:00:00 (24hr format) Hour-minutes-Sec
  //rtc.setDate(28, 10, 2023);  // Set the date to Day-Month-Year

  while (!Serial)
    ;

  Serial.print("Initializing SD card...");
  if (!SD.begin(inputSDCard)) {
    Serial.println("initialization failed!");
    while (1)
      ;
  }
  Serial.println("Initialization done.");
}

void loop() {
  // MPU6050 Processing
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float mmPerSec2x = ax * 9.81 * 1000;
  float mmPerSec2y = ay * 9.81 * 1000;
  float mmPerSec2z = az * 9.81 * 1000;

  Wire.beginTransmission(0x68);  // MPU-6050 address
  Wire.write(0x3B);              // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);      // Request 14 bytes from the MPU-6050
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  float accX = ((float)AcX) / 2048.0;
  float accY = ((float)AcY) / 2048.0;
  float accZ = ((float)AcZ) / 2048.0;
  double roll = atan2(AcY, AcZ) * (180.0 / M_PI);

  // INA219 Processing
  voltage = ina219.getBusVoltage_V();
  current = ina219.getCurrent_mA() * 1000;  // Convert from Miliampere to Ampere
  power = voltage * (current / 1000);

  // LM393 Processing

  // Sending to Serial ESP32
  String send = String(accX) + ";" + String(accY) + ";" + String(accZ) + ";" + String(roll) + ";" + String(voltage) + ";" + String(current) + ";" + String(power);
  // String send = String(accX) + ";" + String(accY) + ";" + String(accZ) + ";" + String(voltage) + ";" + String(current) + ";" + String(power) + ";" + String(rpm);
  //String send = String(accX) + ";" + String(accY) + ";" + String(accZ) + ";" + String(roll) + ";" + String(voltage) + ";" + String(current) + ";" + String(power) + ";" + String(rpm);
  Serial.println(send);
  esp32.println(send);

  // Write to SD Card
  myFile = SD.open("Dataset.csv", FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to Dataset.csv...");

    if (!isHeaderSetup) {
      myFile.println("Voltage (V), Current (A), Power (W), Update Time");
      // myFile.println("accX (mm/s), accY (mm/s), accZ (mm/s) , Voltage (V), Current (A), Power (W), Update Time");
      isHeaderSetup = true;
    }

    // myFile.print(String(AccX));
    // myFile.print(";");
    // myFile.print(String(AccY));
    // myFile.print(";");
    // myFile.print(String(AccZ));
    // myFile.print(";");
    myFile.print(String(voltage));
    myFile.print(",");
    myFile.print(String(current));
    myFile.print(",");
    myFile.print(String(power));
    myFile.print(",");
    // myFile.print(String(rpm));
    // myFIle.print(";");
    myFile.println(getTimeConvert()); // Y-m-d H:i:s

    myFile.close();
    Serial.println("Done.");
  } else {
    Serial.println("Error opening Dataset.csv");
  }

  delay(500);
}

String getTimeConvert() {
  time = myRTC.getTime();

  return String(time.year) + "-" + print2digits(time.mon) + "-" + print2digits(time.date) + " " + String(myRTC.getTimeStr());
}

String print2digits(int number) {
  if (number < 10) {
    return "0" + String(number);
  }
  return String(number);
}