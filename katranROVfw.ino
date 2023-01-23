#include <MultiStepper.h>
#include <AccelStepper.h>
#include <MS5837.h>
#include <AFMotor.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Servo.h>
#include <Ethernet.h>
#include <EthernetUDP.h>
#include "I2Cdev.h"
#include "Wire.h"

#define BUFFER_SIZE_CALIBRATION 100
Servo thruster[6];
enum thrusters
{
    VERTICAL_0, //0
    VERTICAL_1, //1
    HORIZONTAL_0, //2
    HORIZONTAL_1, //3
    HORIZONTAL_2, //4
    HORIZONTAL_3  //5
};
MPU6050 GyroAccel;
MS5837 barometer;
int16_t ax, ay, az;
//избавиться от глобальных переменных
int16_t gx, gy, gz;
byte mac_local[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip_local(192,168,0,177);
unsigned int port_local = 8080;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // 24 байта
EthernetUDP udp;
AF_DCMotor manipulator_0(1);
AF_DCMotor manipulator_1(2);

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(53, OUTPUT);
  Ethernet.init(10);
  Ethernet.begin(mac_local, ip_local);
  
  for (int i = 0; i <= 5; i++) {  // pins 3-8
      thruster[i].attach(i + 3);
  }

  for (int i = 0; i <= 5; i++) {
      thruster[i].write(180);
  }

  delay(2);
  for (int i = 0; i <= 5; i++) {
      thruster[i].write(90);
  }
  delay(1.5);
  Serial.begin(9600);
  Wire.begin();
  if (!barometer.begin()) {
      Serial.println("Init failed!");
      Serial.println("Are SDA/SCL connected correctly?");
      Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
      Serial.println("\n\n\n");
  }
  else {
      Serial.println("Barometer has initialized!!!");
      barometer.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)В;
  }
  barometer.setModel(MS5837::MS5837_30BA);
 
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  udp.begin(port_local);
  Serial.print("Arduino IP and port: ");
  Serial.print(ip_local);
  Serial.print(":");
  Serial.println(port_local);
  //calibration();
}

void loop() 
{
    int packetSize = udp.parsePacket();
    //updateBarometerReadings();

    if (packetSize) {
        Serial.print("Remote PC IP and port: ");
        Serial.print(udp.remoteIP());
        Serial.print(":");
        Serial.println(udp.remotePort());
        Serial.print("Size of packet: ");
        Serial.println(packetSize);

        udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
        Serial.println("Contents:");
        //for (int i = 0; i <= 6; i++) {
        //    packetBuffer[i] = atoi(&packetBuffer[i]);    
        //}
        Serial.println(packetBuffer);

        if (packetBuffer[6] == 0xFF) {
            calibration();
        }
        manipulatorActions(packetBuffer[7], packetBuffer[8]);
    }

    for (int i = 0; i <= 5; i++) {
        thruster[i].write(packetBuffer[0 + i]);
    }

    delay(15);  // waits for the servo to get there

    // TODO : вынести в отдельную функцию и заставить работать постоянно
    // расшифровать сырые данные
    // пересмотреть структуру udp пакета для отправки данных наверх
    // прикрутить калибровку и команду на неё сверxy
    GyroAccel.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    /*Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);*/
}

void calibration() {
    long offsets[6];
    long offsetsOld[6];
    int16_t mpuGet[6];
    // используем стандартную точность
    GyroAccel.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    GyroAccel.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    // обнуляем оффсеты
    GyroAccel.setXAccelOffset(0);
    GyroAccel.setYAccelOffset(0);
    GyroAccel.setZAccelOffset(0);
    GyroAccel.setXGyroOffset(0);
    GyroAccel.setYGyroOffset(0);
    GyroAccel.setZGyroOffset(0);
    delay(10);
    Serial.println("Calibration start. It will take about 5 seconds");
    for (byte n = 0; n < 10; n++) {     // 10 итераций калибровки
        for (byte j = 0; j < 6; j++) {    // обнуляем калибровочный массив
            offsets[j] = 0;
        }
        for (byte i = 0; i < 100 + BUFFER_SIZE_CALIBRATION; i++) { // делаем BUFFER_SIZE измерений для усреднения
            GyroAccel.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
            if (i >= 99) {                         // пропускаем первые 99 измерений
                for (byte j = 0; j < 6; j++) {
                    offsets[j] += (long)mpuGet[j];   // записываем в калибровочный массив
                }
            }
        }
        for (byte i = 0; i < 6; i++) {
            offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE_CALIBRATION); // учитываем предыдущую калибровку
            if (i == 2) offsets[i] += 16384;                               // если ось Z, калибруем в 16384
            offsetsOld[i] = offsets[i];
        }
        // ставим новые оффсеты
        GyroAccel.setXAccelOffset(offsets[0] / 8);
        GyroAccel.setYAccelOffset(offsets[1] / 8);
        GyroAccel.setZAccelOffset(offsets[2] / 8);
        GyroAccel.setXGyroOffset(offsets[3] / 4);
        GyroAccel.setYGyroOffset(offsets[4] / 4);
        GyroAccel.setZGyroOffset(offsets[5] / 4);
        delay(2);
    }
}

void updateBarometerReadings() {
    // Update pressure and temperature readings
    barometer.read();

    Serial.print("Pressure: ");
    Serial.print(barometer.pressure());
    Serial.println(" mbar");

    Serial.print("Temperature: ");
    Serial.print(barometer.temperature());
    Serial.println(" deg C");

    Serial.print("Depth: ");
    Serial.print(barometer.depth());
    Serial.println(" m");

    Serial.print("Altitude: ");
    Serial.print(barometer.altitude());
    Serial.println(" m above mean sea level");

    delay(1000);
}

void manipulatorActions(int cgrip, int crotate) { // 0-255
    float val_grip = fabs(cgrip);
    float val_rotate = fabs(crotate);
    int sign_grip, sign_rotate;

    if (val_grip != 0) {
        sign_grip = val_grip / cgrip;
    }
    else {
        sign_grip = 0;
    }

    if (val_rotate != 0) {
        sign_rotate = val_rotate / crotate;
    }
    else {
        sign_rotate = 0;
    }

    if (sign_grip >= 0) { // 1 - на мотор шилде
        manipulator_0.run(FORWARD);
    }
    else {
        manipulator_0.run(BACKWARD);
    }
    manipulator_0.setSpeed(val_grip);

    /*if (sign_rotate >= 0) { //  - на мотор шилде
        manipulator_1.run(FORWARD);
    }
    else {
        manipulator_1.run(BACKWARD);
    }
    manipulator_1.setSpeed(val_rotate);*/


    //-------------------------------------------
    /*for (int i = 0; i < 255; i++) {
        manipulator_0.run(FORWARD);
        manipulator_0.setSpeed(i);
    }
    for (int i = 0; i < 255; i++) {
        manipulator_0.run(BACKWARD);
        manipulator_0.setSpeed(i);
    }
    for (int i = 0; i < 255; i++) {
        manipulator_1.run(FORWARD);
        manipulator_1.setSpeed(i);
    }
    for (int i = 0; i < 255; i++) {
        manipulator_1.run(BACKWARD);
        manipulator_1.setSpeed(i);
    }*/
}