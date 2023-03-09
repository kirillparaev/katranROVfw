#include <MultiStepper.h>
#include <AccelStepper.h>
#include <MS5837.h>
#include <AFMotor.h>
#include <MPU6050.h>
#include <SPI.h>
#include <Servo.h>
#include <Ethernet.h>
#include <EthernetUDP.h>
#include <ArduinoOTA.h>
#include "I2Cdev.h"
#include "Wire.h"

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
#define BUFFER_SIZE_CALIBRATION 100

Servo thruster[6];
enum thrusters
{
    VERTICAL_FRONT,
    VERTICAL_BACK,
    HORIZONTAL_FRONT_RIGHT,
    HORIZONTAL_BACK_RIGHT, 
    HORIZONTAL_BACK_LEFT, 
    HORIZONTAL_FRONT_LEFT 
};

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = { 3, 10 };  // INA: Clockwise input
int inBpin[2] = { 4, 6 }; // INB: Counter-clockwise input
int ENpin[2] = { 5, 7 }; // PWM input

MPU6050 GyroAccel;
MS5837 barometer;
int16_t ax, ay, az;
int16_t gx, gy, gz;
byte mac_local[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip_local(192,168,0,177);
unsigned int port_local = 8080;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
EthernetUDP udp;
AF_DCMotor manipulator_0(1);
AF_DCMotor manipulator_1(2);

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(53, OUTPUT);
  Ethernet.init(10);
  Ethernet.begin(mac_local, ip_local);
  
  thruster[VERTICAL_FRONT].attach(8);
  thruster[VERTICAL_BACK].attach(9);
  thruster[HORIZONTAL_FRONT_RIGHT].attach(11);
  thruster[HORIZONTAL_BACK_RIGHT].attach(13);
  thruster[HORIZONTAL_BACK_LEFT].attach(2);
  thruster[HORIZONTAL_FRONT_LEFT].attach(12);

  for (int i = 0; i <= 5; i++) {
      thruster[i].write(180);
      delay(2);
      thruster[i].write(90);
      delay(1.5);
  }

  /*delay(2);
  for (int i = 0; i <= 5; i++) {
      thruster[i].write(90);
  }
  delay(1.5);*/
  Serial.begin(9600);
  Wire.begin();
  for (int i = 0; i < 2; i++)
  {
      pinMode(inApin[i], OUTPUT);
      pinMode(inBpin[i], OUTPUT);
      pinMode(ENpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
      digitalWrite(inApin[i], LOW);
      digitalWrite(inBpin[i], LOW);
  }
  if (!barometer.begin()) {
      Serial.println("Init failed!");
      Serial.println("Are SDA/SCL connected correctly?");
      Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
      Serial.println("\n\n\n");
  }
  else {
      Serial.println("Barometer has initialized!!!");
      barometer.setModel(MS5837::MS5837_30BA);
      barometer.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater);
  }
  
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  udp.begin(port_local);
  Serial.print("Arduino IP and port: ");
  Serial.print(ip_local);
  Serial.print(":");
  Serial.println(port_local);
  ArduinoOTA.begin(ip_local, "Arduino", "password", InternalStorage);
  //calibration();
}

void loop() 
{
    ArduinoOTA.poll();
    // check for updates
    int packetSize = udp.parsePacket();
    // получение данных ПЕРЕД пакетом
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
        Serial.println(packetBuffer);
    }
    // отправка данных ПОСЛЕ получения нового пакета
    for (int i = 0; i <= 5; i++) {
        thruster[i].write(packetBuffer[i]);
    }
    manipulatorActions(packetBuffer[6], packetBuffer[7]);

    GyroAccel.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
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

void motorOff(int motor)
{
    // Initialize braked
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(inApin[i], LOW);
        digitalWrite(inBpin[i], LOW);
    }
    analogWrite(ENpin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.

 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled

 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND

 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
    if (motor <= 1)
    {
        if (direct <= 4)
        {
            // Set inA[motor]
            if (direct <= 1)
                digitalWrite(inApin[motor], HIGH);
            else
                digitalWrite(inApin[motor], LOW);

            // Set inB[motor]
            if ((direct == 0) || (direct == 2))
                digitalWrite(inBpin[motor], HIGH);
            else
                digitalWrite(inBpin[motor], LOW);

            analogWrite(ENpin[motor], pwm);
        }
    }
}

void manipulatorActions(int cgrip, int crotate) {
   if (crotate == 0) {
       motorGo(0, CCW, 1023);
    }
    else if (crotate == 2) {
       motorGo(0, CW, 1023);
    }
    else {
        motorOff(0);
    }

    if (cgrip == 0) {
        motorGo(1, CCW, 1023);
    }
    else if (cgrip == 2) {
        motorGo(1, CW, 1023);
    }
    else {
        motorOff(1);
    }
}