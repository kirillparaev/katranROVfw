#include <MPU6050.h>
#include <SPI.h>
#include <Servo.h>
#include <Ethernet.h>
#include <EthernetUDP.h>
#include "I2Cdev.h"
#include "Wire.h"

#define BUFFER_SIZE 100
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
//Servo th1, th2, th3, th4, th5, th6;
MPU6050 GyroAccel;
int16_t ax, ay, az;
int16_t gx, gy, gz;
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168,1,177);
unsigned int localPort = 8888;
char buffer[49];
//char buffer[UDP_TX_PACKET_MAX_SIZE] // 24 �����
EthernetUDP udp;

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(53, OUTPUT);
  Ethernet.begin(mac, ip);
  
  for (int i = 0; i <= 5; i++) {
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

  /*th1.attach(3);  // attaches the servo on pin 9 to the servo object
  th2.attach(4); 
  th3.attach(5);
  th4.attach(6); 
  th5.attach(7); 
  th6.attach(8);  

  th1.write(180);
  th2.write(180);
  th3.write(180);
  th4.write(180);
  th5.write(180);  
  th6.write(180);

  delay(2);

  th1.write(90);
  th2.write(90);
  th3.write(90);
  th4.write(90);
  th5.write(90);
  th6.write(90);

  delay(1.5);
  */
  Serial.begin(9600);
  Wire.begin();
  delay(100);

  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  udp.begin(localPort);
  calibration();
}

void loop() 
{
    int packetSize = udp.parsePacket();
    if (packetSize) {
        udp.read(buffer, 49);
        Serial.println("Contents:");
        Serial.println(buffer);
        if (buffer[48] == 0xFF) {
            calibration();
        }
    }
    for (int i = 0; i <= 5; i++) {
        thruster[i].write(buffer[42 + i]); // ������ � ������ ���������� � 42 �����
    }
    delay(15);  // waits for the servo to get there

    // TODO : ������� � ��������� ������� � ��������� �������� ���������
    // ������������ ����� ������
    // ������������ ��������� udp ������ ��� �������� ������ ������
    // ���������� ���������� � ������� �� �� ������
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
    // ���������� ����������� ��������
    GyroAccel.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    GyroAccel.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    // �������� �������
    GyroAccel.setXAccelOffset(0);
    GyroAccel.setYAccelOffset(0);
    GyroAccel.setZAccelOffset(0);
    GyroAccel.setXGyroOffset(0);
    GyroAccel.setYGyroOffset(0);
    GyroAccel.setZGyroOffset(0);
    delay(10);
    Serial.println("Calibration start. It will take about 5 seconds");
    for (byte n = 0; n < 10; n++) {     // 10 �������� ����������
        for (byte j = 0; j < 6; j++) {    // �������� ������������� ������
            offsets[j] = 0;
        }
        for (byte i = 0; i < 100 + BUFFER_SIZE; i++) { // ������ BUFFER_SIZE ��������� ��� ����������
            GyroAccel.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);
            if (i >= 99) {                         // ���������� ������ 99 ���������
                for (byte j = 0; j < 6; j++) {
                    offsets[j] += (long)mpuGet[j];   // ���������� � ������������� ������
                }
            }
        }
        for (byte i = 0; i < 6; i++) {
            offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // ��������� ���������� ����������
            if (i == 2) offsets[i] += 16384;                               // ���� ��� Z, ��������� � 16384
            offsetsOld[i] = offsets[i];
        }
        // ������ ����� �������
        GyroAccel.setXAccelOffset(offsets[0] / 8);
        GyroAccel.setYAccelOffset(offsets[1] / 8);
        GyroAccel.setZAccelOffset(offsets[2] / 8);
        GyroAccel.setXGyroOffset(offsets[3] / 4);
        GyroAccel.setYGyroOffset(offsets[4] / 4);
        GyroAccel.setZGyroOffset(offsets[5] / 4);
        delay(2);
    }
}
