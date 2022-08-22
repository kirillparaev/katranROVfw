#include <SPI.h>
#include <Servo.h>
#include <Ethernet.h>
#include <EthernetUDP.h>

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

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168,1,177);
unsigned int localPort = 8888;

char buffer[48];
//char buffer[UDP_TX_PACKET_MAX_SIZE] // 24 байта
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

  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (Ethernet.hardwareStatus() == EthernetNoHardware) // Check for Ethernet hardware present
  {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) 
    {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }

  if (Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Ethernet cable is not connected.");
  }

  udp.begin(localPort);
}

void loop() 
{
    int packetSize = udp.parsePacket();
    if (packetSize) {
        udp.read(buffer, 48);
        Serial.println("Contents:");
        Serial.println(buffer);
    }
    for (int i = 0; i <= 5; i++) {
        thruster[i].write(buffer[42 + i]); // данные в пакете начинаются с 42 байта
    }
    delay(15);  // waits for the servo to get there
}
