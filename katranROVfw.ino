#include <SPI.h>
#include <Servo.h>
#include <Ethernet.h>
#include <EthernetUDP.h>

Servo th1, th2, th3, th4, th5, th6;
int val;   

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168,1,177);
unsigned int localPort = 8888;

char buffer[7];
char speedbuffer[7];
EthernetUDP udp;

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(53, OUTPUT);

  Ethernet.begin(mac, ip);
  
  th1.attach(3);  // attaches the servo on pin 9 to the servo object
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
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  
  int prev_val = val;
  int size = udp.parsePacket();
  
  udp.write(size, buffer);
  
  if ( size > 0 )
  {
    udp.read(buffer, size);
    Serial.println(size);

    
  
  
    val = constrain(val,0,180);  // scale it for use with the servo (value between 0 and 180)

    myservo.write(val); // sets the servo position according to the scaled value
    delay(15);  // waits for the servo to get there
    
  }
   udp.endPacket();
}
