#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUDP.h>
#include <Servo.h>

Servo Thrusters[6]; //Vert1, Vert2, Hori1, Hori2, Hori3, Hori4
byte m_arduino_mac[6] =
{
      0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress m_arduino_ip[4] = { 192, 168,1,177 };
unsigned int localPort = 8888;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
EthernetClass connection;
EthernetUDP Udp;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(53, OUTPUT);
    //53 на вывод чтоб spi работал, w5100 - сетевой контроллер - на пине 10(вывод)
    //microSD - на вводе 4.
    analogWrite(10, 255);

    // инициализация моторов (пины от 2 до 7)
    for (int i = 0; i <= 5; i++) {
        Thrusters[i].attach(i + 2);
        Thrusters[i].write(2000);
        delay(2000);  // TODO: попробовать снизить значение
        Thrusters[i].write(1500);
        delay(10000); // TODO: попробовать снизить значение
    }

    //инициализация сети
    connection.begin(m_arduino_mac, m_arduino_ip);
    Udp.begin(localPort);
    Serial.begin(9600);
}

void loop()
{
    int size = Udp.parsePacket();
    digitalWrite(LED_BUILTIN, LOW);
    if (size)
    {
        Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
        Serial.println("Contents:");
        Serial.println(packetBuffer);

        digitalWrite(LED_BUILTIN, HIGH);
        for (int i = 0; i <= 5; i = i++)
        {
            Thrusters[i].write(packetBuffer[i]);
            delay(50);
        }
    }
}
