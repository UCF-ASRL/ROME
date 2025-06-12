/*

Connections of Drive, Arduino and MAX485 TTL to RS485 Converter
Serial Port 0 is not used to connect to drive because its connected to USB-Serial and used to show information on console.

For Arduino Uno Software serial needs to be used as there is only one hardware serial port and its connected to USB-Serial. 
   Drive       MAX485 TTL to RS485 Converter
     A         -       A
     B         -       B
    GND        -      GND
    
   MAX485 TTL to RS485 Converter    Arduino (Software Serial)
     R0                  -            D2              
     DE & RE             -            D8
     DI                  -            D3
     GND                 -            GND
*/

#include <ModbusMaster.h>   
#include <SoftwareSerial.h> 
SoftwareSerial mySerial(2,3);    //Software Serial port For Arduino Uno (OBSOLETE)
#define MODBUS_DATA_TRANSACTION_PIN 8   //DE & RE Pin of MAX485
ModbusMaster node;                      //object for ModbusMaster

long int a1,a2,a3,a4,a5,a6,a7,a8;
uint8_t Slave_ID = 1;   //Slave ID of Drive

void setup() 
{
    pinMode(MODBUS_DATA_TRANSACTION_PIN, OUTPUT);
    digitalWrite(MODBUS_DATA_TRANSACTION_PIN, 0);
    Serial.begin(9600);     //Baud rate is 9600 8N1
    mySerial.begin(9600);   //Modbus Baud rate is 9600 8N1
    node.begin(mySerial);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
    Serial.println("StepperOnline DBLS-01S BLDC Driver");
    Serial.print("Slave ID : ");
    Serial.println(Slave_ID);
    
    a1 = node.RS485_Communication(Slave_ID, 0x0904);  // 0x0904 enable motor with RS485 // 0x4904 is for open loop 
    Serial.print("RS485 Communication status : ");
    Serial.println(a1); 
}

void loop() 
{
  Serial.println("Motor is running in Forward Direction");   
  
   //for (int i=0; i<=10; i+=1)
    //{
      a2 = node.Set_Speed(Slave_ID, 30);     // Set Speed, speed range is (0- 12 ---> max is (3500/50) RPM)
      delay(1000);

     while(1)
      {

      a3 = node.Read_Status(Slave_ID);
       Serial.print("Driver status: ");
       Serial.println(a3);
       delay(1000);

       a4 = node.Read_Speed(Slave_ID);
       Serial.print("Motor Speed: ");
       Serial.println(a4/10);
       delay(1000);

      };
    //};
}

void GetFeedback(void) 
{
  a6 = node.Speed_Feedback(Slave_ID);    
  a7 = node.Current_Feedback(Slave_ID);  
  a8 = node.Voltage_Feedback(Slave_ID);
  Serial.print("Set Speed :");
  Serial.print(a2);
  Serial.print("\t\tSpeed_Feedback :");
  Serial.print(a6);  
  Serial.print("\tCurrent_Feedback :");
  Serial.print(a7);
  Serial.print("\tVoltage_Feedback :");
  Serial.println(a8);
}

void preTransmission() 
{
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN, 1);
}

void postTransmission()
{
  digitalWrite(MODBUS_DATA_TRANSACTION_PIN, 0);
}
