/*
  Example of DataFlash library.
  Code by Jordi Muñoz and Jose Julio. DIYDrones.com
*/

#include <DataFlash.h>

#define HEAD_BYTE1 0xA3
#define HEAD_BYTE2 0x95

void setup()
{
  Serial.begin(57600);
  DataFlash.Init();       // DataFlash initialization
  
  Serial.println("Dataflash Log Test 1.0");

  // Test
  delay(20);
  DataFlash.ReadManufacturerID();
  delay(10);
  Serial.print("Manufacturer:");
  Serial.print(int(DataFlash.df_manufacturer));
  Serial.print(",");
  Serial.print(int(DataFlash.df_device_0));
  Serial.print(",");
  Serial.print(int(DataFlash.df_device_1));
  Serial.println();
  
  // We start to write some info (sequentialy) starting from page 1
  // This is similar to what we will do...
  DataFlash.StartWrite(1);
  Serial.println("Writing to flash... wait...");
  for (int i=0;i<1000;i++)   // Write 1000 packets...
    {
    // We write packets of binary data... (without worry about nothing more)
    DataFlash.WriteByte(HEAD_BYTE1);
    DataFlash.WriteByte(HEAD_BYTE2);
    DataFlash.WriteInt(2000+i);
    DataFlash.WriteInt(2001+i);
    DataFlash.WriteInt(2002+i);
    DataFlash.WriteInt(2003+i);
    DataFlash.WriteLong((long)i*5000);
    DataFlash.WriteLong((long)i*16268);
    DataFlash.WriteByte(0xA2);   // 2 bytes of checksum (example)
    DataFlash.WriteByte(0x4E);
    delay(10);
    }
  delay(100);
}

void loop()
{
  int i;
  byte tmp_byte1;
  byte tmp_byte2;
  int tmp_int;
  long tmp_long;
  
  Serial.println("Start reading page 1...");
  DataFlash.StartRead(1);   // We start reading from page 1
  for (i=0;i<200;i++)   // Read 200 packets...
    {
    tmp_byte1=DataFlash.ReadByte();
    tmp_byte2=DataFlash.ReadByte();
    Serial.print("PACKET:");
    if ((tmp_byte1==HEAD_BYTE1)&&(tmp_byte1==HEAD_BYTE1))
      {
      // Read 4 ints...
      tmp_int=DataFlash.ReadInt();
      Serial.print(tmp_int);
      Serial.print(",");
      tmp_int=DataFlash.ReadInt();
      Serial.print(tmp_int);
      Serial.print(",");
      tmp_int=DataFlash.ReadInt();
      Serial.print(tmp_int);
      Serial.print(",");
      tmp_int=DataFlash.ReadInt();
      Serial.print(tmp_int);
      Serial.print(",");
      
      // Read 2 longs...
      tmp_long=DataFlash.ReadLong();
      Serial.print(tmp_long);
      Serial.print(",");
      tmp_long=DataFlash.ReadLong();
      Serial.print(tmp_long);
      Serial.print(";");
      
      // Read the checksum... 
      tmp_byte1=DataFlash.ReadByte();
      tmp_byte2=DataFlash.ReadByte();
      }    
    Serial.println();
    }
  
  delay(10000);
}