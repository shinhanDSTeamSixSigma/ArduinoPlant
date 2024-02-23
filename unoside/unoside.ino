#include <DHT.h>
#include <SoftwareSerial.h>
DHT dht(A2, DHT11);

int soil = 0;
int thomer = 0;
int humid = 0;
int lux = 0;
SoftwareSerial mySerial(4,5);
bool isWater = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  dht.begin();
  pinMode(A3, INPUT);
  mySerial.begin(115200);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
}
void watering(){
  if(mySerial.available() > 0){
      mySerial.readBytesUntil(".");
  }
  if(isWater){
    isWater = false;
    
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  soil = analogRead(A1);
  thomer = dht.readTemperature();
  humid = dht.readHumidity();
  lux = analogRead(A3);
  mySerial.write(thomer);
  mySerial.print(",");
  mySerial.write(humid);
  mySerial.print(",");
  mySerial.write(lux);
  mySerial.print(",");
  mySerial.write(soil);
  
}
