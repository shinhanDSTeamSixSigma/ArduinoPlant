#include <DHT.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
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
  mySerial.begin(9600);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
}
void watering(){
  if(mySerial.available() > 0){
      mySerial.readBytesUntil(".");
      isWater = true;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  soil = analogRead(A1);
  thomer = dht.readTemperature();
  humid = dht.readHumidity();
  lux = analogRead(A3);
  //200~1000 사이 값을 변경
  soil = map(soil,1023, 200, 0 ,100);
  //210 ~50 사이 값을 변경
  lux = map(lux, 210, 50, 0, 100);


  mySerial.write(thomer);
  mySerial.print(",");
  mySerial.write(humid);
  mySerial.print(",");
  mySerial.write(lux);
  mySerial.print(",");
  mySerial.write(soil);
  Serial.write(thomer);
  Serial.print(",");
  Serial.write(humid);
  Serial.print(",");
  Serial.write(lux);
  Serial.print(",");
  Serial.write(soil);
  watering();
  if(isWater){
    digitalWrite(9,HIGH);
    delay(5000);
    digitalWrite(9,LOW);
    delay(58000);
  }else{
    delay(60000);
  }
}
