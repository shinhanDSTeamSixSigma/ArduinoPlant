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
  /*if(mySerial.available() > 0){
      mySerial.readBytesUntil('.');
      isWater = true;
  }*/
}

void loop() {
  // put your main code here, to run repeatedly:
  
    digitalWrite(9,HIGH);
    delay(5000);
    digitalWrite(9,LOW);
  soil = analogRead(A1);
  thomer = dht.readTemperature();
  humid = dht.readHumidity();
  lux = analogRead(A3);
  //200~1000 사이 값을 변경
  soil = map(soil,1023, 200, 0 ,100);
  //900 ~100 사이 값을 변경
  lux = map(lux, 900, 100, 0, 100);


  mySerial.print(thomer);
  mySerial.print(",");
  mySerial.print(humid);
  mySerial.print(",");
  mySerial.print(lux);
  mySerial.print(",");
  mySerial.print(soil);
  mySerial.print("\n");
  
  Serial.print(thomer);
  Serial.print(",");
  Serial.print(humid);
  Serial.print(",");
  Serial.print(lux);
  Serial.print(",");
  Serial.print(soil);
  Serial.print("\n");
  watering();
  if(true){
    delay(300000);
    digitalWrite(9,HIGH);
    delay(5000);
    digitalWrite(9,LOW);
    delay(1200000);
  }else{
    delay(60000);
  }
}
