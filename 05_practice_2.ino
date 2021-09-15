#define PIN_LED 7

void setup() {
   pinMode(PIN_LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int i=0;
  digitalWrite(PIN_LED, 0);
  delay(1000);
  while(i<10){
  digitalWrite(PIN_LED, 1);
  delay(100);
  i++;
  digitalWrite(PIN_LED, 0);
  delay(100);
  i++;
  }
  while(1){
    digitalWrite(PIN_LED, 1);
  }
}
