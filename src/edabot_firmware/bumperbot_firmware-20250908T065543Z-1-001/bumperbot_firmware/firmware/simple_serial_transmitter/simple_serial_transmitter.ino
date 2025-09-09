#define LED 2

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    int x = Serial.readString().toInt();
    if (x == 0)
    {
      digitalWrite(LED, LOW);
    }
    else{
      digitalWrite(LED, HIGH);
    }
  }
}
