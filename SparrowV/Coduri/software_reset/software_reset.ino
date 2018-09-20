int count;
#define resetPin 10

void setup() {
  // put your setup code here, to run once:
  count = 0;
  Serial.begin(9600);
  pinMode(resetPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("count ");
  Serial.println(count);

  delay(1000);
  if (count == 4) {
    digitalWrite(resetPin, LOW);
    delay(1000);
    digitalWrite(resetPin, HIGH);
    RSTN = ;
  }
  else
    count++;
}
