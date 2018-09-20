#define led_vcc 6
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led_vcc, OUTPUT);
  digitalWrite(led_vcc, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("HIGH");
  digitalWrite(led_vcc, HIGH);
  delay(1000);
  Serial.println("LOW");
  digitalWrite(led_vcc, LOW);
  delay(1000);
}
