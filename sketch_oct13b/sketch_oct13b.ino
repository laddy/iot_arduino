int Trig = 8;
int Echo = 9;
int LED = 13;
int Duration;
float Distance;

void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(1);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(11);
  digitalWrite(Trig, LOW);
  Duration = pulseIn(Echo, HIGH);
  if (Duration > 0) {
    Distance = Duration / 2;
    Distance = Distance * 340 * 100 / 1000000; // ultrasonic speed is 340m/s = 34000cm/s = 0.034cm/us
    Serial.print(Duration);
    Serial.print(" us ");
    Serial.print(Distance);
    Serial.println(" cm");

    if ( Distance < 30 ) {
      digitalWrite(LED, HIGH);
    }
    else {
      digitalWrite(LED, LOW);
    }
  }
  delay(500);
}
