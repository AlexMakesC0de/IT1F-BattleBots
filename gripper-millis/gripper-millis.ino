#define PIN_GRIPPER 2 // pin 2 is wired to the grippers
#define GRIPPER_OPEN 1750 // 1750 μs rotation
#define GRIPPER_CLOSE 1220 // 1220 μs rotation

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_GRIPPER, OUTPUT);
  digitalWrite(PIN_GRIPPER, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  gripper(GRIPPER_OPEN);
  delay(500);
  gripper(GRIPPER_CLOSE);
  delay(500);
}

void gripper(int pulse) {
  static unsigned long timer = 0;
  static int lastPulse;
  unsigned long currentTime = millis();

  if (pulse > 0)
  {
    lastPulse = pulse;
  }
  else
  {
    pulse = lastPulse;
  }

  if (currentTime >= timer)
  {
    digitalWrite(PIN_GRIPPER, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(PIN_GRIPPER, LOW);
    timer = currentTime + 20;
  }
}