// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  Serial.begin(115200);
}

float distance = _DIST_MAX;

void loop() { 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  int pwm_val = 255; // 기본은 꺼짐(active low)
  if ((distance == 0.0) || (distance > _DIST_MAX)) {
    pwm_val = 255; // 꺼짐
  } else if (distance < _DIST_MIN) {
    pwm_val = 255; // 꺼짐
  } else if (distance <= 200.0) {
    pwm_val = map(distance, 100, 200, 255, 0);
  } else {
    pwm_val = map(distance, 200, 300, 0, 255);
  }
  analogWrite(PIN_LED, pwm_val);

  // 시리얼 모니터에 pwm 값 미출력
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.println("");

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
