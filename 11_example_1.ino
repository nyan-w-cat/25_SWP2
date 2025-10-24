#include <Servo.h>

#define PIN_LED   9
#define PIN_TRIG  12
#define PIN_ECHO  13
#define PIN_SERVO 10

#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 180.0   // 18cm
#define _DIST_MAX 360.0   // 36cm
#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)
#define _EMA_ALPHA 0.3    // 실험을 통해 최적값 조정 가능

#define _DUTY_MIN 600     // 실측 0도 펄스폭
#define _DUTY_NEU 1500    // 실측 90도 펄스폭 (연속제어에서는 직접 사용 안함)
#define _DUTY_MAX 2400    // 실측 180도 펄스폭

float dist_ema = _DIST_MIN, dist_prev = _DIST_MIN;
unsigned long last_sampling_time;
Servo myservo;

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, HIGH);  // LED active-low, 초기 꺼짐

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_MIN);

  dist_prev = _DIST_MIN;
  dist_ema = _DIST_MIN;
  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_filtered;
  if (millis() < last_sampling_time + INTERVAL) return;

  // 초음파 거리 측정
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 범위 필터: 18~36cm 제한
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
      dist_filtered = dist_prev;
      digitalWrite(PIN_LED, HIGH);  // 범위 밖: LED 꺼짐 (active-low)
  } else if (dist_raw < _DIST_MIN) {
      dist_filtered = dist_prev;
      digitalWrite(PIN_LED, HIGH);  // 범위 밖: LED 꺼짐
  } else {
      dist_filtered = dist_raw;
      digitalWrite(PIN_LED, LOW);   // 범위 안: LED 켜짐 (active-low)
  }

  // ema 필터 적용
  dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_prev;
  dist_prev = dist_ema;

  // dist_ema가 360 이상이면 360으로 클램핑
  if (dist_ema > _DIST_MAX) dist_ema = _DIST_MAX;

  // 거리별 서보 각도 제어 (연속 변화)
  int duty;
  float deg = 0.0;
  if (dist_ema <= _DIST_MIN) {
    duty = _DUTY_MIN;
    deg = 0.0;
  } else if (dist_ema >= _DIST_MAX) {
    duty = _DUTY_MAX;
    deg = 180.0;
  } else {
    // 18~36cm 사이: 0~180도 연속 변화
    deg = (dist_ema - _DIST_MIN) * 180.0 / (_DIST_MAX - _DIST_MIN);
    duty = (int)(_DUTY_MIN + (deg / 180.0) * (_DUTY_MAX - _DUTY_MIN));
  }
  myservo.writeMicroseconds(duty);

  // 시리얼 플로터 출력
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(min(dist_raw, _DIST_MAX + 100));
  Serial.print(",ema:");   Serial.print(min(dist_ema, _DIST_MAX + 100));
  Serial.print(",Servo:"); Serial.print(myservo.read());
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");

  last_sampling_time += INTERVAL;
}
