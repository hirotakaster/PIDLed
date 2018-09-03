#include <Arduino.h>

// PIDのゲイン設定
#define Kp      10.0
#define Ki      0.0
#define Kd      0.0
#define setval  2.5

unsigned long preTime;
float P, I, D, preP;
int pwm = 0;

void setup() {
  Serial.begin(115200);
  preTime = millis();
  P = I = D = 0.0;
}

void loop() {

  // A0からデータ取得
  float vol = 0.0;
  for (int i = 0; i < 100; i++) {
    vol += analogRead(0);
  }
  vol = 3.3 * (vol / 100) / 1023;

  unsigned long now = millis();
  float dt = (now - preTime) / 1000.00;
  preTime = now;

  // PID計算
  P  = setval - vol;
  I += P * dt;
  D  = (P - preP) / dt;
  preP = P;
  pwm += Kp * P + Ki * I + Kd * D;

  // 出力
  analogWrite(3, pwm);

  Serial.print(vol);  Serial.print(",");
  Serial.print(pwm);
  Serial.println("");
}
