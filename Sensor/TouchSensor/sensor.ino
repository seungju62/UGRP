int in0 = A0;
int in1 = A1;
int in2 = A2;                           // 센서값을 아나로그 A0핀 설정

void setup() {

  Serial.begin(115200);                           // 시리얼 통신 설정 (보드레이트 9600)

}

void loop(){

  int sensor0 = analogRead(in0);     // 센서값을 아나로그로 읽어 value 변수에 저장
  int sensor1 = analogRead(in1);
  int sensor2 = analogRead(in2);
  
  Serial.println(sensor0);                           // 센서값을 시리얼 모니터에 출력
  delay(10);                                         // 0.01초의 딜레이

}
