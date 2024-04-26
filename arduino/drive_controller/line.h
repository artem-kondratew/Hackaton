#define LEFT_PWM 6
#define LEFT_DIR 7
#define RIGHT_PWM 5
#define RIGHT_DIR 4

#define K_P 5.0
#define K_D 20.0

#define V 190


namespace LineFollower {

bool FOLLOW_FLAG = false;
uint8_t cnt = 0;
uint64_t prev_cross = 0;

int left_min = 1023;
int left_max = 0;
int right_min = 1023;
int right_max = 0;

int errold = 0;

bool flag = true;
bool button_old = 1;

void drive(int left, int right)
{
  digitalWrite(LEFT_DIR, left > 0);
  digitalWrite(RIGHT_DIR, right > 0);

  analogWrite(LEFT_PWM, abs(left));
  analogWrite(RIGHT_PWM, abs(right));
}


void calib() {
  int tim = millis();
  while(1)
  {
    if(digitalRead(A1) == 1)
    {
      delay(5);
      if(digitalRead(A1) == 1)
      {
        break; 
      }
    }
  }
  while(millis() - tim < 4000)
  {
    drive(200, -200);
    int left = analogRead(A3);
    int right = analogRead(A2);
    if(left < left_min) left_min = left;
    if(left > left_max) left_max = left; 
    if(right < right_min) right_min = right;
    if(right > right_max) right_max = right;
  }
  drive(0, 0);
  pinMode(A1, INPUT);
}


void turn_left()
{
  drive(-140, 140);
  delay(1000);
  drive(0, 0);
}


void follow() {
  if (FOLLOW_FLAG == false) {
    return;
  }
  int s1 = map(analogRead(A3), left_min, left_max, 0, 100);
  int s2 = map(analogRead(A2), right_min, right_max, 0, 100);
  double err = s1 - s2;
  double u = err * K_P + (err - errold) * K_D;
  drive(constrain(V + u, 250, -100), constrain(V - u, 250, -100));
  errold = err;
}

}
