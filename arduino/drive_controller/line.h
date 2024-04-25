#define LEFT_PWM 6
#define LEFT_DIR 7
#define RIGHT_PWM 5
#define RIGHT_DIR 4

//#define K_P 1.0
//#define K_D 5.0

#define K_P 3.0
#define K_D 20.0

#define V 190

int left_min = 1023;
int left_max = 0;
int right_min = 1023;
int right_max = 0;

int errold = 0;

bool flag = true;
bool button_old = 1;

void drive(int left, int right)
{
  digitalWrite(LEFT_DIR, left <= 0);
  digitalWrite(RIGHT_DIR, right <= 0);

  analogWrite(LEFT_PWM, abs(left));
  analogWrite(RIGHT_PWM, abs(right));
}

void setup() {
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
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
    int left = analogRead(A2);
    int right = analogRead(A3);
    if(left < left_min) left_min = left;
    if(left > left_max) left_max = left; 
    if(right < right_min) right_min = right;
    if(right > right_max) right_max = right;
  }
  drive(0, 0);
  pinMode(A1, INPUT);
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
}

void loop() {
  if(flag)
  {
    int s1 = map(analogRead(A2), left_min, left_max, 0, 100);
    int s2 = map(analogRead(A3), right_min, right_max, 0, 100);
    double err = s1 - s2;
    double u = err*K_P + (err - errold)*K_D;
    drive(constrain(V + u, 250, -100), constrain(V - u, 250, -100));
    errold = err;
  }
  else
  {
    drive(0, 0);
  }
  //delay(1);
  if(digitalRead(A1) == 1 && button_old == 0)
  {
    delay(5);
    if(digitalRead(A1) == 1)
    {
      flag = !flag; 
    }
  }
  button_old = digitalRead(A1);
}