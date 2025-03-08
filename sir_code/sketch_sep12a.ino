#define encoderPinA 2
#define encoderPinB 3
#define DIR1 8
#define PWM1 9
volatile long encodercount = 0;
float prevenccount =0;

long previoustime = 0;
long eprevious = 0;
long eIntegral = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encoderPinA, INPUT );
  pinMode(encoderPinB, INPUT );

  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleencoder, RISING );
}

void loop() {
  int target = 360;
  float kp = 2.0;
  float kd = 0.1;
  float ki = 0.01;
  float u = pidController(target, kp, kd, ki);
  moveMotor(DIR1, PWM1, u);

  //Serial.print(target);
  //Serial.print(", ");
  //Serial.print(encodercount);
  // put your main code here, to run repeatedly:
  if(encodercount!=prevenccount)
  {
  Serial.println(encodercount);}
}


void handleencoder() {
  if (digitalRead(encoderPinA) > digitalRead(encoderPinB))
  { encodercount++;}
  else {encodercount--;}
  
  prevenccount=encodercount;
}

void moveMotor(int dirpin, int pwmpin, float u)
{
  float speed = fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  int direction = 1;
  if (u < 0) {
    direction = 0;
  }

  digitalWrite(dirpin, direction);
  analogWrite(pwmpin, speed);
}

float pidController(int target, float kp, float kd, float ki )
{
  long currenttime = micros();
  float deltaT = ((float)(currenttime - previoustime) / 1.0e6) ;

  int e = encodercount - target;
  float ederivative = (e - eprevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;
  float u = (kp * e) + (kd * ederivative) + (ki * eIntegral);
  previoustime = currenttime;
  eprevious = e;
  return u;
}
