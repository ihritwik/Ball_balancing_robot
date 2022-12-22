#define encoder1PinA 2
#define encoder1PinB 3
volatile long encoder1Pos = 0;
unsigned int A1old = 0;
unsigned int B1new = 0;
double pitch=0;
void setup() {
  pinMode(A0,INPUT);
  //pinMode(A1,INPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(encoder1PinA,INPUT);
  pinMode(encoder1PinB,INPUT);
  attachInterrupt (0, doEncoder1A, CHANGE);
  attachInterrupt (1, doEncoder1B, CHANGE);
  Serial.begin(9600);
  // put your setup code here, to run once:
}
int pos=0;
double newpos, oldpos, newtime=0, oldtime=0, vel, rpm, tmp=0;
String posRead="";

void loop() {

  pitch= (double)((double)(encoder1Pos)/4000)*360;
  newpos=pitch;
  newtime=(0.001*millis());
  vel=(newpos-oldpos)/(newtime-oldtime);
  oldpos=newpos;
  oldtime=newtime;
  rpm=vel/6;
  int val1=0,pwm1=0;
  int val2=0;
  //int pwm=0;
  //float pwm2;
  val1=analogRead(A0);
  val2=analogRead(A1);
  pwm1=map(val1,0,1024,0,255);
  //pwm2= (7.365 + (0.68*rpm));
  //pwm = (int)pwm2;
  analogWrite(9,pwm1);
  digitalWrite(8,HIGH);
  Serial.print("val2 = ");
  Serial.print(val2);
  Serial.print("     ");
  Serial.print("pwm = ");
  Serial.print(pwm1);
  Serial.print("     ");
  Serial.print("rpm = ");
  Serial.print(rpm);
  Serial.println();
  // put your main code here, to run repeatedly:
}

void doEncoder1A(){
  B1new^A1old ? encoder1Pos++:encoder1Pos--;
  A1old = digitalRead(encoder1PinA);
  //if (encoder1Pos >= 4000 || encoder1Pos <= (-4000))
  //encoder1Pos=0;
}
void doEncoder1B(){
  B1new = digitalRead(encoder1PinB);
  B1new^A1old ? encoder1Pos++:encoder1Pos--;
  //if (encoder1Pos >= 4000 || encoder1Pos <= (-4000))
  //encoder1Pos=0;
}

