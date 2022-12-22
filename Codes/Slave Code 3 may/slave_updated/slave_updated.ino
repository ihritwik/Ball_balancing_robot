#include <Encoder.h>
#include <Wire.h>
#include <MatrixMath.h>

Encoder myEnc1(18, 26);
Encoder myEnc2(19, 27);
Encoder myEnc3(2, 5);
//definition of state variables and others
struct ephi
{
  float ep;
  float tim; 
};
ephi p1={0,0.01};
ephi p2={0,0.01};
ephi p3={0,0.01};
ephi p1_old={0,0.01};
ephi p2_old={0,0.01};
ephi p3_old={0,0.01};

float newtimec=0;
float oldtimec=0;


int ctr=0;
float test=0;
float pi=3.14159265359;
float vxd=0;
float vyd=0;
float vxdd=0;
float vydd=0;
float epx=0;
float epx1=0;
float epx_old=0;
float epy=0;
float epy1=0;
float epy_old=0;
float eix=0;
float eix1=0;
float eiy=0;
float eiy1=0;
float edx=0;
float edx1=0;
float edy=0;
float edy1=0;
float ep1=0;
float ed1=0;
float ei1=0;
float ep2=0;
float ed2=0;
float ei2=0;
float ep3=0;
float ed3=0;
float ei3=0;
float kp=10;
float ki=10;
float kd=0;
//float kpx=0.1745;
//float kix=0.1;
//float kdx=0.01;
//float kpy=0.2617;
//float kiy=0.1;
//float kdy=0.01;
float kpx=0;
float kix=0;
float kdx=0;
float kpy=0;
float kiy=0;
float kdy=0;
float mu1=0;
float mu2=0;
float mu3=0;
long newPosition1 = 0;
long newPosition2 = 0;
long newPosition3 = 0;
float newtime01=0;
float newtime02=0;
float oldtime01=0;
float oldtime02=0;
float newtime11=0;
float newtime12=0;
float newtime13=0;
float oldtime11=0;
float oldtime12=0;
float oldtime13=0;
float xs=0;
float xsdot=0;
float ys=0;
float ysdot=0;
float psix=0;
float psix1=0;
float psixdot=0;
float psixdot1=0;
float psiy=0;
float psiy1=0;
float psiydot=0;
float psiydot1=0;
float psiz=0;
float psiz1=0;
float psizdot=0;
float psizdot1=0;
float phi1=0;
float phi2=0;
float phi3=0;
float phi1dot=0;
float phi2dot=0;
float phi3dot=0;
float phi1ds=0;
float phi2ds=0;
float phi3ds=0;
float psix_old=0;
float psiy_old=0;
float psiz_old=0;
float pwm1f=0;
float pwm2f=0;
float pwm3f=0;
int pwm1=0;
int pwm2=0;
int pwm3=0;

//definition of system constants

float ms=0.6;
float mw=0.570;
float mb=6.78;
float Is=0.00585;
float Ibx=0.1791;
float Iby=0.1791;
float Ibz=0.0998;
float Iw=0.0007125;
float Im=0.0001;
float beta=1.04719;
float g=9.78;
float k=3;
float rw=0.05;
float rs=0.121;
float l=0.16;
float b=0.05;
float alpha=1.1;
float lam1=18;
float lam2=18;
float lam3=12;
//initialization of matrix variables;
float D111=0;
float D112=0;
float D113=0;
float D114=0;
float D115=0;
float D116=0;
float D117=0;
float D118=0;
float D119=0;
float D1110=0;
float D211=0;
float D212=0;
float D213=0;
float D214=0;
float D215=0;
float D216=0;
float D217=0;
float D218=0;
float D219=0;
float D2110=0;
float D311=0;
float D312=0;
float D313=0;
float D314=0;
float D315=0;
float D316=0;
float D317=0;
float D318=0;
float D319=0;
float D3110=0;
float D411=0;
float D412=0;
float D413=0;
float D414=0;
float D415=0;
float D416=0;
float D417=0;
float D418=0;
float D419=0;
float D4110=0;
float D511=0;
float D512=0;
float D513=0;
float D514=0;
float D515=0;
float D516=0;
float D517=0;
float D518=0;
float D519=0;
float D5110=0;
float e11=0;
float e21=0;
float e31=0;
float e41=0;
float e51=0;
float J211=0; 
float J212=0; 
float J213=0; 
float J221=0; 
float J222=0;
float J223=0; 
float J231=0; 
float J232=0; 
float J233=0; 
float J241=0; 
float J242=0; 
float J243=0; 
float J251=0; 
float J252=0; 
float J253=0;
float e1[][1]={{0},{0}};
float e2[][1]={{0},{0},{0}};
float M11[][2]={{0,0},{0,0}};
float M12[][3]={{0,0,0},{0,0,0}};
float M11d[][2]={{0,0},{0,0}};
float M12d[][3]={{0,0,0},{0,0,0}};
float M21[][2]={{0,0},{0,0},{0,0}};
float M22[][3]={{0,0,0},{0,0,0},{0,0,0}};
float M21d[][2]={{0,0},{0,0},{0,0}};
float M22d[][3]={{0,0,0},{0,0,0},{0,0,0}};
float niM11[][2]={{0,0},{0,0}};


float AJ1[2][3]={{0,0,0},{0,0,0}};
float AJ2[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float iM11[2][2]={{0,0},{0,0}};
float A[2][3]={{0,0,0},{0,0,0}};
float B[2][2]={{0,0},{0,0}};
float C[2][3]={{0,0,0},{0,0,0}};
float D[2][2]={{0,0},{0,0}};
float E[2][3]={{0,0,0},{0,0,0}};
float Mc[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float iMc[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float Md[3][2]={{0,0},{0,0},{0,0}};
float Mdd[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float Mddd[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float Mdddd[3][1]={{0},{0},{0}};
float omegasII[3][1]={{0},{0},{0}};
float thsII[3][1]={{0},{0},{0}};
float iMddd[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float Mcm[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float Mdm[3][2]={{0,0},{0,0},{0,0}};
float Mddm[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float Mdddm[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float Mddddm[3][3]={{0,0,0},{0,0,0},{0,0,0}};
float Mddddms[2][1]={{0},{0}};
float mu[3][1]={{0},{0},{0}};
float abc1[3][1]={{0},{0},{0}};
float abc2[3][1]={{0},{0},{0}};
float abc3[3][1]={{0},{0},{0}};
float abc4[3][1]={{0},{0},{0}};
float abc5[3][1]={{0},{0},{0}};
float abc6[3][1]={{0},{0},{0}};
float abc7[3][1]={{0},{0},{0}};
float abc8[3][1]={{0},{0},{0}};
float abc9[3][1]={{0},{0},{0}};
float abc10[3][1]={{0},{0},{0}};
float abc11[3][1]={{0},{0},{0}};
float abc12[3][1]={{0},{0},{0}};
float abc13[3][1]={{0},{0},{0}};
float abc14[3][1]={{0},{0},{0}};
float abc15[3][1]={{0},{0},{0}};
float abc16[3][1]={{0},{0},{0}};
float dzdt34[3][1]={{0},{0},{0}};
float abc17[2][1]={{0},{0}};
float abc18[2][1]={{0},{0}};
float abc19[2][1]={{0},{0}};
float abc20[2][1]={{0},{0}};
float abc21[2][1]={{0},{0}};
float abc22[2][1]={{0},{0}};
float abc23[2][1]={{0},{0}};
float abc24[2][1]={{0},{0}};
float abc25[2][1]={{0},{0}};
float abc26[2][1]={{0},{0}};
float abc27[2][1]={{0},{0}};
float dzdt24[2][1]={{0},{0}};
float dzdt[10][1];

float ab1[2][1]={{0},{0}};
float ab2[3][1]={{0},{0},{0}};
float tau[3][1]={{0},{0},{0}};
double pitch1=0, pitch2=0, pitch3=0;
double oldPos1  = 0;
double oldPos2  = 0;
double oldPos3  = 0;
double oldPosition1 = -999;
double oldPosition2 = -999;
double oldPosition3 = -999;
double newpos1, newpos2, newpos3, newtime = 0, newtime1 = 0, newtime2 = 0, newtime3 = 0, oldtime = 0, oldtime1 = 0, oldtime2 = 0, oldtime3 = 0, vel1, vel2, vel3, rpm1, rpm2, rpm3, rad_p_sec1, rad_p_sec2, rad_p_sec3, pos_rad1, pos_rad2, pos_rad3;
#define LED_PIN 13
int i=0;
typedef struct  {
    byte slaveID;
    byte cmdID;
    float myFloat1;
} RemoteSensorData_t;

typedef union float2bytes_t
{ 
  float f;
  char b[sizeof(float)]; 
}; 


float2bytes_t pitch={0};
float2bytes_t roll={0};
float2bytes_t yaw={0};
const byte addrSlaveI2C =  8;
bool blinkState = false;

void receiveEvent() 
{
  
    while(Wire.available())
    {
     for ( int i=0; i <sizeof(float); i++ )
     yaw.b[i]=Wire.read();
     for ( int j=0; j < sizeof(float); j++ )
     pitch.b[j]=Wire.read();
     for ( int k=0; k < sizeof(float); k++ )
     roll.b[k]=Wire.read();
    }

  newPosition1 = myEnc1.read();
  newtime11=(0.001*millis());
  newPosition2 = myEnc2.read();
  newtime12=(0.001*millis());
  newPosition3 = myEnc3.read();
  newtime13=(0.001*millis());
  
  pitch1 = (float)((float)(newPosition1)/4000)*360;
  pitch2 = (float)((float)(newPosition2)/4000)*360; 
  pitch3 = (float)((float)(newPosition3)/4000)*360;

  newpos1=pitch1;
  newpos2=pitch2;
  newpos3=pitch3;
  
  vel1=(newpos1-oldPos1)/(newtime11-oldtime11);
  vel2=(newpos2-oldPos2)/(newtime12-oldtime12);
  vel3=(newpos3-oldPos3)/(newtime13-oldtime13);
  
  oldPos1=newpos1;
  oldPos2=newpos2;
  oldPos3=newpos3;
  
  rpm1=vel1/6;
  rpm2=vel2/6;
  rpm3=vel3/6;
  
  rad_p_sec1 = rpm1 * 3.14/60;
  pos_rad1 = newpos1 * 3.14/360;

  rad_p_sec2 = rpm2 * 3.14/60;
  pos_rad2 = newpos2 * 3.14/360;

  rad_p_sec3 = rpm3 * 3.14/60;
  pos_rad3 = newpos3 * 3.14/360;
  psiz1=yaw.f;
  //psiz1=pi/12;
 newtime1 = (0.001 * millis());
 psiy1=pitch.f;
 //psiy1=pi/12;
 newtime2 = (0.001 * millis());
psix1=roll.f;
 //psix1=pi/12;
 newtime3 = (0.001 * millis());

 psizdot1=(psiz1-psiz_old)/(newtime1-oldtime1);
 psiz_old=psiz1;
 
 psiydot1=(psiy1-psiy_old)/(newtime2-oldtime2);
 psiy_old=psiy1;
 
 psixdot1=(psix1-psix_old)/(newtime3-oldtime3);
 psix_old=psix1;
 
 epx1=vxd-xsdot;
 newtime01=(0.001*millis());
 edx1=(epx1-epx_old)/(newtime01-oldtime01);
epy1=vyd-ysdot;
newtime02=(0.001*millis());
edy1=(epy1-epy_old)/(newtime02-oldtime02);
oldtime11 = newtime11;
oldtime12=newtime12;
oldtime13=newtime13;
oldPosition1 = newPosition1;
oldPosition2 = newPosition2;
oldPosition3 = newPosition3;


  if(!isnan(epx1))
    eix1=eix1+(epx1*(newtime01-oldtime01));
  if(!isnan(epy1))
     eiy1=eiy1+(epy1*(newtime02-oldtime02));
  
  oldtime1 = newtime1;
  oldtime2 = newtime2;
  oldtime3 = newtime3;
  oldtime01 = newtime01;
  oldtime02 = newtime02;
  epx_old=epx1;
  epy_old=epy1;
  Serial.println();
  Serial.print("HAHA");
}

void setup() {
  delay(45);
  Serial.begin(115200);
  Wire.begin(addrSlaveI2C);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  pinMode(LED_PIN, OUTPUT);
 pinMode (9, OUTPUT);
 pinMode (34, OUTPUT);
 pinMode (35, OUTPUT);
 pinMode (11, OUTPUT);
 pinMode (12, OUTPUT);
 pinMode (13, OUTPUT);
 pinMode (10, OUTPUT);
 pinMode (44, OUTPUT);
 pinMode (45, OUTPUT);
 }
 

void loop() {
  //BELOW NOT COPIED
  ctr=ctr+1;
  phi1=-pos_rad1;
  phi2=-pos_rad2;
  phi3=-pos_rad3;

  phi1dot=-rad_p_sec1;
  phi2dot=-rad_p_sec2;
  phi3dot=-rad_p_sec3;
  psiz=psiz1;
  psiy=-psiy1;
  psix=psix1;
  psizdot=psizdot1;
  psiydot=-psiydot1;
  psixdot=psixdot1;
  epx=epx1;
  epy=epy1;
  edx=edx1;
  edy=edy1;
  eix=eix1;
  eiy=eiy1;
//ABOVE NOT COPIED
    
 Serial.println();
 Serial.print("ypr =");
 Serial.print(psiz);   
 Serial.print("   ");
 Serial.print(psiy);   
 Serial.print("   ");
 Serial.print(psix);   
 Serial.print("   "); 
  Serial.print("yprd =");
 Serial.print(psizdot);   
 Serial.print("   ");
 Serial.print(psiydot);   
 Serial.print("   ");
 Serial.print(psixdot);   
 Serial.print("   "); 
  Serial.print("phi1 =");
  Serial.print(phi1);
   Serial.print("   ");
  Serial.print("phi2 =");
  Serial.print(phi2);
   Serial.print("   ");
  Serial.print("phi3 =");
  Serial.print(phi3);
   Serial.print("   ");
  Serial.print("phi1dot=");
  Serial.print(phi1dot);   
  Serial.print("   ");
  Serial.print("phi2dot=");
  Serial.print(phi2dot);   
  Serial.print("   ");  
  Serial.print("phi3dot");
  Serial.print(phi3dot); 



omegasII[0][0]=(cos(psix)*sin(psiz) + cos(psiz)*sin(psix)*sin(psiy))*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (sqrt(3)*rw*(phi2dot - phi3dot))/(3*rs*cos(alpha))) - (sin(psix)*sin(psiz) - cos(psix)*cos(psiz)*sin(psiy))*(psiydot*sin(psix) + psizdot*cos(psix)*cos(psiy) + (rw*(phi1dot + phi2dot + phi3dot))/(3*rs*sin(alpha))) - cos(psiy)*cos(psiz)*(psixdot + psizdot*sin(psiy) - (rw*(phi2dot - 2*phi1dot + phi3dot))/(3*rs*cos(alpha)));
omegasII[1][0]=(cos(psiz)*sin(psix) + cos(psix)*sin(psiy)*sin(psiz))*(psiydot*sin(psix) + psizdot*cos(psix)*cos(psiy) + (rw*(phi1dot + phi2dot + phi3dot))/(3*rs*sin(alpha))) - (cos(psix)*cos(psiz) - sin(psix)*sin(psiy)*sin(psiz))*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (sqrt(3)*rw*(phi2dot - phi3dot))/(3*rs*cos(alpha))) - cos(psiy)*sin(psiz)*(psixdot + psizdot*sin(psiy) - (rw*(phi2dot - 2*phi1dot + phi3dot))/(3*rs*cos(alpha)));
omegasII[2][0]=sin(psiy)*(psixdot + psizdot*sin(psiy) - (rw*(phi2dot - 2*phi1dot + phi3dot))/(3*rs*cos(alpha))) + cos(psix)*cos(psiy)*(psiydot*sin(psix) + psizdot*cos(psix)*cos(psiy) + (rw*(phi1dot + phi2dot + phi3dot))/(3*rs*sin(alpha))) + cos(psiy)*sin(psix)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (sqrt(3)*rw*(phi2dot - phi3dot))/(3*rs*cos(alpha)));
thsII[0][1]= (cos(psix)*sin(psiz) + cos(psiz)*sin(psix)*sin(psiy))*(psiz*cos(psiy)*sin(psix) - psiy*cos(psix) + (sqrt(3)*rw*(phi2 - phi3))/(3*rs*cos(alpha))) - (sin(psix)*sin(psiz) - cos(psix)*cos(psiz)*sin(psiy))*(psiy*sin(psix) + psiz*cos(psix)*cos(psiy) + (rw*(phi1 + phi2 + phi3))/(3*rs*sin(alpha))) - cos(psiy)*cos(psiz)*(psix + psiz*sin(psiy) - (rw*(phi2 - 2*phi1 + phi3))/(3*rs*cos(alpha)));
thsII[1][1]= (cos(psiz)*sin(psix) + cos(psix)*sin(psiy)*sin(psiz))*(psiy*sin(psix) + psiz*cos(psix)*cos(psiy) + (rw*(phi1 + phi2 + phi3))/(3*rs*sin(alpha))) - (cos(psix)*cos(psiz) - sin(psix)*sin(psiy)*sin(psiz))*(psiz*cos(psiy)*sin(psix) - psiy*cos(psix) + (sqrt(3)*rw*(phi2 - phi3))/(3*rs*cos(alpha))) - cos(psiy)*sin(psiz)*(psix + psiz*sin(psiy) - (rw*(phi2 - 2*phi1 + phi3))/(3*rs*cos(alpha)));
thsII[2][1]= sin(psiy)*(psix + psiz*sin(psiy) - (rw*(phi2 - 2*phi1 + phi3))/(3*rs*cos(alpha))) + cos(psix)*cos(psiy)*(psiy*sin(psix) + psiz*cos(psix)*cos(psiy) + (rw*(phi1 + phi2 + phi3))/(3*rs*sin(alpha))) + cos(psiy)*sin(psix)*(psiz*cos(psiy)*sin(psix) - psiy*cos(psix) + (sqrt(3)*rw*(phi2 - phi3))/(3*rs*cos(alpha)));
xs=rs*thsII[0][0];
ys=rs*thsII[1][0];
xsdot=rs*omegasII[0][0];
ysdot=rs*omegasII[1][0];
//DECLARATION OF CALCULATIONS
D111=0;
D112=0;
D113=l*mb*psixdot*sin(psix)*sin(psiz) - ((Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)))*(cos(alpha)*(sin(psix)*sin(psiz) + cos(psix)*cos(psiz)*sin(psiy)) + cos(psix)*cos(psiy)*sin(alpha)))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*((((pow(k,2))))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) - sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) + rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs))))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) + sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) - rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))))*(cos(beta)*(cos(alpha)*(sin(psix)*sin(psiz) + cos(psix)*cos(psiz)*sin(psiy)) - cos(beta)*cos(psix)*cos(psiy)*sin(alpha)) + sin(beta)*(cos(alpha)*(cos(psiz)*sin(psix) - cos(psix)*sin(psiy)*sin(psiz)) - cos(psix)*cos(psiy)*sin(alpha)*sin(beta))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))))*(cos(beta)*(cos(alpha)*(sin(psix)*sin(psiz) + cos(psix)*cos(psiz)*sin(psiy)) - cos(beta)*cos(psix)*cos(psiy)*sin(alpha)) - sin(beta)*(cos(alpha)*(cos(psiz)*sin(psix) - cos(psix)*sin(psiy)*sin(psiz)) + cos(psix)*cos(psiy)*sin(alpha)*sin(beta))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) + rs*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs))*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2)) - l*mb*psixdot*cos(psix)*cos(psiz)*sin(psiy);
D114=((Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix))*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) + rs*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(cos(alpha)*cos(psiy)*sin(psix)*sin(psiz) + sin(alpha)*sin(beta)*sin(psix)*sin(psiy)) + cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz)*sin(psix) + cos(beta)*sin(alpha)*sin(psix)*sin(psiy)))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(cos(alpha)*cos(psiy)*sin(psix)*sin(psiz) - sin(alpha)*sin(beta)*sin(psix)*sin(psiy)) - cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz)*sin(psix) + cos(beta)*sin(alpha)*sin(psix)*sin(psiy)))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) + rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) - cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy)))))/(pow(rw,2)) + ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) - rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) + cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy)))))/(pow(rw,2)) + ((sin(alpha)*sin(psix)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz)*sin(psix))*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy))))/(pow(rw,2)) - l*mb*psixdot*cos(psiy)*cos(psiz)*sin(psix);
D115=((Im*(((pow(k,2)))) + Iw)*(cos(alpha)*cos(beta)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(alpha)*sin(beta)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(cos(alpha)*cos(beta)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(alpha)*sin(beta)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) + cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2)) + ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) - cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2)) + (cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz))*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy))))/(pow(rw,2)) - l*mb*psixdot*cos(psix)*cos(psiz) + (cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix))*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)))/(pow(rw,2)) + l*mb*psixdot*sin(psix)*sin(psiy)*sin(psiz);
D116=mb + ms + Is/(pow(rs,2)) + ((Im*(((pow(k,2)))) + Iw)*(pow((cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)),2)))/(pow(rw,2)) + ((pow((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix))),2))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2)) + ((pow((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix))),2))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2));
D117=((Im*(((pow(k,2)))) + Iw)*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz))*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2));
D118=(rs*cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2)) - l*mb*cos(psix)*sin(psiz) - l*mb*cos(psiz)*sin(psix)*sin(psiy) + (rs*cos(alpha)*cos(beta)*(cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2)) + (rs*cos(alpha)*cos(beta)*(cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2));
D119=((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix)))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix))))/(pow(rw,2)) + (rs*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2));
D1110=((Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha))*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2)) + ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2));
D211=0;
D212=0;
D213=((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) + sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) - rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) - sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) + rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz))*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) + rs*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))/(pow(rw,2)) - l*mb*psixdot*cos(psiz)*sin(psix) - l*mb*psixdot*cos(psix)*sin(psiy)*sin(psiz);
D214=((Im*(((pow(k,2)))) + Iw)*(cos(psiy)*sin(alpha) + cos(alpha)*cos(psiz)*sin(psiy))*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) - rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) + cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) + rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) - cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz))*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) + rs*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))))*(cos(beta)*(cos(beta)*cos(psiy)*sin(alpha) - cos(alpha)*cos(psiz)*sin(psiy)) + sin(beta)*(cos(psiy)*sin(alpha)*sin(beta) + cos(alpha)*sin(psiy)*sin(psiz))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))))*(cos(beta)*(cos(beta)*cos(psiy)*sin(alpha) - cos(alpha)*cos(psiz)*sin(psiy)) + sin(beta)*(cos(psiy)*sin(alpha)*sin(beta) - cos(alpha)*sin(psiy)*sin(psiz))))/(pow(rw,2)) - l*mb*psixdot*cos(psiy)*sin(psix)*sin(psiz);
D215=((Im*(((pow(k,2)))) + Iw)*(cos(alpha)*cos(beta)*cos(psiy)*sin(psiz) + cos(alpha)*cos(psiy)*cos(psiz)*sin(beta))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) - ((cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) + cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - ((cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) - cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(cos(alpha)*cos(beta)*cos(psiy)*sin(psiz) - cos(alpha)*cos(psiy)*cos(psiz)*sin(beta))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + (cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz))*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)))/(pow(rw,2)) - l*mb*psixdot*cos(psix)*sin(psiz) - l*mb*psixdot*cos(psiz)*sin(psix)*sin(psiy) + (cos(alpha)*cos(psiy)*sin(psiz)*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy))))/(pow(rw,2));
D216=((Im*(((pow(k,2)))) + Iw)*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz))*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2));
D217=mb + ms + Is/(pow(rs,2)) + ((Im*(((pow(k,2)))) + Iw)*(pow((cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))),2)))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(pow((cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))),2)))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(pow((sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz)),2)))/(pow(rw,2));
D218=l*mb*cos(psix)*cos(psiz) + (rs*cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz)))/(pow(rw,2)) - l*mb*sin(psix)*sin(psiy)*sin(psiz) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2));
D219=((Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix)))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) + (rs*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz)))/(pow(rw,2));
D2110=((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha)))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha)))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha))*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz)))/(pow(rw,2));
D311=0;
D312=0;
D313=l*mb*xsdot*sin(psix)*sin(psiz) - l*mb*ysdot*cos(psiz)*sin(psix) - (rs*cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) + rs*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))/(pow(rw,2)) - l*mb*xsdot*cos(psix)*cos(psiz)*sin(psiy) - l*mb*ysdot*cos(psix)*sin(psiy)*sin(psiz) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) + sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) - rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs))))/(pow(rw,2)) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) - sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) + rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs))))/(pow(rw,2));
D314=Ibx*psizdot*cos(psiy) + (rs*cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) + rs*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))/(pow(rw,2)) - l*mb*xsdot*cos(psiy)*cos(psiz)*sin(psix) - l*mb*ysdot*cos(psiy)*sin(psix)*sin(psiz) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) + rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) - cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy)))))/(pow(rw,2)) + (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) - rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) + cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy)))))/(pow(rw,2));
D315=(rs*(pow((cos(alpha)),2))*(Im*(((pow(k,2)))) + Iw)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)))/(pow(rw,2)) - l*mb*ysdot*cos(psix)*sin(psiz) - l*mb*xsdot*cos(psix)*cos(psiz) - l*mb*ysdot*cos(psiz)*sin(psix)*sin(psiy) + l*mb*xsdot*sin(psix)*sin(psiy)*sin(psiz) + (rs*cos(alpha)*cos(beta)*(cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) + cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2)) + (rs*cos(alpha)*cos(beta)*(cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) - cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2));
D316=(rs*cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2)) - l*mb*cos(psix)*sin(psiz) - l*mb*cos(psiz)*sin(psix)*sin(psiy) + (rs*cos(alpha)*cos(beta)*(cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2)) + (rs*cos(alpha)*cos(beta)*(cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2));
D317=l*mb*cos(psix)*cos(psiz) + (rs*cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz)))/(pow(rw,2)) - l*mb*sin(psix)*sin(psiy)*sin(psiz) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2));
D318=Ibx + (pow(l,2))*mb + ((pow(rs,2))*(pow((cos(alpha)),2))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2)) + (2*(pow(rs,2))*(pow((cos(alpha)),2))*(pow((cos(beta)),2))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2));
D319=((pow(rs,2))*cos(alpha)*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2)) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix))))/(pow(rw,2)) + (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix)))/(pow(rw,2));
D3110=Ibx*sin(psiy) + (rs*cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha)))/(pow(rw,2)) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) + (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2));
D411=0;
D412=0;
D413=((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) - sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) + rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) + sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) - rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix)))/(pow(rw,2)) - Iby*psizdot*(pow((cos(psix)),2))*cos(psiy) + Ibz*psizdot*(pow((cos(psix)),2))*cos(psiy) + Iby*psizdot*cos(psiy)*(pow((sin(psix)),2)) - Ibz*psizdot*cos(psiy)*(pow((sin(psix)),2)) - ((Im*(((pow(k,2)))) + Iw)*(rs*cos(psix)*sin(alpha)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*sin(psix) + rs*cos(psix)*sin(alpha)*sin(beta)))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*sin(psix) - rs*cos(psix)*sin(alpha)*sin(beta)) - rs*(pow((cos(beta)),2))*cos(psix)*sin(alpha))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) - 2*Iby*psiydot*cos(psix)*sin(psix) + 2*Ibz*psiydot*cos(psix)*sin(psix) + (rs*cos(psix)*sin(alpha)*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy))))/(pow(rw,2)) - (rs*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) + rs*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))/(pow(rw,2));
D414=((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) + rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) - cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) - rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) + cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix)))/(pow(rw,2)) + Iby*psizdot*cos(psix)*sin(psix)*sin(psiy) - Ibz*psizdot*cos(psix)*sin(psix)*sin(psiy) + (rs*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) + rs*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))/(pow(rw,2));
D415=((cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) - cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix)))/(pow(rw,2)) - ((cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) + cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix))))/(pow(rw,2)) + (rs*cos(alpha)*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)))/(pow(rw,2));
D416=((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix)))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix))))/(pow(rw,2)) + (rs*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2));
D417=((Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix)))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) + (rs*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz)))/(pow(rw,2));
D418=((pow(rs,2))*cos(alpha)*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2)) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix))))/(pow(rw,2)) + (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix)))/(pow(rw,2));
D419=Ibz + Iby*(pow((cos(psix)),2)) - Ibz*(pow((cos(psix)),2)) + ((Im*(((pow(k,2)))) + Iw)*(pow((rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix))),2)))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(pow((sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix)),2)))/(pow(rw,2)) + ((pow(rs,2))*(pow((sin(alpha)),2))*(pow((sin(psix)),2))*(Im*(((pow(k,2)))) + Iw))/(pow(rw,2));
D4110=Ibz*cos(psix)*cos(psiy)*sin(psix) - Iby*cos(psix)*cos(psiy)*sin(psix) - ((Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix)))*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix))*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) + (rs*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha)))/(pow(rw,2));
D511=0;
D512=0;
D513=((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) + sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) - rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) - sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) + rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) - Iby*psiydot*(pow((cos(psix)),2))*cos(psiy) + Ibz*psiydot*(pow((cos(psix)),2))*cos(psiy) + Iby*psiydot*cos(psiy)*(pow((sin(psix)),2)) - Ibz*psiydot*cos(psiy)*(pow((sin(psix)),2)) + ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))))*(sin(beta)*(rs*cos(alpha)*cos(psix)*cos(psiy) + rs*cos(psiy)*sin(alpha)*sin(beta)*sin(psix)) + rs*(pow((cos(beta)),2))*cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))))*(sin(beta)*(rs*cos(alpha)*cos(psix)*cos(psiy) - rs*cos(psiy)*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha))*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) + rs*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))/(pow(rw,2)) + 2*Iby*psizdot*cos(psix)*(pow((cos(psiy)),2))*sin(psix) - 2*Ibz*psizdot*cos(psix)*(pow((cos(psiy)),2))*sin(psix) - (rs*cos(psiy)*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy))))/(pow(rw,2));
D514=Ibx*psixdot*cos(psiy) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) + rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) - cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) - rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) + cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*cos(psiy) - rs*cos(psix)*sin(alpha)*sin(psiy))*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha))*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) + rs*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*cos(psiy) + rs*cos(beta)*cos(psix)*sin(alpha)*sin(psiy)) + sin(beta)*(rs*cos(alpha)*sin(psix)*sin(psiy) + rs*cos(psix)*sin(alpha)*sin(beta)*sin(psiy)))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*cos(psiy) + rs*cos(beta)*cos(psix)*sin(alpha)*sin(psiy)) - sin(beta)*(rs*cos(alpha)*sin(psix)*sin(psiy) - rs*cos(psix)*sin(alpha)*sin(beta)*sin(psiy)))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + 2*Ibx*psizdot*cos(psiy)*sin(psiy) - 2*Iby*psizdot*cos(psiy)*sin(psiy) + 2*Iby*psizdot*(pow((cos(psix)),2))*cos(psiy)*sin(psiy) - 2*Ibz*psizdot*(pow((cos(psix)),2))*cos(psiy)*sin(psiy) + Iby*psiydot*cos(psix)*sin(psix)*sin(psiy) - Ibz*psiydot*cos(psix)*sin(psix)*sin(psiy);
D515=((cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) + cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) - ((cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) - cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) + (cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha))*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)))/(pow(rw,2));
D516=((Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha))*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) - cos(psiy)*sin(alpha)*sin(psix)))/(pow(rw,2)) + ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) + sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) - ((cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) - cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)) - sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) + sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2));
D517=((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha)))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) + sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) + sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha)))*(cos(beta)*(cos(alpha)*cos(psiy)*cos(psiz) + cos(beta)*sin(alpha)*sin(psiy)) - sin(beta)*(cos(alpha)*cos(psiy)*sin(psiz) - sin(alpha)*sin(beta)*sin(psiy))))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha))*(sin(alpha)*sin(psiy) - cos(alpha)*cos(psiy)*cos(psiz)))/(pow(rw,2));
D518=Ibx*sin(psiy) + (rs*cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha)))/(pow(rw,2)) - (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) + (rs*cos(alpha)*cos(beta)*(Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2));
D519=Ibz*cos(psix)*cos(psiy)*sin(psix) - Iby*cos(psix)*cos(psiy)*sin(psix) - ((Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*sin(psix)*(pow((cos(beta)),2)) + sin(beta)*(rs*cos(alpha)*cos(psix) + rs*sin(alpha)*sin(beta)*sin(psix)))*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*cos(psix) - rs*sin(alpha)*sin(beta)*sin(psix)) - rs*(pow((cos(beta)),2))*sin(alpha)*sin(psix))*(sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))))/(pow(rw,2)) + (rs*sin(alpha)*sin(psix)*(Im*(((pow(k,2)))) + Iw)*(rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha)))/(pow(rw,2));
D5110=Ibx - Ibx*(pow((cos(psiy)),2)) + Iby*(pow((cos(psiy)),2)) - Iby*(pow((cos(psix)),2))*(pow((cos(psiy)),2)) + Ibz*(pow((cos(psix)),2))*(pow((cos(psiy)),2)) + ((Im*(((pow(k,2)))) + Iw)*(pow((sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) + rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) - cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))),2)))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(pow((sin(beta)*(rs*cos(alpha)*cos(psiy)*sin(psix) - rs*cos(psix)*cos(psiy)*sin(alpha)*sin(beta)) + cos(beta)*(rs*cos(alpha)*sin(psiy) - rs*cos(beta)*cos(psix)*cos(psiy)*sin(alpha))),2)))/(pow(rw,2)) + ((Im*(((pow(k,2)))) + Iw)*(pow((rs*cos(alpha)*sin(psiy) + rs*cos(psix)*cos(psiy)*sin(alpha)),2)))/(pow(rw,2));
e11=-b*xsdot;
e21=-b*ysdot;
e31=Ibz*(pow(psiydot,2))*cos(psix)*sin(psix) - Iby*(pow(psiydot,2))*cos(psix)*sin(psix) - ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) + sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) - rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(beta)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)) - sin(beta)*(rs*cos(alpha)*(psiydot*sin(psix) - (xsdot*cos(psiz)*sin(psix) - xsdot*cos(psix)*sin(psiy)*sin(psiz))/rs + psizdot*cos(psix)*cos(psiy)) + rs*sin(alpha)*sin(beta)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs)))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(cos(alpha)*(xsdot*sin(psix)*sin(psiz) + xsdot*cos(psix)*cos(psiz)*sin(psiy)) + rs*sin(alpha)*(psizdot*cos(psiy)*sin(psix) - psiydot*cos(psix) + (xsdot*cos(psix)*cos(psiy))/rs))*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy))))/(pow(rw,2)) + g*l*mb*cos(psiy)*sin(psix) + Iby*(pow(psizdot,2))*cos(psix)*(pow((cos(psiy)),2))*sin(psix) - Ibz*(pow(psizdot,2))*cos(psix)*(pow((cos(psiy)),2))*sin(psix) - Iby*psiydot*psizdot*(pow((cos(psix)),2))*cos(psiy) + Ibz*psiydot*psizdot*(pow((cos(psix)),2))*cos(psiy) + Iby*psiydot*psizdot*cos(psiy)*(pow((sin(psix)),2)) - Ibz*psiydot*psizdot*cos(psiy)*(pow((sin(psix)),2)) - l*mb*psixdot*ysdot*cos(psiz)*sin(psix) + l*mb*psixdot*xsdot*sin(psix)*sin(psiz) - l*mb*psixdot*xsdot*cos(psix)*cos(psiz)*sin(psiy) - l*mb*psixdot*ysdot*cos(psix)*sin(psiy)*sin(psiz);
e41=((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) - rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) + cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) - ((Im*(((pow(k,2)))) + Iw)*(sin(beta)*(rs*cos(alpha)*((ysdot*sin(psiy)*sin(psiz) - xsdot*cos(psiy)*sin(psix)*sin(psiz))/rs + psizdot*sin(psix)*sin(psiy)) + rs*sin(alpha)*sin(beta)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))) - cos(beta)*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) - rs*cos(beta)*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + Ibx*(pow(psizdot,2))*cos(psiy)*sin(psiy) - Iby*(pow(psizdot,2))*cos(psiy)*sin(psiy) + Ibx*psixdot*psizdot*cos(psiy) + ((Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)))*(rs*cos(alpha)*((ysdot*cos(psiz)*sin(psiy) - xsdot*cos(psiy)*cos(psiz)*sin(psix))/rs + psizdot*cos(psiy)) + rs*sin(alpha)*((ysdot*cos(psiy) + xsdot*sin(psix)*sin(psiy))/rs - psizdot*cos(psix)*sin(psiy))))/(pow(rw,2)) + g*l*mb*cos(psix)*sin(psiy) + Iby*(pow(psizdot,2))*(pow((cos(psix)),2))*cos(psiy)*sin(psiy) - Ibz*(pow(psizdot,2))*(pow((cos(psix)),2))*cos(psiy)*sin(psiy) + Iby*psiydot*psizdot*cos(psix)*sin(psix)*sin(psiy) - Ibz*psiydot*psizdot*cos(psix)*sin(psix)*sin(psiy) - l*mb*psixdot*xsdot*cos(psiy)*cos(psiz)*sin(psix) - l*mb*psixdot*ysdot*cos(psiy)*sin(psix)*sin(psiz);
e51=((cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) + cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + ((cos(alpha)*cos(beta)*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)) - cos(alpha)*sin(beta)*(ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy)))*(Im*(((pow(k,2)))) + Iw)*(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)))))/(pow(rw,2)) + (cos(alpha)*(Im*(((pow(k,2)))) + Iw)*(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)))*(xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz)))/(pow(rw,2)) - l*mb*psixdot*xsdot*cos(psix)*cos(psiz) - l*mb*psixdot*ysdot*cos(psix)*sin(psiz) - l*mb*psixdot*ysdot*cos(psiz)*sin(psix)*sin(psiy) + l*mb*psixdot*xsdot*sin(psix)*sin(psiy)*sin(psiz);


e1[0][0]=e11;
e1[1][0]=e21;
e2[0][0]=e31;
e2[1][0]=e41;
e1[2][0]=e51;
M11[0][0]=D116;
M11[0][1]=D117;
M11[1][0]=D216;
M11[1][1]=D217;
M12[0][0]=D118;
M12[0][1]=D119;
M12[0][2]=D1110;
M12[1][0]=D218;
M12[1][1]=D219;
M12[1][2]=D2110;
M11d[0][0]=D111;
M11d[0][1]=D112;
M11d[1][0]=D211;
M11d[1][1]=D212;
M12d[0][0]=D113;
M12d[0][1]=D114;
M12d[0][2]=D115;
M12d[1][0]=D213;
M12d[1][1]=D214;
M12d[1][2]=D215;

M21[0][0]=D316;
M21[0][1]=D317;
M21[1][0]=D416;
M21[1][1]=D417;
M21[2][0]=D516;
M21[2][1]=D517;



M22[0][0]=D318;
M22[0][1]=D319;
M22[0][2]=D3110;
M22[1][0]=D418;
M22[1][1]=D419;
M22[1][2]=D4110;
M22[2][0]=D518;
M22[2][1]=D519;
M22[2][2]=D5110;



M21d[0][0]=D311;
M21d[0][1]=D312;
M21d[1][0]=D411;
M21d[1][1]=D412;
M21d[2][0]=D511;
M21d[2][1]=D512;



M22d[0][0]=D313;
M22d[0][1]=D314;
M22d[0][2]=D315;
M22d[1][0]=D413;
M22d[1][1]=D414;
M22d[1][2]=D415;
M22d[2][0]=D513;
M22d[2][1]=D514;
M22d[2][2]=D515;
J211=(cos(psiy)*sin(alpha)*sin(psix))/rw - (cos(alpha)*(cos(psix)*sin(psiz) + cos(psiz)*sin(psix)*sin(psiy)))/rw;
J212=(cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) + cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)))/rw - (sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) - sin(psix)*sin(psiy)*sin(psiz)) - cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))/rw;
J213=(cos(beta)*(cos(alpha)*(cos(psix)*sin(psiz) + cos(psiz)*sin(psix)*sin(psiy)) + cos(beta)*cos(psiy)*sin(alpha)*sin(psix)))/rw + (sin(beta)*(cos(alpha)*(cos(psix)*cos(psiz) - sin(psix)*sin(psiy)*sin(psiz)) + cos(psiy)*sin(alpha)*sin(beta)*sin(psix)))/rw;
J221=(sin(alpha)*sin(psiy) + cos(alpha)*cos(psiy)*cos(psiz))/rw;
J222=-(cos(alpha)*cos(beta)*cos(psiy)*cos(psiz) - sin(alpha)*sin(psiy) + cos(alpha)*cos(psiy)*sin(beta)*sin(psiz))/rw;
J223=(sin(alpha)*sin(psiy) - cos(alpha)*cos(beta)*cos(psiy)*cos(psiz) + cos(alpha)*cos(psiy)*sin(beta)*sin(psiz))/rw;
J231=-(rs*cos(alpha))/rw;
J232=(rs*cos(alpha)*cos(beta))/rw;
J233=(rs*cos(alpha)*cos(beta))/rw;
J241=-(rs*sin(alpha)*sin(psix))/rw;
J242=-(rs*(sin(alpha)*sin(psix) - cos(alpha)*cos(psix)*sin(beta)))/rw;
J243=-(rs*(sin(alpha)*sin(psix) + cos(alpha)*cos(psix)*sin(beta)))/rw;
J251=(rs*(cos(alpha)*sin(psiy) - cos(psix)*cos(psiy)*sin(alpha)))/rw;
J252=-(rs*(cos(alpha)*cos(beta)*sin(psiy) + cos(psix)*cos(psiy)*sin(alpha) + cos(alpha)*cos(psiy)*sin(beta)*sin(psix)))/rw;
J253=-(rs*(cos(alpha)*cos(beta)*sin(psiy) + cos(psix)*cos(psiy)*sin(alpha) - cos(alpha)*cos(psiy)*sin(beta)*sin(psix)))/rw;

AJ1[0][0]=J211;
AJ1[0][1]=J212;
AJ1[0][2]=J213;
AJ1[1][0]=J221;
AJ1[1][1]=J222;
AJ1[1][2]=J223;
AJ2[0][0]=J231;
AJ2[0][1]=J232;
AJ2[0][2]=J233;
AJ2[1][0]=J241;
AJ2[1][1]=J242;
AJ2[1][2]=J243;
AJ2[2][0]=J251;
AJ2[2][1]=J252;
AJ2[2][2]=J253;

for(int m1=0;m1<=1;m1++)
{
  for(int m2=0;m2<=1;m2++)
  {
   iM11[m1][m2]=M11[m1][m2]; 
  }
}

Matrix.Invert((float*)iM11,2);
for(int m1=0;m1<=1;m1++)
{
  for(int m2=0;m2<=1;m2++)
  {
   niM11[m1][m2]=-iM11[m1][m2]; 
  }
}
Matrix.Multiply((float*)niM11, (float*)M12, 2, 2, 3, (float*)A);

Matrix.Multiply((float*)niM11, (float*)M11d, 2, 2,2 , (float*)B);

Matrix.Multiply((float*)niM11, (float*)M12d, 2, 2, 3, (float*)C);

for(int m1=0;m1<=1;m1++)
{
  for(int m2=0;m2<=1;m2++)
  {
   D[m1][m2]=iM11[m1][m2]; 
  }
}
Matrix.Multiply((float*)iM11, (float*)AJ1, 2, 2, 3, (float*)E);

Matrix.Multiply((float*)M21, (float*)A, 3, 2, 3, (float*)Mcm);
Matrix.Add((float*) Mcm, (float*) M22, 3, 3, (float*) Mc);

Matrix.Multiply((float*)M21, (float*)B, 3, 2, 2, (float*)Mdm);
Matrix.Add((float*) Mdm, (float*) M21d, 3, 2, (float*) Md);

Matrix.Multiply((float*)M21, (float*)C, 3, 2, 3, (float*)Mddm);
Matrix.Add((float*) Mddm, (float*) M22d, 3, 3, (float*) Mdd);

Matrix.Multiply((float*)M21, (float*)E, 3, 2, 3, (float*)Mdddm);

for (int i=0; i<3; i++)
 {
  for(int j=0; j<3; j++)
   {
     Mdddm[i][j] = -Mdddm[i][j];
   }
 }

Matrix.Add((float*) AJ2, (float*) Mdddm, 3, 3, (float*) Mddd);




Matrix.Multiply((float*)D, (float*)e1, 2, 2, 1, (float*)Mddddms);
Matrix.Multiply((float*)M21, (float*)Mddddms, 3, 2, 1, (float*)Mddddm);
for (int i=0; i<3; i++)
 {
  for(int j=0; j<3; j++)
   {
     Mddddm[i][j] = -Mddddm[i][j];
   }
 }
Matrix.Add((float*) e2, (float*) Mddddm, 3, 1, (float*) Mdddd);


for(int m1=0;m1<=2;m1++)
{
  for(int m2=0;m2<=2;m2++)
  {
   iMddd[m1][m2]=Mddd[m1][m2]; 
  }
}

Matrix.Invert((float*)iMddd, 3);


mu1=-2*lam1*psixdot-(lam1*lam1*(psix-(kpy*epy+kiy*eiy+kdy*edy)));
mu2=-2*lam2*psiydot-(lam2*lam2*(psiy-(kpx*epx+kix*eix+kdx*edx)));

mu3=-2*lam3*psizdot-(lam3*lam3*psiz);
//mu3=-2*lam3*psizdot;

mu[0][0]=mu1;
mu[1][0]=mu2;
mu[2][0]=mu3;
ab1[0][0]=xsdot;
ab1[0][1]=ysdot;

ab2[0][0]=psixdot;
ab2[0][1]=psiydot;
ab2[0][2]=psizdot;

Matrix.Multiply((float*)Mc, (float*)mu, 3, 3, 1, (float*)abc1);
Matrix.Multiply((float*)Md, (float*)ab1, 3, 2, 1, (float*)abc2);
Matrix.Multiply((float*)Mdd, (float*)ab2, 3, 3, 1, (float*)abc3);
Matrix.Add((float*)abc1, (float*)abc2, 3, 1, (float*)abc4);
Matrix.Add((float*)abc4, (float*)abc3, 3, 1, (float*)abc5);
for(int q=0;q<3;q++)
{
  abc6[q][0]=-1*Mdddd[q][0];
}
Matrix.Add((float*)abc5, (float*)abc6, 3, 1, (float*)abc7);
Matrix.Multiply((float*)iMddd, (float*)abc7, 3, 3, 1, (float*)abc8);
for(int q=0;q<3;q++)
{
  tau[q][0]=abc8[q][0];
}
Serial.println();
for(int q=0;q<3;q++)
{
  Serial.print("Tau =");
  Serial.print(tau[q][0]);
}

for(int p=0;p<3;p++)
{
for(int q=0;q<3;q++)
{
  iMc[p][q]=Mc[p][q];
}
}
Matrix.Invert((float*)iMc, 3);
Matrix.Multiply((float*)Mddd, (float*)tau, 3, 3, 1, (float*)abc9);
Matrix.Multiply((float*)Mdd, (float*)ab2, 3, 3, 1, (float*)abc10);
Matrix.Multiply((float*)Md, (float*)ab1, 3, 3, 1, (float*)abc11);
for(int q=0;q<3;q++)
{
  abc12[q][0]=-abc10[q][0];
}
for(int q=0;q<3;q++)
{
  abc13[q][0]=-abc11[q][0];
}
Matrix.Add((float*)abc12, (float*)abc13, 3, 1, (float*)abc14);
Matrix.Add((float*)abc9, (float*)abc14, 3, 1, (float*)abc15);
Matrix.Add((float*)abc15, (float*)Mdddd, 3, 1, (float*)abc16);
Matrix.Multiply((float*)iMc, (float*)abc16, 3, 3, 1, (float*)dzdt34);

Matrix.Multiply((float*)M12, (float*)dzdt34, 2, 3, 1, (float*)abc17);
Matrix.Multiply((float*)M11d, (float*)ab1, 2, 2, 1, (float*)abc18);
Matrix.Multiply((float*)M12d, (float*)ab2, 2, 3, 1, (float*)abc19);
Matrix.Multiply((float*)AJ1, (float*)tau, 2, 3, 1, (float*)abc20);
for(int q=0;q<2;q++)

{
  abc21[q][0]=-abc17[q][0];
}
for(int q=0;q<2;q++)
{
  abc22[q][0]=-abc18[q][0];
}
for(int q=0;q<2;q++)
{
  abc23[q][0]=-abc19[q][0];
}
Matrix.Add((float*)abc20, (float*)e1, 2, 1, (float*)abc24);
Matrix.Add((float*)abc24, (float*)abc23, 2, 1, (float*)abc25);
Matrix.Add((float*)abc25, (float*)abc22, 2, 1, (float*)abc26);
Matrix.Add((float*)abc26, (float*)abc21, 2, 1, (float*)abc27);
Matrix.Multiply((float*)iM11,(float*)abc27,2, 2, 1, (float*)dzdt24);
dzdt[0][0]=xsdot;
dzdt[1][0]=dzdt24[0][0];
dzdt[2][0]=ysdot;
dzdt[3][0]=dzdt24[1][0];
dzdt[4][0]=psixdot;
dzdt[5][0]=dzdt34[0][0];
dzdt[6][0]=psiydot;
dzdt[7][0]=dzdt34[1][0];
dzdt[8][0]=psizdot;
dzdt[9][0]=dzdt34[2][0];

Serial.println();
Serial.print("dzdt0 =");
Serial.print(dzdt[0][0]);
Serial.print("dzdt0 =");
Serial.print(dzdt[1][0]);
Serial.print("dzdt0 =");
Serial.print(dzdt[2][0]);
Serial.print("dzdt0 =");
Serial.print(dzdt[3][0]);
Serial.print("dzdt0 =");
Serial.print(dzdt[4][0]);
Serial.print("dzdt0 =");
Serial.print(dzdt[5][0]);
Serial.print("dzdt0 =");
Serial.print(dzdt[6][0]);
Serial.print("dzdt0 =");
Serial.print(dzdt[7][0]);
Serial.print("dzdt0 =");
Serial.print(dzdt[8][0]);
Serial.print("dzdt0 =");
Serial.print(dzdt[9][0]);

xs=xs+(newtimec-oldtimec)*dzdt[0][0];
xsdot=xsdot+(newtimec-oldtimec)*dzdt[1][0];
ys=ys+(newtimec-oldtimec)*dzdt[2][0];
ysdot=ysdot+(newtimec-oldtimec)*dzdt[3][0];
psix=psix+(newtimec-oldtimec)*dzdt[4][0];
psixdot=psixdot+(newtimec-oldtimec)*dzdt[5][0];
psiy=psiy+(newtimec-oldtimec)*dzdt[6][0];
psiydot=psiydot+(newtimec-oldtimec)*dzdt[7][0];
psiz=psiz+(newtimec-oldtimec)*dzdt[8][0];
psizdot=psizdot+(newtimec-oldtimec)*dzdt[9][0];

phi1ds=-(rs*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy)) + rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)))/rw;
phi2ds=(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) - sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) + rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))))/rw;
phi3ds=(cos(beta)*(rs*cos(alpha)*(psixdot - (ysdot*cos(psiy)*cos(psiz) - xsdot*cos(psix)*sin(psiz) + xsdot*cos(psiz)*sin(psix)*sin(psiy))/rs + psizdot*sin(psiy)) - rs*cos(beta)*sin(alpha)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))) + sin(beta)*(rs*cos(alpha)*((xsdot*cos(psix)*cos(psiz) + ysdot*cos(psiy)*sin(psiz) + xsdot*sin(psix)*sin(psiy)*sin(psiz))/rs - psiydot*cos(psix) + psizdot*cos(psiy)*sin(psix)) - rs*sin(alpha)*sin(beta)*(psiydot*sin(psix) + (ysdot*sin(psiy) - xsdot*cos(psiy)*sin(psix))/rs + psizdot*cos(psix)*cos(psiy))))/rw;
Serial.println();
Serial.print("phi1ds =");
Serial.print(phi1ds);
Serial.print("phi2ds =");
Serial.print(phi2ds);
Serial.print("phi1ds =");
Serial.print(phi3ds);
//phi1ds=3.141*sin(1.59*.001*millis());
//phi2ds=3.141*sin(1.59*.001*millis());
//phi3ds=3.141*sin(1.59*.001*millis());


ep1=phi1ds-phi1dot;
ep2=phi2ds-phi2dot;
ep3=phi3ds-phi3dot;
Serial.println();
Serial.print("ep1=");
Serial.print(ep1);
Serial.print("ep2=");
Serial.print(ep2);
Serial.print("ep3=");
Serial.print(ep3);

p1={ep1,0.001*millis()};
p2={ep2,0.001*millis()};
p3={ep3,0.001*millis()};
pwm1f=kp*ep1+ki*ei1+kd*ed1;
pwm2f=kp*ep2+ki*ei2+kd*ed1;
pwm3f=kp*ep3+ki*ei3+kd*ed1;
oldtimec=newtimec;
newtimec=0.001*millis();
pwm1=(int)pwm1f;
pwm2=(int)pwm2f;
pwm3=(int)pwm3f;




if (pwm1 >= 0)
 {
   analogWrite (9, pwm1);
   digitalWrite(35, LOW);
   digitalWrite(34, HIGH);
 }
  else
  {
    pwm1 = -1 * pwm1;
    analogWrite (9, pwm1);
    digitalWrite(34, LOW);
    digitalWrite(35, HIGH);
  }
   if (pwm2 >= 0)
 {
  analogWrite (11, pwm2);
  digitalWrite(13, LOW);
  digitalWrite(12, HIGH);
 }
  else
  {
    pwm2 = -1 * pwm2;
    analogWrite (11, pwm2);
    digitalWrite(12, LOW);
    digitalWrite(13, HIGH);
  }
   if (pwm3 >= 0)
 {
   analogWrite (10, pwm3);
   digitalWrite(45, LOW);
   digitalWrite(44, HIGH);

 }
  else
  {
    pwm3 = -1 * pwm3;
    analogWrite (10, pwm3);
    digitalWrite(44, LOW);
    digitalWrite(45, HIGH);
  }

Serial.println();
Serial.print("ctr=");
Serial.print("   ");
Serial.print(ctr);
ed1=(p1.ep-p1_old.ep)/(p1.tim-p1_old.tim);
ed2=(p2.ep-p2_old.ep)/(p2.tim-p2_old.tim);
ed3=(p3.ep-p3_old.ep)/(p3.tim-p3_old.tim);
if(!isnan(p1.ep))
 ei1=ei1+((p1.ep)*(p1.tim-p1_old.tim));
 if(!isnan(p2.ep))
 ei2=ei2+((p2.ep)*(p2.tim-p2_old.tim));
 if(!isnan(p3.ep))
 ei3=ei3+((p3.ep)*(p3.tim-p3_old.tim));
 //DECLARATION OF NEW VARIABLES HERE PLEASE
 p1_old.ep=p1.ep;
 p2_old.ep=p2.ep;
 p3_old.ep=p3.ep;
 p1_old.tim=p1.tim;
 p2_old.tim=p2.tim;
 p3_old.tim=p3.tim;  
 Serial.println();
 Serial.print(pwm1f);
 Serial.println();
 Serial.print(pwm2f);

Serial.println();
 Serial.print(pwm3f);
   /*  Matrix.Print((float*)M11, 2, 2, "M11");
    Matrix.Print((float*)M12, 2, 3, "M12");
    Matrix.Print((float*)M21, 3, 2, "M21");
    Matrix.Print((float*)M22, 3, 3, "M22");
    Matrix.Print((float*)M11d, 2, 2, "M11d");
    Matrix.Print((float*)M12d, 2, 3, "M12d");
    Matrix.Print((float*)M21d, 3, 2, "M21d");
    Matrix.Print((float*)M22d, 3, 3, "M22d");
        Matrix.Print((float*)e1, 2, 1, "e1");
    Matrix.Print((float*)e2, 3, 1, "e2");
        Matrix.Print((float*)AJ1, 2, 3, "AJ1");
            Matrix.Print((float*)AJ2, 3, 3, "AJ2");
              Matrix.Print((float*)dzdt24, 2, 1, "dzdt24");
                Matrix.Print((float*)dzdt34, 3, 1, "dzdt34");
 Matrix.Print((float*)mu, 3, 1, "mu");
  Matrix.Print((float*)Mdddd, 3, 1, "Mdddd");   
  Matrix.Print((float*)Mddd, 3,3, "Mddd");
Matrix.Print((float*)Mdd, 3, 3, "Mdd");
Matrix.Print((float*)Md, 3, 2, "Md");
  Matrix.Print((float*)iMddd, 3, 3, "iMddd");*/
} 




