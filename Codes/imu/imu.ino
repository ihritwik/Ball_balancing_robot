#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#define RAD_TO_DEG 57.29578
unsigned long current_time = 0;
unsigned long last_time = 0;
double dt = 0;
byte calibration_reset = 0;
int calibration_reps = 1000;
double roll_calibration_sum =0, pitch_calibration_sum=0, yaw_calibration_sum=0;
double roll_offset=0, pitch_offset=0, yaw_offset=0;
double accel_roll=0, accel_pitch =0, accel_yaw = 0;
double last_filtered_roll=0, last_filtered_pitch=0, last_filtered_yaw=0;
double filtered_roll =0, filtered_pitch=0, filtered_yaw=0;
double alpha;
double time_constant = 1.00; //[Seconds]

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(30303);

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  Serial.println("Entered Function");
  //if(!accel.begin())
  //{
  // /* There was a problem detecting the LSM303 ... check your connections */
  //  Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
  //  while(1);
  //}
  accel.begin();
  Serial.println("Accelerometer done");
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  Serial.println("Magnetometer done");
 // if(!gyro.begin())
 // {
 //   /* There was a problem detecting the L3GD20 ... check your connections */
 //   Serial.println("Ooops, no L3GD20H detected ... Check your wiring!");
  //  while(1);
 // }
  //gyro.begin();
  //Serial.println("Gyro done");
  Serial.println("Exit from function");
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); Serial.println("");
  /* Initialise the sensors */
  //pinMode(13, OUTPUT);
  gyro.enableAutoRange(true);
  Serial.println("Level 2");
  initSensors();
  Serial.println("setup done");
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{

  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;
  Serial.println("Loop entered");
  //gyroscope stuff
  //sensors_event_t gyro_event; 
  //gyro.getEvent(&gyro_event);
  //Serial.println("gyro event");
  /* Calculate pitch and roll from the raw accelerometer data */
  digitalWrite(13, HIGH);
  accel.getEvent(&accel_event);
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  /* Calculating Gyroscope Information */
  if( calibration_reset == 0)
  {
    for (int i =0; i<calibration_reps; i++)
    {
      roll_calibration_sum += orientation.roll;
      pitch_calibration_sum += orientation.pitch;
      yaw_calibration_sum += orientation.heading;
    }
    roll_offset = roll_calibration_sum/(double)calibration_reps;
    pitch_offset = pitch_calibration_sum/(double)calibration_reps;
    yaw_offset = yaw_calibration_sum/(double)calibration_reps;
    calibration_reset = 1;
  }
  else
  {
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
      accel_roll = orientation.roll - roll_offset-0.9945;
      accel_pitch = orientation.pitch - pitch_offset+6.1149;
    }
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
      accel_yaw = orientation.heading - yaw_offset+0.1344; 
//The values 0.9945, -6.1149 and -0.1344 are additional offsets I had to introduce after I let my calibration code run with a stationary sensor and noticed deviation from 0
    }
    current_time = millis();
    dt = current_time-last_time; 
    
    //Complementary Filter   
    alpha = time_constant/(time_constant + dt);
    filtered_roll = alpha*(last_filtered_roll + gyro_event.gyro.x*dt*RAD_TO_DEG) + (1-alpha)*(accel_roll);
    filtered_pitch = alpha*(last_filtered_pitch + gyro_event.gyro.y*dt*RAD_TO_DEG) + (1-alpha)*(accel_pitch);
    filtered_yaw = alpha*(last_filtered_yaw + gyro_event.gyro.z*dt*RAD_TO_DEG) + (1-alpha)*(accel_yaw);
    last_filtered_roll = filtered_roll;
    last_filtered_pitch = filtered_pitch;
    last_filtered_yaw = filtered_yaw;
 
    Serial.print(filtered_roll); Serial.print("  ");
    Serial.print(filtered_pitch); Serial.print("  ");
    Serial.print(filtered_yaw); Serial.print("  ");
    Serial.print("deg ");
    
    Serial.print("X: "); Serial.print(accel_event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(accel_event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(accel_event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
    delay(50);
    
    digitalWrite(13, LOW);
    delay(50);
    last_time = current_time;
  }
}
