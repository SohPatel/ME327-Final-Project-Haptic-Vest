#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//#define CALIBRATE_FLEXION
//#define SPEED_FEEDBACK
#define CALIBRATE

const int buzzerPin1 = 9; // buzzer front
const int buzzerPin2 = 10; // buzzer back
const int MAX_BUZZER_AMP = 170;
const int MIN_BUZZER_AMP = 70;

const int forward_angle_min = 1;
const int forward_angle_max = 8;

const int backward_angle_min = -3;
const int backward_angle_max = -12;

int absolute_angle_max = 50;

const int num_threshold = 3;

int counter_positive = 0;
int counter_negative = 0;
int counter_absolute = 0;
int counter_calib = 0;

double top_temp, bot_temp;
double top_avg, bot_avg;

bool user_calibrated = false;
double max_flexion = 0;

/*
Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VIN to 3.3-5V DC
   Connect GROUND to common ground
   Connect ADR to 3.3-5V DC (Bottom IMU Only)
  */

uint16_t BNO055_SAMPLERATE_DELAY_MS = 50;
const uint16_t MIN_DELAY = 20;
const uint16_t MAX_DELAY = 200; 

const double MAX_ANGULAR_VELOCITY = 1.5; // rad/s

Adafruit_BNO055 bno_top = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bno_bottom = Adafruit_BNO055(55, 0x29, &Wire);

void setup(void)
{
  pinMode(buzzerPin1, OUTPUT);
  pinMode(buzzerPin2, OUTPUT);

  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  if (!bno_top.begin())
  {
    Serial.print("Ooops, no top BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!bno_bottom.begin())
  {
    Serial.print("Ooops, no bottom BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop(void)
{
  sensors_event_t topOrientationData;
  sensors_event_t bottomOrientationData;
  bno_top.getEvent(&topOrientationData);
  bno_bottom.getEvent(&bottomOrientationData);

  sensors_event_t topAngularVelocityData;
  sensors_event_t bottomAngularVelocityData;
  bno_top.getEvent(&topAngularVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno_bottom.getEvent(&bottomAngularVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);



  // printOrientationData(&topOrientationData, bno_top);
  // Serial.print(",");
  // printOrientationData(&bottomOrientationData, bno_bottom);
  // Serial.println("");

  double z_top = getOrientationZ(&topOrientationData, bno_top);
  double z_bottom = getOrientationZ(&bottomOrientationData, bno_bottom);

  double z_dot_top = getAngularVelocityZ(&topAngularVelocityData, bno_top);
  double z_dot_bottom = getAngularVelocityZ(&bottomAngularVelocityData, bno_bottom);

  uint16_t buzzer_delay = BNO055_SAMPLERATE_DELAY_MS;

  #ifdef CALIBRATE_FLEXION
  if (!user_calibrated) {
    if (z_top >= max_flexion) {
      max_flexion = z_top;
    } else {
      counter_calib++;
    }

    // Calibrated if no new max flexion for > 1s.
    if (counter_calib > 20) {
      user_calibrated = true;
      absolute_angle_max = max_flexion;
      indicateCalibrated(buzzerPin1, buzzerPin2, BNO055_SAMPLERATE_DELAY_MS);
      Serial.print("Front flexion calibrated at: ");
      Serial.println(max_flexion);
    }

    delay(BNO055_SAMPLERATE_DELAY_MS);
    return;
  }
  #endif

  #ifdef CALIBRATE
  if (!user_calibrated) {
      top_temp += z_top;
      bot_temp += z_bottom;
      counter_calib++;

    // Calibrated if no new max flexion for > 5s.
    if (counter_calib > 100) {
      user_calibrated = true;
      top_avg = top_temp / counter_calib;
      bot_avg = bot_temp / counter_calib;
      indicateCalibrated(buzzerPin1, buzzerPin2, BNO055_SAMPLERATE_DELAY_MS);
      Serial.print("Top avg: ");
      Serial.println(top_avg);
      Serial.print("Bot avg: ");
      Serial.println(bot_avg);
    }

    delay(BNO055_SAMPLERATE_DELAY_MS);
    return;
  }
  #endif

  // -------- Absolute diff --------
  // if (z_top > absolute_angle_max) {
  //   counter_absolute++;
  //   if (counter_absolute > num_threshold) {
  //     analogWrite(buzzerPin1, MAX_BUZZER_AMP);
  //     analogWrite(buzzerPin2, MAX_BUZZER_AMP);
  //     Serial.println("Max absolute angle reached");
  //     #ifdef SPEED_FEEDBACK
  //     buzzer_delay = MIN_DELAY;
  //     #endif
  //   }
  // } else {
  //   counter_absolute = 0;
  // }

  double diff = (z_top - z_bottom - top_avg + bot_avg);
  Serial.println(diff);

  // -------- Relative diff --------
  if (z_top - z_bottom > top_avg - bot_avg + forward_angle_min) {
    counter_positive++;
    if (counter_positive > num_threshold) {
      double diff = (z_top - z_bottom - top_avg + bot_avg);
      int buzzStrength = map(diff, forward_angle_min, forward_angle_max, MIN_BUZZER_AMP, MAX_BUZZER_AMP);
      analogWrite(buzzerPin1, buzzStrength);
      Serial.print("Buzz Forward: ");
      Serial.println(diff);
      Serial.print("strength: ");
      Serial.println(buzzStrength);
      #ifdef SPEED_FEEDBACK
        if (z_dot_top > 0)
          buzzer_delay = map(z_dot_top, MAX_ANGULAR_VELOCITY, 0.0 , MIN_DELAY, MAX_DELAY);
        else buzzer_delay = MAX_DELAY;
      #endif
    }
  } else {
    counter_positive = 0;
  }

  if (z_top - z_bottom < top_avg - bot_avg + backward_angle_min) {
    counter_negative++;
    if (counter_negative > num_threshold) {
      double diff = (z_top - z_bottom - top_avg + bot_avg);
      int buzzStrength = map(diff, backward_angle_max, backward_angle_min, MIN_BUZZER_AMP, MAX_BUZZER_AMP);
      analogWrite(buzzerPin2, buzzStrength);
      Serial.print("Buzz Backward: ");
      Serial.println(diff, buzzStrength);
      #ifdef SPEED_FEEDBACK
        if (z_dot_top < 0)
          buzzer_delay = map(z_dot_top, -MAX_ANGULAR_VELOCITY, 0.0 , MIN_DELAY, MAX_DELAY);
        else buzzer_delay = MAX_DELAY;
      #endif
    }
  } else {
    counter_negative = 0;
  }

  delay(2*buzzer_delay);
  analogWrite(buzzerPin1, 0);
  analogWrite(buzzerPin2, 0);
  delay(buzzer_delay);
}

void printOrientationData(sensors_event_t* event, Adafruit_BNO055 bno) {
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    //double x = event->orientation.x;
    //double y = event->orientation.y;
    //double z = event->orientation.z;
    
  /* Convert quaternion to Euler, because BNO055 Euler data is broken */
    imu::Quaternion q = bno.getQuat();
    q.normalize();
    float temp = q.x();  q.x() = -q.y();  q.y() = temp;
    q.z() = -q.z();
    imu::Vector<3> euler = q.toEuler();
    double x = 180/M_PI * euler.x(); // heading, nose-right is positive, z-axis points up
    double y = 180/M_PI * euler.y(); // roll, rightwing-up is positive, y-axis points forward
    double z = -180/M_PI * euler.z(); // pitch, nose-down is positive, x-axis points right

    /*
    Serial.print(-360); // To freeze the lower limit
    Serial.print(" ");
    Serial.print(360); // To freeze the upper limit
    Serial.print(" ");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    */
    Serial.print(z);
  }
}

double getOrientationZ(sensors_event_t* event, Adafruit_BNO055 bno) {
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    imu::Quaternion q = bno.getQuat();
    q.normalize();
    float temp = q.x();  q.x() = -q.y();  q.y() = temp;
    q.z() = -q.z();
    imu::Vector<3> euler = q.toEuler();
    double x = 180/M_PI * euler.x(); // heading, nose-right is positive, z-axis points up
    double y = 180/M_PI * euler.y(); // roll, rightwing-up is positive, y-axis points forward
    double z = -180/M_PI * euler.z(); // pitch, nose-down is positive, x-axis points right
    return z;
  }
  return 0.0; // Return a default value if the event type is not SENSOR_TYPE_ORIENTATION
}

double getAngularVelocityZ(sensors_event_t* event, Adafruit_BNO055 bno) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
    return z;
  }
  return 0.0; // Return a default value if the event type is not SENSOR_TYPE_GYROSCOPE
}

// Indicates user is calibrated
void indicateCalibrated(const int bp1, const int bp2, double delay_rate) {
    analogWrite(bp1, MAX_BUZZER_AMP);
    delay(delay_rate);
    analogWrite(bp1, 0);
    delay(delay_rate);
    analogWrite(bp1, MAX_BUZZER_AMP);
    delay(delay_rate);
    analogWrite(bp1, 0);
    delay(delay_rate);

    analogWrite(bp2, MAX_BUZZER_AMP);
    delay(delay_rate);
    analogWrite(bp2, 0);
    delay(delay_rate);
    analogWrite(bp2, MAX_BUZZER_AMP);
    delay(delay_rate);
    analogWrite(bp2, 0);
    delay(delay_rate);
}