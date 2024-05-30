#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int buzzerPin1 = 9; // buzzer front
const int buzzerPin2 = 10; // buzzer back

const int forward_angle_min = 10;
const int forward_angle_max = 50;

const int backward_angle_min = -10;
const int backward_angle_max = -50;

const int absolute_angle_max = 50;

const int num_threshold = 3;

int counter_positive = 0;
int counter_negative = 0;
int counter_absolute = 0;

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

  // printOrientationData(&topOrientationData, bno_top);
  // Serial.print(",");
  // printOrientationData(&bottomOrientationData, bno_bottom);
  // Serial.println("");

  double z_top = getOrientationZ(&topOrientationData, bno_top);
  double z_bottom = getOrientationZ(&bottomOrientationData, bno_bottom);

  // -------- Absolute diff --------
  if (z_top > absolute_angle_max) {
    counter_absolute++;
    if (counter_absolute > num_threshold) {
      analogWrite(buzzerPin1, 255);
      analogWrite(buzzerPin2, 255);
      Serial.println("Max absolute angle reached");
    }
  } else {
    counter_absolute = 0;
  }

  // -------- Relative diff --------
  if (z_top - z_bottom > forward_angle_min) {
    counter_positive++;
    if (counter_positive > num_threshold) {
      analogWrite(buzzerPin1, (z_top-z_bottom)/(forward_angle_max)*255);
      Serial.print("Buzz Forward: ");
      Serial.println((z_top-z_bottom)/(forward_angle_max)*255);
    }
  } else {
    counter_positive = 0;
  }

  if (z_top - z_bottom < backward_angle_min) {
    counter_negative++;
    if (counter_negative > num_threshold) {
      analogWrite(buzzerPin2, (z_top-z_bottom)/(backward_angle_max)*255);
      Serial.print("Buzz Backward: ");
      Serial.println((z_top-z_bottom)/(backward_angle_max)*255);
    }
  } else {
    counter_negative = 0;
  }

  delay(2*BNO055_SAMPLERATE_DELAY_MS);
  analogWrite(buzzerPin1, 0);
  analogWrite(buzzerPin2, 0);
  delay(BNO055_SAMPLERATE_DELAY_MS);
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