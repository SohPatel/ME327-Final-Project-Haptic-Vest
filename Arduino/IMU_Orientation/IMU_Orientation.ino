#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*
Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
  */

uint16_t BNO055_SAMPLERATE_DELAY_MS = 50;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop(void)
{
  sensors_event_t orientationData;
  bno.getEvent(&orientationData);

  printOrientationData(&orientationData);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printOrientationData(sensors_event_t* event) {
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
    double x = -180/M_PI * euler.x(); // heading, nose-right is positive, z-axis points up
    double y = -180/M_PI * euler.y(); // roll, rightwing-up is positive, y-axis points forward
    double z = -180/M_PI * euler.z(); // pitch, nose-down is positive, x-axis points right

    Serial.print(-360); // To freeze the lower limit
    Serial.print(" ");
    Serial.print(360); // To freeze the upper limit
    Serial.print(" ");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.println(z);
  }
}