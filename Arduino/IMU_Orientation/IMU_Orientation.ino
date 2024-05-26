#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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

  printOrientationData(&topOrientationData, bno_top);
  Serial.print(",");
  printOrientationData(&bottomOrientationData, bno_bottom);
  Serial.println("");

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