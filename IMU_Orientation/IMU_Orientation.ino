#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  //Serial.println("Orientation Sensor Test"); Serial.println("");

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
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  printOrientationData(&orientationData);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printOrientationData(sensors_event_t* event) {
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    double x = event->orientation.x;
    double y = event->orientation.y;
    double z = event->orientation.z;
    /*
    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print(" ms");
    Serial.print("\tOrient:\tx= ");
    Serial.print(x);
    Serial.print(" |\ty= ");
    Serial.print(y);
    Serial.print(" |\tz= ");
    Serial.println(z);
    */
    Serial.println(y);
  }
}