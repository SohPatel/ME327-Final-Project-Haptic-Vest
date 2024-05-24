import processing.serial.*;

Serial myPort;        // The serial port
String[] data;        // To hold the incoming serial data
float[] pitch, roll, yaw;  // Arrays to hold past values
int maxPoints = 500;  // Maximum number of points to plot

void setup() {
  size(800, 600);
  String portName = Serial.list()[1];  // Change the index if the wrong port is selected
  myPort = new Serial(this, portName, 115200);
  pitch = new float[maxPoints];
  roll = new float[maxPoints];
  yaw = new float[maxPoints];
}

void draw() {
  background(255);
  strokeWeight(2);

  // Read the serial data
  if (myPort.available() > 0) {
    String inString = myPort.readStringUntil('\n');
    if (inString != null) {
      inString = trim(inString);
      data = split(inString, ',');
      if (data.length == 3) {
        // Shift the old values to the left
        for (int i = 0; i < maxPoints-1; i++) {
          pitch[i] = pitch[i+1];
          roll[i] = roll[i+1];
          yaw[i] = yaw[i+1];
        }
        // Add the new values at the end
        pitch[maxPoints-1] = float(data[2]);
        roll[maxPoints-1] = float(data[1]);
        yaw[maxPoints-1] = float(data[0]);
      }
    }
  }

  // Draw the axes
  stroke(0);
  line(50, height-50, width-50, height-50);
  line(50, 50, 50, height-50);

  // Plot the pitch data
  stroke(255, 0, 0);  // Red for pitch
  noFill();
  beginShape();
  for (int i = 0; i < maxPoints; i++) {
    float x = map(i, 0, maxPoints, 50, width-50);
    float y = map(pitch[i], 0, 360, height-50, 50);
    vertex(x, y);
  }
  endShape();

  // Plot the roll data
  stroke(0, 255, 0);  // Green for roll
  beginShape();
  for (int i = 0; i < maxPoints; i++) {
    float x = map(i, 0, maxPoints, 50, width-50);
    float y = map(roll[i], 0, 360, height-50, 50);
    vertex(x, y);
  }
  endShape();

  // Plot the yaw data
  stroke(0, 0, 255);  // Blue for yaw
  beginShape();
  for (int i = 0; i < maxPoints; i++) {
    float x = map(i, 0, maxPoints, 50, width-50);
    float y = map(yaw[i], 0, 360, height-50, 50);
    vertex(x, y);
  }
  endShape();
}
