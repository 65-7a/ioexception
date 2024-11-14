#include <Arduino.h>
#include <Servo.h>
#include <math.h>

Servo Base_Horizontal;
Servo Base_Vertical;
Servo Elbow_Horizontal;
Servo Elbow_Vertical;

void setup() {
  // Adjust these values for the correct pins, once known
  Base_Horizontal.attach(0);
  Base_Vertical.attach(0);
  Elbow_Horizontal.attach(0);
  Elbow_Vertical.attach(0);
};

float armLength = 2.5;

int moveto(int rad, int deg) {
  rad = constrain(rad, 0, 2 * armLength);
  deg = constrain(deg, 0, 360);
  Base_Horizontal.write(deg);
  float Base_Vert_Angle = acos((pow(rad, 2))/2*(armLength*rad)); //a^2 & c^2 cancel out making it b^2/2ab
  Base_Vertical.write(Base_Vert_Angle);

  float Elbow_Vert_Angle = acos((2*(pow(armLength, 2) - pow(rad, 2))/2*(pow(armLength, 2)))); //All these being isosceles triangles is real neat
  Elbow_Vertical.write(Elbow_Vert_Angle);
  return 1;
};

int wave = 0;
void loop() {
  int success = moveto(sin(wave), wave%360);
  wave = (wave+1)%360;
};