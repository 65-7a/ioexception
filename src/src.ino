#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  150 // Subject to change, based on testing
#define SERVOMAX  600 // Subject to change, based on testing
#define USMIN  600  // Subject to change, based on testing
#define USMAX  2400 // Subject to change, based on testing
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// A guess - will need to be recalibrated in testing
uint8_t Base_Horizontal  = 0;
uint8_t Base_Vertical    = 1;
uint8_t Elbow_Vertical   = 2;
uint8_t Wrist_Horizontal = 3;

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();

float Forearm = 2.5; // Remember to update this
float Aftarm  = 2.5; // Remember to update this

void setup() {
  Serial.begin(115200); // Output to connected computer
  Serial.println("Begin controlling now"); // Idk we may need a more appropriate startup message but that *is* the end goal so 

  servo.begin(); // Initialise connection to the adafruit servo controller thingy
  servo.setOscillatorFrequency(27*pow(10,6)); // Set the oscillator to 27MHz
  servo.setPWMFreq(SERVO_FREQ);  // Tells the PWM board the speed to output to the servos

  delay(10); // Wait 10ms
};

int moveto(int rad, int deg) {
  rad = constrain(rad, 0, Forearm + Aftarm); // Stops you from going to a location farther away than the robot can reach
  deg = deg%360; // Converts all angles to a number between 1 & 360

  servo.setPWM(Base_Horizontal, 0, map(deg, 0, 360, SERVOMIN, SERVOMAX)); // Spins the base of the robot to the desired angle

  float Base_Vert_Angle = acos((pow(rad, 2)+(Aftarm, 2) - pow(Forearm,2))/2*Forearm*rad); // Gets the angle of the shoulder joint using the law of cosines.
  servo.setPWM(Base_Vertical, 0, map(Base_Vert_Angle, 0, 360, SERVOMIN, SERVOMAX)); // Raises the shoulder to the desired angle


  float Elbow_Vert_Angle = acos((pow(Forearm, 2) + pow(Aftarm, 2) - pow(rad, 2)) / 2 * Forearm * Aftarm); // Gets the angle of the shoulder joint using the law of cosines.
  servo.setPWM(Elbow_Vertical, 0, map(Elbow_Vert_Angle, 0, 360, SERVOMIN, SERVOMAX)); // Raises the elbow to the desired angle
  return 1;
};

int wave = 0;
void loop() {
  int success = moveto(sin(wave), wave%360);
  wave = (wave+1)%360;

  if (Serial.available()) {
    Serial.print("Keypress:");
    Serial.println(Serial.read());
  }
};