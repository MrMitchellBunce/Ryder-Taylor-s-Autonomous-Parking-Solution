#include <Wire.h>
#include <MPU6050_tockn.h>
#define I2C_SLAVE_ADDR 0x04
// 4 in hexadecimal
#define SOUND_SPEED 0.034
// define sound speed in cm/uS
MPU6050 mpu6050(Wire);
// Defines the gyroscope of MPU-9250
int leftMotor_forward = 230;
int leftMotor_backwards = -115;
int leftMotor_stop = 0;
int rightMotor_forward = 230;
int rightMotor_backwards = -115;
int rightMotor_stop = 0;
int servoAngle_straight = 60;
int servoAngle_turn_right = 180;
int servoAngle_turn_left = 0;
// Defines variables used to control motor speed/direction and steering angle
long enc1_count;
long enc2_count;
long enc1_count_new;
long enc2_count_new;
// Defines variables used to track encoder counts requested from slave arduino
void setup()
{
  Serial.begin(9600);
  Wire.begin();
// join i2c bus (address optional for the master) - on the Arduino NANO
//the default I2C pins are A4 (SDA), A5 (SCL)
}

void loop()
{ // Parking problem solution main loop performs actions in order requested
  forwards();  
  delay(1000);
  // Drives forwards for 1 second
  turn(180);
  // Turns 180 degrees anti-clockwise (left)
  delay(10);
  Reverse();
  // Reverses car until object is sensed within 10cm then stops
  delay(10);
  turn(90);
  // Turns 90 degrees clockwise (right)
  delay(10);
  Reverse();
  // Reverses car until object is sensed within 10cm then stops
  delay(10);
  turn(90);
  // Turns 90 degrees clockwise (right) so that method b. can be completed
  delay(10);  
  move_30cm();
  // Moves car 30cm backward then stops (method b. implemented after a.)
  delay(10000);
}

void forwards()
{ // Function transmits motor & steering values to slave arduino to make EEEBot go straight forward
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)((leftMotor_forward & 0x0000FF00) >> 8));
  Wire.write((byte)(leftMotor_forward & 0x000000FF));
  Wire.write((byte)((rightMotor_forward & 0x0000FF00) >> 8));
  Wire.write((byte)(rightMotor_forward & 0x000000FF));
  Wire.write((byte)((servoAngle_straight & 0x0000FF00) >> 8));
  Wire.write((byte)(servoAngle_straight & 0x000000FF));
  Wire.endTransmission();
}

void backwards() 
{// Fucntion transmits motor & steering values to slave arduino to make EEEBot go straight backward
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)((leftMotor_backwards & 0x0000FF00) >> 8));
  Wire.write((byte)(leftMotor_backwards & 0x000000FF));
  Wire.write((byte)((rightMotor_backwards & 0x0000FF00) >> 8));
  Wire.write((byte)(rightMotor_backwards & 0x000000FF));
  Wire.write((byte)((servoAngle_straight & 0x0000FF00) >> 8));
  Wire.write((byte)(servoAngle_straight & 0x000000FF));
  Wire.endTransmission();
}

void stop()
{ // Function transmits motor & steering values to slave arduino to make EEEBot stop moving
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write((byte)((leftMotor_stop & 0x0000FF00) >> 8));
  Wire.write((byte)(leftMotor_stop & 0x000000FF));
  Wire.write((byte)((rightMotor_stop & 0x0000FF00) >> 8));
  Wire.write((byte)(rightMotor_stop & 0x000000FF));
  Wire.write((byte)((servoAngle_straight & 0x0000FF00) >> 8));
  Wire.write((byte)(servoAngle_straight & 0x000000FF));
  Wire.endTransmission();
}

void turn(int turnType)
{ // Function transmits motor and steering values to the slave arduino
  // to make the EEEBot turn a certain amount right or left
  int angle = 0;
  //Defines local variable to measure the original Z angle of the EEEBot
  float angleChange = 0; 
  //Defines local variable to measure the change in the angle
  Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write((byte)((leftMotor_forward & 0x0000FF00) >> 8));
    Wire.write((byte)(leftMotor_forward & 0x000000FF));
    Wire.write((byte)((rightMotor_forward & 0x0000FF00) >> 8));
    Wire.write((byte)(rightMotor_forward & 0x000000FF));
    // Makes the EEEBot begin to drive forward 
  if (turnType == 180){                                  
    Wire.write((byte)((servoAngle_turn_left & 0x0000FF00) >> 8));
    Wire.write((byte)(servoAngle_turn_left & 0x000000FF));  
    Wire.endTransmission();
    // Front wheels steer to left for first turn of 180 degrees
  }
  else{
    Wire.write((byte)((servoAngle_turn_right & 0x0000FF00) >> 8));
    Wire.write((byte)(servoAngle_turn_right & 0x000000FF));  
    Wire.endTransmission();   
    // Front wheels steer to right for final turn of 90 degrees 
  }
  mpu6050.begin(); 
  // Starts the gyrocsope
  angle = (mpu6050.getAngleZ());
  // Stores initial Z angle
  while (angleChange < turnType)
  { // Loops until the Z angle of the vehicle has decreased
    // or increased by the amount specified by turnType
    // meaning the EEEBot will have turned that many degrees in the correct direction
    mpu6050.update();
    // Updates the angles
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    // Prints the values onto serial monitor so they can be tracked by user
    if (turnType == 90)
    { // If 90 is passed the car is truning right whereas 180 is left
      angleChange = (angle - (mpu6050.getAngleZ()));
      // Angle change for turning right is found by taking away the new Z angle from the original 
    }
    else
    {
      angleChange = ((mpu6050.getAngleZ()) - angle);
      // Angle change for turning left is found by taking away
      // the original Z angle from the new one
    } // This is done as when turning left the angle increases
  }   // whereas the opposite is true when turning right
}

void Reverse()
{ // Function reverses the car into a "parking space" and stops when 10cm away from an object
  backwards();
  // Begins to move backwards
  delay(10);
  measureDistance();
  // Meausres distance until object is seen less than 10cm away
  stop();
  // stops the car before hitting the object
  delay(10);
}

void measureDistance()
{ // Function measures the distance from objects using the HC-SR04
  const int trigPin = 5;
  const int echoPin = 18;
  // Local variables define the trigger pin and echo pin connections to ESP-32
  long duration;
  float distanceCm = 100;
  pinMode(trigPin, OUTPUT);
  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);
   // Sets the echoPin as an Input
  while (distanceCm > 10)
  { // Loops until an object is 10cm away from the sensor as the EEEBot reverses
    digitalWrite(trigPin, LOW);
    // Clears the trigPin
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    // Sets the trigPin on HIGH state for 10 micro seconds
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    distanceCm = duration * SOUND_SPEED/2; 
    // Calculate the distance
    Serial.print("Distance (cm): ");
    Serial.println(distanceCm);
    // Calculates the distance    
    delay(10);
  }
}

void move_30cm() 
{ // Function allows car to move 30cm backward
  // after final stop to solve b. in addtion to a.
  int encChange = 0;
  // Define local variable to measure change in encoder
  //  values, arduino does not reset encoder count
  Wire.requestFrom(I2C_SLAVE_ADDR,2);
  // Request encoder values from arduino
  if (Wire.available () >= 2)
  { // Reads values if avaliable
    enc1_count = Wire.read();
    enc2_count = Wire.read();
    // Variables to save original encoder count before movement
  }
  backwards();
  // Starts to move backward
  delay(10);
  while (encChange < 41) 
  { // Loops while the change in the encoder value is less than 41
    // than 41 as that is the number of rotations in 30cm
    Wire.requestFrom(I2C_SLAVE_ADDR,2);
    // Request encoder values from arduino for new count
    if (Wire.available () >= 2)
    { // Reads values for new encoder count if avaliable
      enc1_count_new = Wire.read();
      enc2_count_new = Wire.read();
      if (enc1_count <= enc1_count_new)
      {// Compares new encoder count to original, count increases
       // with rotations backward. new count is assumed to
       // be > original, but this is not always true as the count
       // value loops round to 0 when it reaches 255                                
        encChange = (enc1_count_new - enc1_count);
       // Count change measured by taking original count away
      }// from new count as it is greater than the original
      else if (enc1_count > enc1_count_new)
      {// If the new count is smaller than the original
       // (imagine the original starts as 246 and the new loops from 246 to 16)
       // then this else if is used as the previous calculation
       //would give a negative encChange which should not be possible                                     
        encChange = ((enc1_count_new + 254) - enc1_count);
      }// 254 is added to the new count to make up for the
       // new count looping and provide the correct encChange value      
      if (encChange > 50)
      {// If an error occurs and the change is measured to be greater
       // than possible e.g. new count being 1 smaller than the old count
        encChange = 0;
       // the encChange is set to 0 and
        enc1_count = enc1_count_new; 
      } // The original encoder count is set to match the new one to solve the error
      Serial.print("\enc1 : ");
      Serial.println(enc1_count_new);
      Serial.print("\enc1 og : ");
      Serial.println(enc1_count);
      Serial.print("\enc1 change : ");
      Serial.println(encChange);
    } // Updates serial monitor with relevant information for testing and checking
  }
  stop();  
  delay(10);
} // Stops the EEEBot