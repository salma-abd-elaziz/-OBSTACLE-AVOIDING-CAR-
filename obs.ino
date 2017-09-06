#include <NewPing.h>              //The Ultrasonic sensor library
#include <Servo.h>                //The servo motor library 
#include <AFMotor.h>              //The Car library


#define S1 255                   //The car wheels speed
#define TRIGGER_PIN  26           
#define ECHO_PIN     28
#define MAX_DISTANCE 400         //The max distance that the sensor checks
#define LEFT 180                 //To move the servo
#define RIGHT 0    
#define FRONT 90
#define BUZZ 14 
#define GREEN_LINE 15

//identifing variables
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
Servo servo; 

int motor_speed;
int motor_direction1;
int motor_direction2;
int motor_direction3;
int motor_direction4;
int distance;
int left_dist;
int right_dist;

void setup() {
  Serial.begin(9600);
  pinMode(BUZZ, OUTPUT); 
  pinMode(GREEN_LINE, OUTPUT);
   // Turn on motor
  motor1.setSpeed(S1);
  motor2.setSpeed(S1);
  motor3.setSpeed(S1);
  motor4.setSpeed(S1);

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  servo.attach(24);      // Attach the signal pin of servo to pin 24 of arduino
  delay(1000);
  servo.write(FRONT);
}
 
void loop() {
  //Reading the distance with the sensor in cm
  distance = sonar.ping_cm();  
  Serial.println(distance);
  //If there is no obstacle just keep walking forward
  if (distance > 50 || distance == 0) {
      digitalWrite(BUZZ, LOW);  
      digitalWrite(GREEN_LINE, HIGH);
      motor_direction1 = FORWARD;
      motor_direction2 = FORWARD;
      motor_direction3 = FORWARD;
      motor_direction4 = FORWARD; 
      motor1.run(motor_direction1);
      motor2.run(motor_direction2);
      motor3.run(motor_direction3);
      motor4.run(motor_direction4);
  } else { //Obstacle detected
      digitalWrite(BUZZ, HIGH); 
      digitalWrite(GREEN_LINE, LOW);
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);
      servo.write(LEFT);
      delay(200);
      left_dist = sonar.ping_cm();
      servo.write(RIGHT);
      delay(200);
      right_dist = sonar.ping_cm();
      //Reading the distance left and right to decide which way to take
      if (left_dist > right_dist && right_dist != 0) { //move left
         motor_direction1 = FORWARD;
         motor_direction2 = FORWARD;
         motor_direction3 = RELEASE;
         motor_direction4 = RELEASE;
      } else {                      // Move right
         motor_direction1 = RELEASE;
         motor_direction2 = RELEASE;
         motor_direction3 = FORWARD;
         motor_direction4 = FORWARD;
      }
      int curr = millis(); 
      //The car keep moving in the choosen direction for 1.5 sec
      while (millis() - curr < 1500) {
        motor1.run(motor_direction1);
        motor2.run(motor_direction2);
        motor3.run(motor_direction3);
        motor4.run(motor_direction4);
      }
      servo.write(FRONT);
  }
  digitalWrite(BUZZ, LOW);  
  digitalWrite(GREEN_LINE, HIGH);
  delay(500);
}
