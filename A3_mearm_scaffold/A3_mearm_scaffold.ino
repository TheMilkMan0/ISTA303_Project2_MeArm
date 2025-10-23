/* 
  Assignment 3 (MeArm) Scaffold
  Kinematics code for MeArm
  Do not use or look at the open code available for the MeArm -- you must write your own!
  NOTE: Must change "NO LINE ENDING" to "NEWLINE" in Serial Monitor (button on top right) for this to work. 
*/

#include <Servo.h>

// Servo pins
#define SERVO_CLAW_PIN  6  // CLAW
#define SERVO_RIGHT1_PIN  9  // RIGHT (Kinematics Servo 1) 
#define SERVO_LEFT2_PIN  10 // LEFT (Kinmatics Servo 2)
#define SERVO_MIDDLE_PIN  11 // Middle


// Create servo objects to control servos
Servo SERVO_CLAW;  
Servo SERVO_RIGHT1;  
Servo SERVO_LEFT2;  
Servo SERVO_MIDDLE;   

// Global variables for storing the position to move to
int moveToTheta     = 0;
int moveToR         = 100;
int moveToZ         = 50;
int moveToGripper   = 0;

/*
 * Functions (
 */

// Move the rotational (theta, polar coordinates) axis of the MeArm
void moveTheta(int angle) {
  
}

// Open the gripper a given width/distance
void moveGripper(int distToOpen) {
  
}

// Move the arm along the r axis (polar coordinates), or in height (z)
void moveRZ(int r_side, int z_side) {
  // --- ANGLES NEEDED FOR SERVO 1 ---
  int   a_side = 81; // by default a = 81 mm 
  int   b_side = 81; // by default b = 81 mm
  float c_side = sqrt(pow(z_side,2) + pow(r_side,2)); // Pythagorean theorem to find the 3rd side of the triangle, C is also the hypotnuse of z and r 

  float K_radians = acos(r_side/c_side); // arccos(adj/hypotnuse) = Small K angle  hypotnuse = c because its by defn the longest side of the triangle 

  // To find the angle of B use the law of cosines. Rearanging the formual to get cos(B) by itself and use inverse of Cos
  // -(a^2 - b^2 - c^2)
  float numerator =   -(pow(a_side,2)+pow(c_side,2)-pow(b_side,2));
  float denomerator = -(2*a_side*c_side);
  float B_radians = acos(numerator/denomerator); // 

  float K_angle = K_radians * 180/3.14; // Convert the calculated K radian to an degree angle
  float B_angle = B_radians * 180/3.14; // Convert the calculated B radian to a degree angle

  float right_Servo1Delta = K_angle + B_angle;

  // --- ANGLES NEEDED FOR SERVO 2 ---
  float X_angle = 90 - (K_angle+B_angle); // [Right angle total angle = 90]
  float Y_angle = 180 - 90 - X_angle; // [Total Angle of triangle equals 180]
  float C_angle = 180 - 2*B_angle; // [Isosceles Triangle total angle sum = 180]
  
  float W_angle = 180 - Y_angle - C_angle; // [180 line from one side to another = total sum of 180]

  float left_Servo2Delta = W_angle;

  // -- TESTING -- 
  Serial.println("Checking moveRZ angles:");
  Serial.print("c_side: ");
  Serial.print(c_side);
   Serial.print(", K_angle: ");
  Serial.print(K_angle);
  Serial.print(", B_angle: ");
  Serial.print(B_angle);
  Serial.print(", K+B_angle: ");
  Serial.print(right_Servo1Delta);

  Serial.print(", X_angle: ");
  Serial.print(X_angle);
  Serial.print(", Y_angle: ");
  Serial.print(Y_angle);
  Serial.print(", C_angle: ");
  Serial.print(C_angle);
  Serial.print(", W_angle: ");
  Serial.print(left_Servo2Delta);


  int calculated_RightServo_offset = -1.93*(right_Servo1Delta) + 185.33; // we want to bring back our calculated angle so then the moter can move forward that amount and itll alighn with our K+B Angle
  int rightServoDelta_w_offset = right_Servo1Delta + calculated_RightServo_offset; // Our outputed angle was -12 of the input, so lets add 12
  SERVO_RIGHT1.write(rightServoDelta_w_offset); // this should be just a constant value for the right to make the measuring of the left servo easier after the testing setup is setup 
  int calculated_LeftServo_offset = -1.77*(left_Servo2Delta) + 69.24; // used least squares regression to find this function
  int leftServoDelta_w_offset = left_Servo2Delta + calculated_LeftServo_offset;
  SERVO_LEFT2.write(leftServoDelta_w_offset);


  /*
  int calculated_offset = -(-(1.9*servo1Delta)+181); // we want to bring back our calculated angle so then the moter can move forward that amount and itll alighn with our K+B Angle
  int servo1Delta_w_offset = servo1Delta + calculated_offset; // Our outputed angle was -12 of the input, so lets add 12
  SERVO_RIGHT1.write(servo1Delta_w_offset);// `servo1Delta` references the kinematics diagram labling 

  //int servo2Delta_w_offset = -1 * (servo2Delta + LEFT_SERVO_ANGLE_OFFSET); // We have to do this -1*(moveToTheta-80) to the caclulated angle to get our desired angle 
  //SERVO_LEFT2.write(servo2Delta); // `servo2Delta` references the kinematics diagram labling 
  */
}



/*
 * Arduino core (setup, loop)
 */
 
void setup() {
  // Enable serial port output for debug
  Serial.begin(9600);
  Serial.println("MeArm Initializing...");
 
  // Attaches the servo objects to servos on specific pins
  SERVO_CLAW.attach(SERVO_CLAW_PIN);
  SERVO_RIGHT1.attach(SERVO_RIGHT1_PIN);
  SERVO_LEFT2.attach(SERVO_LEFT2_PIN);
  SERVO_MIDDLE.attach(SERVO_MIDDLE_PIN);
  
}

// Display a simple serial console to the user that allows them to enter positional information for the MeArm to move to. 
void doSerialConsole() {
  // Display serial console 
  char inputStr[80];
  int index = 0;
  
  while (1) {
    int done = 0;
    // Step 1: Display serial console message
    Serial.println("");
    Serial.println("Enter coordinates: theta,r,z,gripper (comma delimited, no spaces)");
    Serial.println("Example: 10,20,30,40");

    // Step 2: Clear string
    for (int i=0; i<80; i++) {
      inputStr[i] = 0;
    }
    index = 0;

    // Step 3: Read serial data
    while (done == 0) {    
      // Step 3A: Read serial data and add to input string, if available
      while(Serial.available() > 0) {
        char inputChar = Serial.read();        
        if (inputChar == '\n') {          
          // Newline character -- user has pressed enter
          done = 1;                
        } else {
          // Regular character -- add to string
          if (index < 79) {
            inputStr[index] = inputChar;                  
            index += 1;
          }
        }      
      }      
    }

    // Step 4: Debug output: Let the user know what they've input
    Serial.print ("Recieved input: ");
    Serial.println(inputStr);

    // Step 5: Check if string is valid
    if (sscanf(inputStr, "%d,%d,%d,%d", &moveToTheta, &moveToR, &moveToZ, &moveToGripper) == 4) {  
      Serial.println("Valid input!");  
      // Valid string
      return;
    } else {
      // Invalid string -- restart
      Serial.println ("Invalid input -- try again.  example: 10,20,30,40");      
    }      
  }    
}

void loop() {
  
  // Step 1: Display Serial console
  doSerialConsole();

  // Step 2: Debug display
  Serial.println("");
  Serial.println("Moving to: ");
  Serial.print("Theta: ");
  Serial.println(moveToTheta);
  Serial.print("R: ");
  Serial.println(moveToR);
  Serial.print("Z: ");
  Serial.println(moveToZ);
  Serial.print("Gripper: ");
  Serial.println(moveToGripper);
  

  // Step 3: Move to requested location
  //moveTheta(moveToTheta);
  moveRZ(moveToR, moveToZ);
  //moveGripper(moveToGripper);  

  // -- TESTING TRIG -- 
  // 0,87,22,0 expecting c_side = 90, K_angle = 14.2, B_angle = 56.25, K+B_Angle= 70.45, X_angle = 20, Y_angle = 70, C_angle = 67.5, W_angle = 42.4

  // -- TESTING HOW ANGLES CONVERT TO THE RIGHT SERVO ANGLES, GOAL FIND THE CONSTANT OFFSET -- 
  // I want 70 -> 110 
  //SERVO_RIGHT1.write(moveToTheta); // expecting it to be streight up and down 
  
  // -- TESTING FOR LEFT SERVO OFFSET -- -40 IS MAXHEIGHT 80 IS MIN HEIHGT
  // in graph what do I have to modify X to get what I want Y 
  // I want 10 -> 70 y 
  // I want 20 -> 60
  // I want 30 -> 50
  // I want 40 -> 40
  // I want 50 -> 30
  // I want 60 -> 20
  // I want 70 -> 15
  // I want 80 -> 15. / outlier 
  // I want 90 -> 0
  // I want 100 -> -10 above the flat plain 
  // I want 120 -> -25 above the flat plain 
  /* TESTING 
  int calculated_RightServo_offset = -1.93*(moveToR) + 185.33; // we want to bring back our calculated angle so then the moter can move forward that amount and itll alighn with our K+B Angle
  int rightServoDelta_w_offset = moveToR + calculated_RightServo_offset; // Our outputed angle was -12 of the input, so lets add 12
  SERVO_RIGHT1.write(rightServoDelta_w_offset); // this should be just a constant value for the right to make the measuring of the left servo easier after the testing setup is setup 
  int calculated_LeftServo_offset = -1.77*(moveToTheta) + 69.24; // used least squares regression to find this function
  int leftServoDelta_w_offset = moveToTheta + calculated_LeftServo_offset;
  SERVO_LEFT2.write(leftServoDelta_w_offset);
  SERVO_CLAW.write(moveToZ);
  */

  /* SHAKE MY HAND 
  delay(2000);
  Serial.println("\n3 seconds ");
  delay(3000);

  for (int i=0; i<5; i++){
    SERVO_RIGHT1.write(120);
    SERVO_LEFT2.write(90);
    delay(500);
    SERVO_RIGHT1.write(120);
    SERVO_LEFT2.write(40);
    delay(500);
  }
  */


  // NOTES
  

}

