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
int moveToZ         = 100;
int moveToGripper   = 35;

int prevSafe_moveToTheta     = 0;
int prevSafe_moveToR         = 100;
int prevSafe_moveToZ         = 100;
int prevSafe_moveToGripper  = 35;

/*
 * Functions (
 */

// Move the rotational (theta, polar coordinates) axis of the MeArm
void moveTheta(int angle) {
  
}

// Open the gripper a given width/distance
void moveGripper(int distToOpen) {
  // Minimum Angle = 30 -> Minimum open = 90
  // Maximum Angle = 145 -> Maximum close 0
  int angleToOpen = (distToOpen -117.98)/(-0.80); // used a least squares regression line to best fit the data of 

  SERVO_CLAW.write(angleToOpen); // measure the points
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

  int calculated_RightServo_offset = -1.93*(right_Servo1Delta) + 185.33; // we want to bring back our calculated angle so then the moter can move forward that amount and itll alighn with our K+B Angle
  int rightServoDelta_w_offset = right_Servo1Delta + calculated_RightServo_offset; // Our outputed angle was -12 of the input, so lets add 12
  SERVO_RIGHT1.write(rightServoDelta_w_offset); // this should be just a constant value for the right to make the measuring of the left servo easier after the testing setup is setup 

  int calculated_LeftServo_offset = -1.77*(left_Servo2Delta) + 69.24; // used least squares regression to find this function
  int leftServoDelta_w_offset = left_Servo2Delta + calculated_LeftServo_offset;
  SERVO_LEFT2.write(leftServoDelta_w_offset);
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

/* verifyInputValues - This function will confirm the global `moveToTheta`,
 * `moveToR`,`moveToZ`, and `moveToGripper` variables are within working 
 * tested boundries. If for some reason a variable is not within its valid 
 * boundries. The plan is to set them back to a previously set valid value.
 * The corrected values will be outputed to the user, also if any ended up
 * getting changed it will show the updated value, and the unsafe value.
 * 
 * The prevously valid values are stored in variables with prefixs of "prevSafe_varname".
 * If any input variables are safe then that `prevSafe_varname` value gets 
 * updated to the current.
 * 
 * Notes on valid ranges
    MAX of R+Z = 120+100 = 220
    MAX of R = 120, Nah 150
      nah its able to go past 120 if z is lower,
    MIN of R = 5, Nah 30
      If you want the claw to be straight at a super low z=0, then r=30
    
    MAX of Z = 150
    MIN of Z = 0

    Gripper Minimum Angle = 30 therefore Minimum open is 90
    Gripper Maximum Angle = 149 therefore Minimum close is 0
*/
void verifyInputValues(){
  // Step 2: Debug display
  Serial.println("");
  Serial.println("Moving to: ");
  
  //                      ------- CHECKS FOR VALID RANGE ------
  Serial.print("Theta: ");
  Serial.println(moveToTheta);
  // if R is a invalid range, print out the prevSafe R value and the unsafe value to let them know 
  if (moveToR < 30 || 150 < moveToR){
    Serial.print("R: ");
    Serial.print(prevSafe_moveToR);
    Serial.print(" ("); Serial.print(moveToR); Serial.println(" unsafe)");
    moveToR = prevSafe_moveToR;
  } else {Serial.print("R: "); Serial.println(moveToR);} // else print out the normal value

  // if Z is a invalid range. print out the prevSafe Z value and the unsafe value to let them know 
  if (150 < moveToZ){
    Serial.print("Z: ");
    Serial.print(prevSafe_moveToZ);
    Serial.print(" ("); Serial.print(moveToZ); Serial.println(" unsafe)");
    moveToZ = prevSafe_moveToZ;
  } else {Serial.print("Z: "); Serial.println(moveToZ);} // else print out the normal value

  // if after updating atleast 1 R or Z var its still more than 220 revert them both back to prevSafe
  if (220 < moveToR + moveToZ){
    Serial.println("!! Error, R+Z > 220 !!");
    Serial.print("R: ");
    Serial.print(prevSafe_moveToR);
    Serial.print(" ("); Serial.print(moveToR); Serial.println(" unsafe)");
    Serial.print("Z: ");
    Serial.print(prevSafe_moveToZ);
    Serial.print(" ("); Serial.print(moveToZ); Serial.println(" unsafe)");
    moveToR = prevSafe_moveToR;
    moveToZ = prevSafe_moveToZ;
  }

  if (moveToGripper < 0 || 90 < moveToGripper){
    Serial.print("Gripper: ");
    Serial.print(prevSafe_moveToGripper);
    Serial.print(" ("); Serial.print(moveToGripper); Serial.println(" unsafe)");
    moveToGripper = prevSafe_moveToGripper;
  } else {Serial.print("Gripper: "); Serial.println(moveToGripper);} // else print out the normal value
  //                      ------- END ------

  //                      ------- UPDATE VALUES IF THEY ARE SAFE ------
  // if the new moveToX values are different than their prevSafe then that means they were valid, update the prevSafe to these new cords
  bool moveToRIsDiff = moveToR != prevSafe_moveToR;
  bool moveToZIsDiff = moveToZ != prevSafe_moveToZ;
  bool moveToGripperIsDiff = moveToGripper != prevSafe_moveToGripper;
  if (moveToRIsDiff) prevSafe_moveToR = moveToR;
  if (moveToZIsDiff) prevSafe_moveToZ = moveToZ;
  if (moveToGripper) prevSafe_moveToGripper = moveToGripper;  

} // end of verifyInputValues()

void loop() {
  
  // Step 1: Display Serial console
  doSerialConsole();

  // Step 1a: Check the user input for valid ranges 
  // Step 2: Also Debug display
  verifyInputValues();

  // Step 3: Move to requested location
  //moveTheta(moveToTheta);
  moveRZ(moveToR, moveToZ);
  moveGripper(moveToGripper);  
 

  
  /* -- SHAKE MY HAND CODE --
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

}

