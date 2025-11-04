/* 
  Assignment 3 (MeArm) Scaffold
  Kinematics code for MeArm
 
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

int gripperDirectionRelationship = 1; // 1 is forward, 2 is for sideways, 3 is for downward

int prevSafe_moveToTheta     = 90; // default from the robot arm setup 
int prevSafe_moveToR         = 100;
int prevSafe_moveToZ         = 100;
int prevSafe_moveToGripper  = 35; // default from the robot arm setup 

// --- Kinimatics Maximimums/ Minimumns Values ---
int r_min = 30; // reasons described in `veriftyInputValues()` docstring
int r_max = 150; 
int z_max = 150;
int z_min = 0;
int rPlusz_max = 219;
int gripper_min = 0;
int gripper_max = 90;
int theta_min = 10; // randomly set, I think I could set it to 0 and it would work
int theta_max = 160; // randomly set, I think I could set it to 160 and it would work

int r_midpoint = (r_max-r_min)/2;
int z_midpoint = (z_max-z_min)/2;
int theta_midpoint = (theta_max-theta_min)/2;
int gripper_midpoint = (gripper_max-gripper_min)/2;

/*
 *  --FORWARD SESNOR PINS--
 *    5V    |   VCC     
 *    7     |   Trig     
 *    8     |   Echo     
 *    GND   |   GND
 */
// Pins
const int FORWARD_SENSOR_TRIG_PIN = 7; // This is the Trigger pin of the forward sensor 
const int FORWARD_SENSOR_ECHO_PIN = 8; // This is the Echo pin for the forward sensor 
 /*
 *  --DOWNWARD SESNOR PINS--
 *    5V    |   VCC     
 *    5     |   Trig     
 *    4     |   Echo     
 *    GND   |   GND
 */
const int DOWNWARD_SENSOR_TRIG_PIN = 5; // This is the Trigger pin of the downward sensor 
const int DOWNWARD_SENSOR_ECHO_PIN = 4; // This is the Echo pin for the downward sensor

 /*
 *  --SIDEWAYS SESNOR PINS--
 *    5V    |   VCC     
 *    3     |   Trig     
 *    2     |   Echo     
 *    GND   |   GND
 */
const int SIDEWAYS_SENSOR_TRIG_PIN = 3; // This is the Trigger pin of the sideways sensor 
const int SIDEWAYS_SENSOR_ECHO_PIN = 2; // This is the Echo pin for the sideways sensor

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200; // this is set by the code given for radar servo project 2


// ---- NOTE ---- 
// All the code below is going to be the attempt to implement the hand drivin moter controls for the robot 
// in the FORWARD DIRECTION ONLY using the R value. After the Forward direction is implemented, its rather easy (copy/paste) to implement the other directions.

int finiteState = 1;
int motionBufferCM = 1; // 2cm, our hand needs to move atleast 2cm away form the previous location it moved to actuall move, this sets that 2cm threshold
int recalibrateThresholdDistCM = 68; // (Randomly set) We know the user is outside the box because 68cm is way to large to be inside a box 

float forwardPercentLocation; // 100% by default, Corrilates the forward change in distance to a percentage. Then use this in relation to the middle point of the maximum value of R to determine how far the robot should move forward or backwards. Defaults to 100% because we base the percentage on the current measurement taken and the middle point. That way we can take the percentage and give it to the robot and it will use its own middle point and times it by this percentage
float calibratedForwardDistanceCM; // -> Holds the cm distance of the wall infront of the hand after calibration, calibration SHOULD consist of putting the hand device in the MIDDLE of the box. This will serve as the middle (neutral) point of the controls for forward direction.
int prevForwardMeasurement; // -> Holds the previous measurement of the forward sensor, we will check this varaible comparing new measurements to see if the forward distance has changed, Defaults to the result of the calibrated forward distance

float downwardPercentLocation; // define in state 1
float calibratedDownwardDistanceCM; // defined in state 2 
int prevDownwardMeasurement;

float sidewaysPercentLocation; // define in state 1 
float calibratedSidewaysDistanceCM; // difined in state 2 
int prevSidewaysMeasurement;





/*
 * moveTheta(angle)
 */
// Move the rotational (theta, polar coordinates) axis of the MeArm
void moveTheta(int angle) {
  SERVO_MIDDLE.write(angle);
}

// Open the gripper a given width/distance you want open in mm
void moveGripper(int distToOpen) {
  // Minimum Angle = 30 -> Minimum open = 90
  // Maximum Angle = 145 -> Maximum close 0
  int angleToOpen = (distToOpen -117.98)/(-0.80); // used a least squares regression line to best fit the data of, The data for this is on desmos 

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

  int calculated_RightServo_offset = -1.93*(right_Servo1Delta) + 185.33; // we want to bring back our calculated angle so then the moter can move forward that amount and itll alighn with our K+B Angle, The data for this in is desmos 
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

  // Setup ultrasonic sensor pin modes
  pinMode(FORWARD_SENSOR_TRIG_PIN, OUTPUT);  
  digitalWrite(FORWARD_SENSOR_ECHO_PIN, LOW);

  pinMode(DOWNWARD_SENSOR_TRIG_PIN, OUTPUT);  
  digitalWrite(DOWNWARD_SENSOR_ECHO_PIN, LOW);

  pinMode(SIDEWAYS_SENSOR_TRIG_PIN, OUTPUT);  
  digitalWrite(SIDEWAYS_SENSOR_ECHO_PIN, LOW);
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
    Serial.println("Enter coordinates: gripperDirectionRelationship,r,z,gripper (comma delimited, no spaces)");
    Serial.println("gripperDirectionRelationship:0 Off, 1 Forward, 2 Sideways, 3 Downwards");
    Serial.println("Example: 1,20,30,40");

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
    if (sscanf(inputStr, "%d,%d,%d,%d", &gripperDirectionRelationship, &moveToR, &moveToZ, &moveToGripper) == 4) {  
      // check that the inputed gripper Relationship direction is within the valid range. Which is only 3 sensors so 0-3 (0 is Off)
      if (gripperDirectionRelationship<0 || 3<gripperDirectionRelationship){
        Serial.println ("Invalid gripper Direction Relationship -- try again.  Acceptable 0-3");   
      }
      else{
        Serial.println("Valid input!");  
        // Valid string
        return;
      }
    } else {
      // Invalid string -- restart
      Serial.println ("Invalid input -- try again.  example: 1,20,30,40");      
    }      
  }    
}

/* measureDistanceCM() -- This function is for the ultrasonic sensor, it will
 * return value in centimeters. 
 * (This code is from the open source HC-SR04 Arduino driver
 * modified from a forum post for the BangGood version of this sensor). 
 */
float measureDistanceCM(int thisTRIG_PIN, int thisECHO_PIN) {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;
  int TRIG_PIN = thisTRIG_PIN;
  int ECHO_PIN = thisECHO_PIN;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  unsigned long wait1 = micros();
  while ( digitalRead(ECHO_PIN) == 0  ) {
    if ( micros() - wait1 > MAX_DIST ) {
        //Serial.println("wait1 Out of range");
        return -1;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
    while ( digitalRead(ECHO_PIN) == 1  ) {
    if ( micros() - t1 > MAX_DIST ) {
        //Serial.println("wait2 Out of range");
        return -1;
    }
  } 
  
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  // of sound in air at sea level (~340 m/s).
  // These constants where found the the data sheet.
  cm = pulse_width / 58;
  inches = pulse_width / 148;

  // Wait at least 60ms before next measurement
  delay(60);

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    //Serial.println("Out of range");
    return 400.0;
  } else {    
    //Serial.print(cm);
    //Serial.print(" cm \t");
    //Serial.print(inches);
    //Serial.println(" in");    
    return cm;
  }
}

float measureDist(int thisTRIG_PIN, int thisECHO_PIN){
  // Step 2: Measure range    
  float range = -1;  
  while (range == -1) {
    range = measureDistanceCM(thisTRIG_PIN, thisECHO_PIN);
    Serial.print("measureDist: Range: ");
    Serial.println(range);
  }
  return range;
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
    MAX of R+Z = 120+100 = 220, nah r=150, and z= 70, breaks it, nah 219
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
  // if theta is a INvalid range, print out the prevSafe thet, and let the user know 
  if (moveToTheta < theta_min || theta_max < moveToTheta){
    Serial.print("Theta: ");
    Serial.print("prevSafe_moveToTheta");
    Serial.print(" ("); Serial.print(moveToTheta); Serial.println(" unsafe)");
    moveToTheta = prevSafe_moveToTheta;
  } else {Serial.print("Theta: "); Serial.println(moveToTheta);} // else print out the new value we are going to move to 

  // if R is a invalid range, print out the prevSafe R value and the unsafe value to let the user know 
  if (moveToR < r_min || r_max < moveToR){
    Serial.print("R: ");
    Serial.print(prevSafe_moveToR);
    Serial.print(" ("); Serial.print(moveToR); Serial.println(" unsafe)");
    moveToR = prevSafe_moveToR;
  } else {Serial.print("R: "); Serial.println(moveToR);} // else print out the new value we are going to move to 

  // if Z is a invalid range. print out the prevSafe Z value and the unsafe value to let the user know 
  if (z_max < moveToZ){
    Serial.print("Z: ");
    Serial.print(prevSafe_moveToZ);
    Serial.print(" ("); Serial.print(moveToZ); Serial.println(" unsafe)");
    moveToZ = prevSafe_moveToZ;
  } else {Serial.print("Z: "); Serial.println(moveToZ);} // else print out the new value we going to move to 

  // if after updating atleast 1 R or Z var its still more than 220 revert them both back to prevSafe, and let the user know
  if (rPlusz_max < moveToR + moveToZ){
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

  if (moveToGripper < gripper_min || gripper_max < moveToGripper){
    Serial.print("Gripper: ");
    Serial.print(prevSafe_moveToGripper);
    Serial.print(" ("); Serial.print(moveToGripper); Serial.println(" unsafe)");
    moveToGripper = prevSafe_moveToGripper;
  } else {Serial.print("Gripper: "); Serial.println(moveToGripper);} // else print out the normal value
  //                      ------- END ------

  //                      ------- UPDATE VALUES IF THEY ARE SAFE ------
  // if the new moveToX values are different than their prevSafe then that means they were valid, update the prevSafe to these new cords
  bool moveToRIsDiff       = moveToR != prevSafe_moveToR;
  bool moveToZIsDiff       = moveToZ != prevSafe_moveToZ;
  bool moveToGripperIsDiff = moveToGripper != prevSafe_moveToGripper;
  bool moveToThetaIsDiff   = moveToTheta != prevSafe_moveToTheta;
  if (moveToRIsDiff) prevSafe_moveToR = moveToR;
  if (moveToZIsDiff) prevSafe_moveToZ = moveToZ;
  if (moveToGripperIsDiff) prevSafe_moveToGripper = moveToGripper;  
  if (moveToThetaIsDiff) prevSafe_moveToTheta = moveToTheta;

} // end of verifyInputValues()

/* newDistIsDifferent(newMeasuredDist, prevDistance) 
 * This function will be given a new measured distance from a sensor in cm 
 * and it will check if it is different ENOUGH compared to the prevDistance + a threshold varaible
 * The threshold variable is used because the sensors may jump around alot in values, and this will be a error reducer. 
*/
boolean newDistIsDifferent(int newMeasuredDist, int prevDistance){
  // if the new distance measured is less than the prev distance
  boolean newDistLessThanOld = newMeasuredDist < prevDistance - motionBufferCM;
  boolean newDistGreaterThanOld = prevDistance+motionBufferCM < newMeasuredDist;
  return newDistLessThanOld || newDistGreaterThanOld;
}

void loop() {
  // STAGE 0 TESTING STAGE 
  if (finiteState == 0){
    Serial.println("Testing State is done");
    finiteState = 1;
  }


  // Set all global constants. Wait for the user input to start calibration
  if (finiteState == 1){
    Serial.println("FiniteState1");
    forwardPercentLocation = 1; // 100% is the middle point (in the future we will do `thisVar * middleOfR`)
    downwardPercentLocation = 1;
    sidewaysPercentLocation = 1;
    // Step 1: Display Serial console, waiting for the user to enter anything to let us know to start calibration
    doSerialConsole();
    finiteState = 2;
  }

  // Calibration, do the calibration (Not sure what the entails yet)
  // Idea; record data for 3 seconds, find the median of that set of values
  if (finiteState == 2){
    Serial.println("Currently at FiniteState2");
    // --- TESTING ---
    calibratedForwardDistanceCM = 15; // 15cm. We will assume that the center place of the hand is at 15 cm
    prevForwardMeasurement = calibratedForwardDistanceCM; // defaults to the calibrated middle

    calibratedDownwardDistanceCM = 15; // 15cm 
    prevDownwardMeasurement = calibratedDownwardDistanceCM;

    calibratedSidewaysDistanceCM = 15; // 15cm 
    prevSidewaysMeasurement = calibratedSidewaysDistanceCM;

    // ^^^^ TESTING 
    finiteState = 3;
  }

  /* Measure Distances. Check if they are different
   * if they are really different, reset to before calibration, 
   * if they are regularly different, do calculations and call the move Functions
   */
  if (finiteState == 3){
    Serial.println("Currently at FiniteState3");
    // Make left, forward, and down measurements
    boolean humanMovedTheDevice = false; // if the human move the handheld device then we want to move the robot, so move to state 4 
    int newForwardMeasurement  = measureDist(FORWARD_SENSOR_TRIG_PIN,FORWARD_SENSOR_ECHO_PIN);
    int newDownwardMeasurement = measureDist(DOWNWARD_SENSOR_TRIG_PIN, DOWNWARD_SENSOR_ECHO_PIN); 
    int newSidewaysMeasurement = measureDist(SIDEWAYS_SENSOR_TRIG_PIN, SIDEWAYS_SENSOR_ECHO_PIN); 

    // we only want to change our motors location if a distance measured has changed

    // -- CHECKING FOR FORWARD DISTANCE CHANGING
    if (newDistIsDifferent(newForwardMeasurement, prevForwardMeasurement)){
      humanMovedTheDevice = true;
      // ---- HOW TO RESET EVERYTHING --- 
      // if the new distance is differnt enough that its greater than the recalibrateThresholdDist which is a number set so large its unrealist to have inside of a box, then we should go back to the start.
      // imagine the user rotates to far making a sesor read outside the box, or the user removes his hand and the device from the box, then it should reset and stop trying to move the robot 
      if (newForwardMeasurement > recalibrateThresholdDistCM){
        Serial.print("Forward sensor read a measurement (");
        Serial.print(newForwardMeasurement);
        Serial.println(") out of bounds. Going to finiteState 1.");
        finiteState = 1;
        return;
      }
      else{ // if we got a valid movement inside the box, lets move the robot 
        float tempCalc = newForwardMeasurement / calibratedForwardDistanceCM;
        // We assume the user puts the hand device in the middle of the box so the percentage change cannot be more than 200% that is what this 2 represetnts. So this movement has to be between 
        if (0 <= tempCalc && tempCalc < 2){
          forwardPercentLocation = tempCalc; // This is what we should times our middle point by to get a new location 
        }
        
      }
    }
    
    // -- CHECKING FOR Downward DISTANCE CHANGING
    if (newDistIsDifferent(newDownwardMeasurement, prevDownwardMeasurement)){
      humanMovedTheDevice = true;
      // ---- HOW TO RESET EVERYTHING --- 
      // if the new distance is differnt enough that its greater than the recalibrateThresholdDist which is a number set so large its unrealist to have inside of a box, then we should go back to the start.
      // imagine the user rotates to far making a sesor read outside the box, or the user removes his hand and the device from the box, then it should reset and stop trying to move the robot 
      if (newDownwardMeasurement > recalibrateThresholdDistCM){
        Serial.print("Downward sensor read a measurement (");
        Serial.print(newDownwardMeasurement);
        Serial.println(") out of bounds. Going to finiteState 1.");
        finiteState = 1;
        return;
      }
      else{ // if we got a valid movement inside the box, lets move the robot 
        float tempCalc = newDownwardMeasurement / calibratedDownwardDistanceCM;
        // We assume the user puts the hand device in the middle of the box so the percentage change cannot be more than 200% that is what this 2 represetnts. So this movement has to be between 
        if (0 <= tempCalc && tempCalc < 2){
          downwardPercentLocation = tempCalc; // This is what we should times our middle point by to get a new location 
        }
        
      }
    }

    // -- CHECKING FOR SIDEWAYS DISTANCE CHANGING
    if (newDistIsDifferent(newSidewaysMeasurement, prevSidewaysMeasurement)){
      humanMovedTheDevice = true;
      // ---- HOW TO RESET EVERYTHING --- 
      // if the new distance is differnt enough that its greater than the recalibrateThresholdDist which is a number set so large its unrealist to have inside of a box, then we should go back to the start.
      // imagine the user rotates to far making a sesor read outside the box, or the user removes his hand and the device from the box, then it should reset and stop trying to move the robot 
      if (newSidewaysMeasurement > recalibrateThresholdDistCM){
        Serial.print("Sideways sensor read a measurement (");
        Serial.print(newSidewaysMeasurement);
        Serial.println(") out of bounds. Going to finiteStatee 1.");
        finiteState = 1;
        return; // we have to return here or else it will continue to finite state 4 
      }
      else{ // if we got a valid movement inside the box, lets move the robot 
        float tempCalc = newSidewaysMeasurement / calibratedSidewaysDistanceCM;
        // We assume the user puts the hand device in the middle of the box so the percentage change cannot be more than 200% that is what this 2 represetnts. So this movement has to be between 
        if (0 <= tempCalc && tempCalc < 2){
          sidewaysPercentLocation = tempCalc; // This is what we should times our middle point by to get a new location 
        }
        
      }
    }

    if (humanMovedTheDevice) finiteState = 4;
  } // ----- END OF STATE 3 -----

  /* State 4: Move the robot to the new location of the hand held device 
   * 
   * Assumption:
   *    We assume that the `directionPercentLocation` is always within the range 0<var<2
  */
  if (finiteState == 4){
    Serial.println("Currently At FiniteState4");
    // Calculate what R and Z we should pass in.
    // R = middle point of robot times forwardPercentLocation
    moveToR = r_midpoint * forwardPercentLocation; // update the global variables
    moveToZ = z_midpoint * downwardPercentLocation;
    moveToTheta = theta_midpoint * sidewaysPercentLocation;

    int tempRelation = 1;
    if (gripperDirectionRelationship == 1) tempRelation = forwardPercentLocation;  // 1-> Forward
    if (gripperDirectionRelationship == 2) tempRelation = sidewaysPercentLocation; // 2-> Sideways 
    if (gripperDirectionRelationship == 3) tempRelation = downwardPercentLocation; // 3-> Downward
    
    moveToGripper = gripper_midpoint * tempRelation;


    // ASSUMTION: The directionPercentLocation is assumed that it is within the range 

    // TODO calculate the move theta, and gripper 

    verifyInputValues();

    moveRZ(moveToR,moveToZ);
    moveTheta(moveToTheta);
    moveGripper(moveToGripper);
    finiteState = 3;
  }
 

  
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

