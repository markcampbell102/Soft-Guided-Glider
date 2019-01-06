#include "Accelerometer_and_GPU_Control.h"

MPU6050 mpu6050(Wire);

double GyroX, GyroY,GyroZ;
double AccelX, AccelY, AccelZ;
double AccelPitchAngle = 0.0, AccelRollAngle = 0.0, AccelYawAngle = 0.0;

//A variable used to ensure frequency of sensor readings is constant
long FrequencyTimer;

bool InitialSetGyroAngles = false;

unsigned int PIDCounter = 0;

//PID Setup
double PitchSet = 0.0, RollSet = 0.0;
double PitchKp = 1.0, PitchKd = 0.15, PitchKi = 0.0;
double RollKp = 1.0, RollKd = 0.15, RollKi = 0.0;


double PitchOutput, RollOutput; //Output values from the PID that will be sent to the servos
//Servo position
double LeftServo;
double RightServo;

//Set up PID for pitch and yaw
PID PitchPID(&AccelPitchAngle, &PitchOutput, &PitchSet, PitchKp, PitchKi, PitchKd, DIRECT);
PID RollPID(&AccelRollAngle, &RollOutput, &RollSet, RollKp, RollKi, RollKd, DIRECT);

Servo Servo_1; //Servo for left steering
Servo Servo_2; //Servo for right steering

//Servo for wing retraction
Servo Servo_3;

//Collect all data
void ReadSensors(){

  mpu6050.update();
  
  GyroX  = mpu6050.getRawGyroX();
  GyroY  = mpu6050.getRawGyroY();
  GyroZ  = mpu6050.getRawGyroZ();
  /*
  AccelX = mpu6050.getRawAccX();
  AccelY = mpu6050.getRawAccY();
  AccelZ = mpu6050.getRawAccZ();
  */
  AccelX = mpu6050.getAccX();
  AccelY = mpu6050.getAccY();
  AccelZ = mpu6050.getAccZ();
   
  AccelPitchAngle = mpu6050.getAngleX();
  AccelRollAngle  = mpu6050.getAngleY();
  AccelYawAngle   = mpu6050.getAngleZ();
  
}

//Print the required variables for debug purposes
void ShowValues(){


  Serial.print("Pitch = "); Serial.print(AccelPitchAngle);
  Serial.print("\tRoll = ");  Serial.print(AccelRollAngle);
  //Serial.printf(" Roll Angle = "); Serial.println(AccelRollAngle);
  //Serial.printf("  Yaw Angle = ");  Serial.println(AccelYawAngle);
  //Serial.println("");
  
}

void setup() {
  
  Serial.begin(9200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin(); //MPU6050 requires Wire for communication
  mpu6050.begin();
  mpu6050.update();
  mpu6050.calcGyroOffsets(true); //Take x readings, adding each one then divide the total by x. Take that value away from each further reading.

  Serial.println();
  Serial.println("Levelling system has been calibrated.");
  ReadSensors();

  //Connect all three servos (2 for steering and one for reeling in the folding wings
  Servo_1.attach(3);
  Servo_2.attach(2);
  Servo_3.attach(4); //wheeling control

  //Output limits ensure that values will not exceed that which the servo can take.
  PitchPID.SetOutputLimits(-90, 90);
  PitchPID.SetSampleTime(20);
  PitchPID.SetMode(AUTOMATIC);

  RollPID.SetOutputLimits(-90, 90);
  RollPID.SetSampleTime(20);
  RollPID.SetMode(AUTOMATIC);

  delay(1000);

}

void loop() {

  Serial.println("Hello World");
  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  //Routine for wing release
  ReadSensors();
  float MagAccel, PrevMagAccel, DifferenceAccel;
  //Collect the magnitude of acceleration
  PrevMagAccel = sqrt((AccelX * AccelX) + (AccelY*AccelY) + (AccelZ*AccelZ));

  //Wait until there is a large enough difference in accelerometer readings to release the wings
  while(1){

    ReadSensors();
    MagAccel = sqrt((AccelX * AccelX) + (AccelY*AccelY) + (AccelZ*AccelZ));

    DifferenceAccel = abs(MagAccel - PrevMagAccel);

    if (DifferenceAccel > 0.4){

      Serial.print("Difference = ");
      Serial.println(DifferenceAccel);
      //Go to servo position to release the wings
      Servo_3.write(135);
      break;
            
    }   

    delay(100);

  }
  
  Serial.println("Left first loop");
  delay(1000);
  Servo_3.write(10);
  delay(1000);
  
  //Enter stabilisation control loop
  while(1){

    FrequencyTimer = micros(); //micros is the amount of uS that the system has been running.

    ReadSensors();

    ShowValues();

    PitchPID.Compute();
    RollPID.Compute();

    //LeftServo = PitchOutput + 90;
    LeftServo  = (0.5 * RollOutput) + (0.5 * PitchOutput) + 90;
    RightServo = (0.5 * RollOutput) - (0.5 * PitchOutput) + 90;

    //Ensure that values sent to the servos will not be out of their working range
    if (LeftServo > 160){

      LeftServo = 160;
      
    }
    if (LeftServo < 45){

      LeftServo = 45;
      
    }
    if (RightServo > 160){

      RightServo = 160;
      
    }
    if (RightServo < 45){

      RightServo = 45;
      
    }

    Serial.print("\tPitchOut = "); Serial.print(PitchOutput);
    Serial.print("\tRollOut = "); Serial.println(RollOutput);
    Serial.println();
    //Go to calculated servo positions
    Servo_2.write(LeftServo);
    Servo_1.write(RightServo);

    while((micros() - FrequencyTimer) < 20000){ //wait until frequency of 50Hz (1/20000) has been reached.
    
    }
    //Take next frequency timer reading to reset it
    FrequencyTimer = micros();
    
  }
  
}
