#include <AFMotor.h>
#include <Servo.h> 

/**
 * CONST DEFINITIONS
 */
// Motors for wheels
AF_DCMotor leftWheel(1);
AF_DCMotor rightWheel(2);

// Left, Right and Front Mid Turret Servos
Servo leftTurret;
Servo rightTurret;

// Combining all lights to input 7
const int lights = 13;

// Front Door & side door
//const int doors = 7;



/*
END CONST DEFINITIONS
 */

/*
 Setup Pin inputs for controls
 */
void setup()
{
  pinMode(lights, OUTPUT);

  Serial.begin(115200);
}


/*
Behavior for controls
 */
void loop()
{
  char inSerial[5];   
  int i=0; 
  delay(1000);

  if (Serial.available() > 0) 
  {             
    while (Serial.available() > 0) {
      inSerial[i]=Serial.read(); //read data  
      holdCommand(inSerial[i]);
      i++;      
    }
    inSerial[i]='\0';
    assignCommand(inSerial);
  } 
}

/*
Handle a hold command
 */
void holdCommand(char cmd)
{
 
}

/*
handle single command
 */
void assignCommand(char cmd[])
{
  Serial.print("Command: ");
  Serial.println(cmd);

  if(!strcmp(cmd, "1"))//forward
  {
    Serial.println("** Motors FORWARD **");
    toggleMotor(leftWheel, 1);
    toggleMotor(rightWheel, 1);
  }
  else if(!strcmp(cmd,"2"))//backward
  {
    Serial.println("** Motors BACKWARDS **");
    toggleMotor(leftWheel, 2);
    toggleMotor(rightWheel, 2);
  }
  else if(!strcmp(cmd,"3"))//left
  {
    Serial.println("** Motors LEFT **");
    toggleMotor(leftWheel, 1);
    toggleMotor(rightWheel, 2);
  }
  else if(!strcmp(cmd,"4"))//right
  {
    Serial.println("** Motors RIGHT **");
    toggleMotor(leftWheel, 2);
    toggleMotor(rightWheel, 1);
  }
  else if(!strcmp(cmd,"5"))//stop
  {
    Serial.println("** Motors STOP **");
    toggleMotor(leftWheel, 3);
    toggleMotor(rightWheel, 3);
  }
  else if(!strcmp(cmd,"0"))// toggle lights
  { 
    toggleLights(); 
  }
}

/*
motor - AF_FDCMotor object
mDirection - 1: Forward, 2:Backward
*/
void toggleMotor(AF_DCMotor motor, int mDirection)
{
  motor.setSpeed(50);
  
  if(mDirection == 1)
  {
    motor.run(FORWARD);
  }
  else if (mDirection == 2)
  {
    motor.run(BACKWARD);
  }
  else
  {
    motor.run(RELEASE);
  }
}

void toggleLights()
{
  int lightState = digitalRead(lights);
    if(lightState == LOW)
    {
      Serial.println("** Lights ON **");
      digitalWrite(lights, HIGH);
    }
    else 
    {
      Serial.println("** Lights OFF **");
      digitalWrite(lights, LOW);
    }
}
