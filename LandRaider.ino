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
int leftpos = 0;
Servo rightTurret;
int rightpos = 0;

// Combining all lights to input 7
const int lights = 13;

const int laser = A5;

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
  // Initialize Servos
  leftTurret.attach(9);
  rightTurret.attach(10);
  
  // Set light output pin
  pinMode(lights, OUTPUT);
  
  pinMode(laser, OUTPUT);

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
  else if(!strcmp(cmd, "q")) //left servo left
  {
    Serial.println("-- Left Turret LEFT --");
    if(leftpos >= 20)
    {
      leftpos = leftpos - 20;
      leftTurret.write(leftpos);
    } 
  }
  else if(!strcmp(cmd, "w"))// left servo right
  {
    Serial.println("-- Left Turret RIGHT --");
    if(leftpos <= 160)
    {
      leftpos = leftpos + 20;
      leftTurret.write(leftpos);
    }
  }
  else if(!strcmp(cmd, "a"))// right servo left
  {
    Serial.println("-- Right Turret LEFT --");
    if(rightpos >= 20)
    {
      rightpos = rightpos - 20;
      rightTurret.write(rightpos);
    }
  }
  else if(!strcmp(cmd, "s"))// right servo right
  {
    Serial.println("-- Right Turret RIGHT --");
    if(rightpos <= 160)
    {
      rightpos = rightpos + 20;
      rightTurret.write(rightpos);
    }
  }
  else if (!strcmp(cmd,"9"))
  {
    Serial.println("** Laser toggle **");
    toggleInput(laser);
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

/*
Toggle an input
*/
void toggleInput(int input)
{
  int state = digitalRead(input);
  if(state == LOW)
  {
    digitalWrite(input, HIGH);
  }
  else
  {
    digitalWrite(input, LOW);
  }
}

/*
Toggle LEDs
*/
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
