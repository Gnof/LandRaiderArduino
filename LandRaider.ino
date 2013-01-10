#include <AFMotor.h>
#include <Servo.h> 

/**
 * CONST DEFINITIONS
 */
// Motors for wheels
AF_DCMotor leftWheel(3);
AF_DCMotor rightWheel(4);

// Left, Right and Front Mid Turret Servos
Servo leftTurret;
int leftpos = 0;
Servo rightTurret;
int rightpos = 0;

// 1: active, 0:kill
int turretOn = 1;

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
    Serial.println("-- Left Turret SWEEP LEFT --");
    while(turretOn == 1)
    {
      if(leftpos >= 0)
      {
        leftpos = leftpos - 1;
        leftTurret.write(leftpos);
        delay(20);
      }
      else 
      {
        turretOn =0;
      }
    }
    turretOn = 1;
  }
  else if(!strcmp(cmd, "w"))
  {
    if(leftpos >=0)
    {
      leftpos = leftpos - 30;
      leftTurret.write(leftpos);
    }
  }
  else if(!strcmp(cmd, "e"))
  {
    if(leftpos <= 180)
    {
      leftpos = leftpos + 30;
      leftTurret.write(leftpos);
    }
  }
  else if(!strcmp(cmd, "r"))// left servo right
  {
    Serial.println("-- Left Turret SWEEP RIGHT --");
    while(turretOn == 1)
    {
      if(leftpos <= 180)
      {
        leftpos = leftpos + 1;
        leftTurret.write(leftpos);
        delay(20);
      }
      else 
      {
        turretOn =0;
      }
    }
    turretOn = 1;
  }
  else if(!strcmp(cmd, "a"))
  {
    Serial.println("-- Right Turret SWEEP LEFT --");
    while(turretOn == 1)
    {
      if(rightpos >= 0)
      {
        rightpos = rightpos - 1;
        rightTurret.write(rightpos);
        delay(20);
      }
      else 
      {
        turretOn =0;
      }
    }
    turretOn = 1;
  }
  else if(!strcmp(cmd, "s"))// right servo left
  {
    if(rightpos >= 0)
    {
      rightpos = rightpos - 30;
      rightTurret.write(rightpos);
    }
  }
  else if(!strcmp(cmd, "d"))// right servo right
  {
    if(rightpos <= 180)
    {
      rightpos = rightpos + 30;
      rightTurret.write(rightpos);
    }
  }
  else if(!strcmp(cmd, "f"))
  {
    Serial.println("-- Right Turret SWEEP RIGHT --");
    while(turretOn == 1)
    {
      if(rightpos <= 180)
      {
        rightpos = rightpos + 1;
        rightTurret.write(rightpos);
        delay(20);
      }
      else 
      {
        turretOn =0;
      }
    }
    turretOn = 1;
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
  motor.setSpeed(200);
  
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
