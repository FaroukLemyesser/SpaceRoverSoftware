// This code enables you to control a space rover with it's gripper using a dualshock 2 controller, using the ps2x library written by http://www.billporter.info

#include <PS2X_lib.h>  //for v1.6

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

#define PS2_DAT        42  //12    
#define PS2_CMD        44  //11
#define PS2_SEL        40  //10
#define PS2_CLK        38  //13

#define pressures   true 
#define rumble      true

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50

PS2X ps2x;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t servon0 = 0; // YAW0
uint8_t servon1 = 2; // YAW1
uint8_t servon2 = 4; // BASE
//uint8_t servon3 = 8; // ELBOW
uint8_t servon4 = 8;// GRIPPER

int YAW0 = 150;
int YAW1 = 30;
int Gripper = 0; 

int PS2 = 0; 

int error = 0;
byte type = 0;
byte vibrate = 0;

int Speed = 180;

// Power transmission to motors is done using relays
    // it's not the best way as it only enables limited movements with not control over the speed, only the direction
//Motor 1 
int MFR = 23;  // Motor Front Right
int MBR = 22;  // Motor Back Right
//Motor 2
int MFL = 25;  // Motor Front Left
int MBL = 24;  // Motor Back Left

void setup(){
  // configuring the ps2 controller 
  PS2 = error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  type = ps2x.readType(); 
  
  
    // Setting up the digital pins to which the relays are connected
    pinMode (MFR, OUTPUT);
    pinMode (MBR, OUTPUT);
    pinMode (MFL, OUTPUT);
    pinMode (MBL, OUTPUT);
    // Ensuring all relays are turned off before starting to avoid unexpected behaviour
    digitalWrite(MFR, LOW);
    digitalWrite(MBR, LOW);
    digitalWrite(MFL, LOW);
    digitalWrite(MBL, LOW);
    
    // Setting up the servo motors of the gripper
    pwm.begin();
    pwm.setPWMFreq(60);
    
    delay(10);
}

void loop(){
  
    ps2x.read_gamepad(false, vibrate);  
    
    // You can test wether the controller is connected if you press the X buttong and the controller vibrates
  
    vibrate = ps2x.Analog(PSAB_CROSS);
    if(ps2x.NewButtonState(PSB_CROSS)) {              //will be TRUE if button was JUST pressed OR released
      
      ps2x.read_gamepad(true, vibrate);
      
      }
    
  
    // Press UP arrow to move forward
    if (ps2x.Button(PSB_PAD_UP)) {
          Forward();} 
    // Press DOWN arrow to move backward
    else if (ps2x.Button(PSB_PAD_DOWN)) {
          Serial.println(MFR);
          Backward();}
    // Press LEFT arrow to move left 
    else if (ps2x.Button(PSB_PAD_LEFT)) {
      Left();}
    // Press RIGHT arrow to move right
    else if (ps2x.Button(PSB_PAD_RIGHT)) {
      Right();} 
    // If no command is sent stop all motors
    else {
      Stop();}
    
    // YAW of the robotic arm 
        // Move the Right Analog joystick UP to move the robotic arm UP 
     while ((ps2x.Analog(PSS_RY) > 225 && YAW0 >= 60 && YAW1 <= 120 )) {
       YAW0 -= 1; 
       YAW1 += 1; 
       pwm.setPWM(servon0, 0, pulseWidth(YAW0));
       pwm.setPWM(servon1, 0, pulseWidth(YAW1)); 
       delay(15); 
    }
        // Move the Right Analog joystick DOWN to move the robotic arm DOWN
     while ((ps2x.Analog(PSS_RY) < 30 && YAW0 <= 120 && YAW1 >= 60)) {
       YAW0 += 1; 
       YAW1 -= 1; 
       pwm.setPWM(servon0, 0, pulseWidth(YAW0));
       pwm.setPWM(servon1, 0, pulseWidth(YAW1)); 
       delay(15);
    }
    // Control the Gripper
    // Press R1 to close the gripper
    while (ps2x.ButtonPressed(PSB_R1) && Gripper >= 0 ) {
      Gripper -= 1;
      pwm.setPWM(servon4, 0, pulseWidth(Gripper));
      delay(15); 
    } 
    // Press R2 to close the gripper
    while (ps2x.ButtonPressed(PSB_R2) && Gripper <= 120) {
      Gripper += 1;
      pwm.setPWM(servon4, 0, pulseWidth(Gripper)); 
      delay(15);
    }

  
  delay(50);

}

// Convert the angle to PWM signal that's understood by the Servo control board (PCA 16 servos module)
int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}


// Move the robot forward
void Forward(){
    
    digitalWrite(MFR, HIGH);
    digitalWrite(MFL, HIGH);
    
    digitalWrite(MBR, LOW);
    digitalWrite(MBL, LOW);
    
}

// Move the robot backward
void Backward(){
    
    digitalWrite(MFR, LOW);
    digitalWrite(MFL, LOW);
    
    digitalWrite(MBR, HIGH);
    digitalWrite(MBL, HIGH);

}

// Move the robot left
void Left(){
    
    digitalWrite(MFR, HIGH);
    digitalWrite(MFL, LOW);
    
    digitalWrite(MBR, LOW);
    digitalWrite(MBL, HIGH);
    
}

// Move the robot right
void Right(){

    digitalWrite(MFR, LOW);
    digitalWrite(MFL, HIGH);
    
    digitalWrite(MBR, HIGH);
    digitalWrite(MBL, LOW);

}

// Stop the robot
void Stop() {
    
    digitalWrite(MFR, LOW);
    digitalWrite(MFL, LOW);
    
    digitalWrite(MBR, LOW);
    digitalWrite(MBL, LOW);
    
}
