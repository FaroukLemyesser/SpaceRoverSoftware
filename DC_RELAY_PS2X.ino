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

int UP = 14; 
int DOWN = 15; 
int LEFT = 16; 
int RIGHT = 17;

int error = 0;
byte type = 0;
byte vibrate = 0;

int Speed = 180;

//Motor 1 
int MFR = 23; 
int MBR = 22;
//Motor 2
int MFL = 25; 
int MBL = 24;

void setup(){
  PS2 = error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  type = ps2x.readType(); 

    pinMode (UP, OUTPUT); 
    pinMode (DOWN, OUTPUT); 
    pinMode (LEFT, OUTPUT); 
    pinMode (RIGHT, OUTPUT);

    pinMode (MFR, OUTPUT);
    pinMode (MBR, OUTPUT);
    pinMode (MFL, OUTPUT);
    pinMode (MBL, OUTPUT);
    
    digitalWrite(MFR, LOW);
    digitalWrite(MBR, LOW);
    digitalWrite(MFL, LOW);
    digitalWrite(MBL, LOW);

    pwm.begin();
    pwm.setPWMFreq(60);
    
    delay(10);
}

void loop(){
  
    ps2x.read_gamepad(false, vibrate);  

    /*vibrate = ps2x.Analog(PSAB_CROSS);
    if(ps2x.NewButtonState(PSB_CROSS)) {              //will be TRUE if button was JUST pressed OR released
      
      ps2x.read_gamepad(true, vibrate);
      
      }
      */
    if (ps2x.Button(PSB_PAD_UP)) {
      Forward();
    } else if (ps2x.Button(PSB_PAD_DOWN)) {
          Serial.println(MFR);
      Backward();
    } else if (ps2x.Button(PSB_PAD_LEFT)) {
      Left();
    } else if (ps2x.Button(PSB_PAD_RIGHT)) {
      Right();
    } else {
      Stop();
    }
    
    //Add different moves 
    // YAW
     while ((ps2x.Analog(PSS_RY) < 30 && YAW0 >= 60 && YAW1 <= 120 )) {
       YAW0 -= 1; 
       YAW1 += 1; 
       pwm.setPWM(servon0, 0, pulseWidth(YAW0));
       pwm.setPWM(servon1, 0, pulseWidth(YAW1)); 
       delay(15); 
    }
     while ((ps2x.Analog(PSS_RY) > 225 && YAW0 <= 120 && YAW1 >= 60)) {
       YAW0 += 1; 
       YAW1 -= 1; 
       pwm.setPWM(servon0, 0, pulseWidth(YAW0));
       pwm.setPWM(servon1, 0, pulseWidth(YAW1)); 
       delay(15);
    }


    //BASE SERVO
    /* while (ps2x.Analog(PSS_RX) < 30 && BaseServo <= 150) {
      BaseServo += 1;
      pwm.setPWM(servon2, 0, pulseWidth(BaseServo));
      delay(15); 
    } while (ps2x.Analog(PSS_RX) > 225 && BaseServo >= 30 ) {
      BaseServo -= 1;
      pwm.setPWM(servon2, 0, pulseWidth(BaseServo)); 
      delay(15);
    }*/

    //Gripper
    while (ps2x.ButtonPressed(PSB_R1) && Gripper >= 0 ) {
      Gripper -= 1;
      pwm.setPWM(servon4, 0, pulseWidth(Gripper));
      delay(15); 
    } 
    while (ps2x.ButtonPressed(PSB_R2) && Gripper <= 120) {
      Gripper += 1;
      pwm.setPWM(servon4, 0, pulseWidth(Gripper)); 
      delay(15);
    }

  
  delay(50);

}


int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}

void Forward(){
    
    digitalWrite(MFR, HIGH);
    digitalWrite(MFL, HIGH);
    
    digitalWrite(MBR, LOW);
    digitalWrite(MBL, LOW);
    
}

void Backward(){
    
    digitalWrite(MFR, LOW);
    digitalWrite(MFL, LOW);
    
    digitalWrite(MBR, HIGH);
    digitalWrite(MBL, HIGH);

}

void Left(){
    
    digitalWrite(MFR, HIGH);
    digitalWrite(MFL, LOW);
    
    digitalWrite(MBR, LOW);
    digitalWrite(MBL, HIGH);
    
}

void Right(){

    digitalWrite(MFR, LOW);
    digitalWrite(MFL, HIGH);
    
    digitalWrite(MBR, HIGH);
    digitalWrite(MBL, LOW);

}

void Stop() {
    
    digitalWrite(MFR, LOW);
    digitalWrite(MFL, LOW);
    
    digitalWrite(MBR, LOW);
    digitalWrite(MBL, LOW);
    
}
