//############################################################################//
//                             SuperPowerWheels v.1                           //
//                 Brought to you by Milligan University Engineering          //
//############################################################################//


// Include libraries for PS4 Bluetooth and USB Host Shield Support


#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>
#include <SoftwareSerial.h>


USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // Create the Bluetooth Dongle instance

// Create the instance of the PS4BT class
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);



// Arduino pin assignments
  /*
   * NOTE: Pins 13, 12, 11 are hardcoded for the USB Host Shield SPI Communication. They cannot be changed.
   *       Pins 10, 9 are used SS and INT respectively. This can be changed in UsbCore.h 
   *       typedef MAX3421e<P10, P9> MAX3421E;
   */
#define SERIAL_OUT            8
#define SERIAL_IN             7

#define BTN_ESTOP             2
#define BTN_FORWARD           4
#define BTN_REVERSE           3
#define BTN_LEFT              6
#define BTN_RIGHT             5

#define POT_GOVERNOR         A0
#define POT_ACCELERATION     A1

#define POT_TRIM             A2

#define LED_HEARTBEAT        A3
#define LED_ESTOP            A4

#define MAG_SWITCH           A5

#define RELAY_FUN1            0
#define RELAY_FUN2            1

//##############################################################################
// The values in this section only can be modified to alter the behavior of the
// individual ride-on toy.

#define USE_ACCELERATION_POT  false  //If there is no installed acceleration potentiometer, set to 'false'



// This value dictates how quickly scootypuff gains speed when responding to
// a button press. The higher the value, the faster the acceleration.
#define BASE_ACCELERATION     1



// This value dictates how quickly scootypuff loses speed when no buttons are
// being pressed. The higher the value, the faster the acceleration.
#define BASE_DECELERATION     2



// Whether the FUN1/2 relays are in use. Set each to 'true' or 'false.'
#define RELAY_FUN1_ENABLED    true
#define RELAY_FUN2_ENABLED    false



// When INVERT is true, the corresponding relay control pin will go LOW when
// fun is to be engaged. When INVERT is false, it will go HIGH instead. If
// you find your relay is energized when it should be at rest (and vice-versa),
// changing this setting should clear that up.

#define RELAY_FUN1_INVERT     true  // The Ywrobot 1 relay is a LOW activated relay thus this setting should be true
#define RELAY_FUN2_INVERT     true



// The buttons that activate FUN1 relays (as well as performing their usual
// function). Use the Arduino pin definition names above (eg, 'BTN_FORWARD').
#define RELAY_FUN1_TRIGGER    BTN_FORWARD
#define RELAY_FUN2_TRIGGER    BTN_REVERSE



// How long FUN1/2 relays should be active when triggered, in milliseconds.
#define RELAY_FUN1_ON_TIME    5000
#define RELAY_FUN2_ON_TIME    500



// The minimum off time (in milliseconds) between triggers of FUN1/2; gives toys
// a chance to go through their motions and/or prevents continuous operation.
#define RELAY_FUN1_COOLDOWN   30000
#define RELAY_FUN2_COOLDOWN   30000



//##############################################################################

#define MOTOR1_STOP           64    //WHY? Ratio of 1:3, motor 1 to motor 2
#define MOTOR2_STOP           192

// communication speed between the Arduino and the Sabertooth motor controller
#define BAUD_RATE             38400
#define LOOP_DELAY            10  // milliseconds

//##############################################################################
//#define USB_XFER_TIMEOUT 50000
//#define USB_RETRY_LIMIT 1000
//#define USB_SETTLE_DELAY 200

//##############################################################################

enum Direction{ FORWARD, REVERSE };

typedef struct { //defining a motor "structure" using the "struct" command in C
  char *name;    //Using "typedef" defines the new data type of "motor"
  int speed;
  Direction direction;
  byte center_throttle;
  byte trim;
} Motor;

typedef struct {  //defining a Relay "structure" using the "struct" command in C
  bool active;    // active is either "on or "off"
  unsigned long cool_at; //"unsigned long" stores positive integers 4 bytes/32 bits long
  unsigned long disable_at;
} Relay;

//##############################################################################

// global variables
SoftwareSerial soft_serial(SERIAL_IN,SERIAL_OUT);
bool estopped; //
bool magstop;
bool dpadInput;
bool dualAnalogInput;
bool timeoutChildControl;
byte max_speed; //"byte" means max_speed will be any number 0->255
byte acceleration = BASE_ACCELERATION;
byte deceleration = BASE_DECELERATION;
Motor left_motor;  // here we are assigning new variables to the struct "motors" that will (cont next line) 
Motor right_motor; //(cont) include the data points contained in the struct "motor"
Relay fun1; // here we are assigning new variables to the struct "Relay" that will (cont next line)
Relay fun2; //(cont) include the data points contained in the struct "Relay"

int MagneticSwitch;
unsigned long previousIncrement_right = 0;
unsigned long previousIncrement_left = 0;
unsigned long previousDecrement_right = 0;
unsigned long previousDecrement_left = 0;
const int accelerationInterval = 15;
const int decelerationInterval = 15;
unsigned long priorityControlTimer = 0;
unsigned long lastDpadInput = 0;
unsigned long lastDualAnalogInput = 0;
unsigned long stopRumble;
const unsigned long priorityControlInterval = 7000; //The time in milliseconds that must pass before the control of the vehicle is automatically returned to the child



//##############################################################################

void setup(){


 

  // configure Arduino pins to the pins that were listed when we used "#define" with the
  // variables in lines 27-44 at the beginning of the program
  
  //"input_pullup" means that when estopped is not pressed the arduino (cont nxt line
  //(cont )will see a high instead of low. this is there so that the the switches and
  // sensors will not be thrown off by electromagnetic intereferance.

  //"input" denotes that the variable on the otherside of the comma is an input value
  // meaning that it will recieve the value for that variable from an outside source
  // i.e. the physical control system in this case.

  //"output" denotes that the value of the resistor is determined inside the program and then sent to the arduino
  
  pinMode(BTN_ESTOP,INPUT_PULLUP); 
  pinMode(BTN_FORWARD,INPUT_PULLUP);
  pinMode(BTN_REVERSE,INPUT_PULLUP);
  pinMode(BTN_LEFT,INPUT_PULLUP);
  pinMode(BTN_RIGHT,INPUT_PULLUP);
  pinMode(POT_GOVERNOR,INPUT);
  pinMode(POT_ACCELERATION,INPUT);
  pinMode(POT_TRIM,INPUT);
  pinMode(LED_HEARTBEAT,OUTPUT);
  pinMode(LED_ESTOP,OUTPUT);
  pinMode(RELAY_FUN1,OUTPUT);
  pinMode(RELAY_FUN2,OUTPUT);
  pinMode(MAG_SWITCH,INPUT);

// initialize global variables
  estopped = false;
  magstop = false;
  dpadInput = false;
  dualAnalogInput = false;
  timeoutChildControl = true;
// initializing start-up/default values
  left_motor.name = "L";
  right_motor.name = "R";
  left_motor.speed = 0;
  right_motor.speed = 0;
  left_motor.direction = FORWARD;
  right_motor.direction = FORWARD;
  left_motor.center_throttle = MOTOR2_STOP;
  right_motor.center_throttle = MOTOR1_STOP;
  left_motor.trim = 0;
  right_motor.trim = 0;
  
//assigning values to relay variables fun
  fun1.active = RELAY_FUN1_INVERT;
  fun2.active = RELAY_FUN2_INVERT;
  fun1.cool_at = 0;
  fun2.cool_at = 0;

  Serial.begin(BAUD_RATE);
  soft_serial.begin(BAUD_RATE);

// USB initialization / Controller Connection Setup
  Usb.Init();

}

//##############################################################################

void loop(){
  digitalWrite(LED_HEARTBEAT,HIGH);
  Usb.Task();

  MagneticSwitch = analogRead(MAG_SWITCH);
  //Serial.println(MagneticSwitch);
  if (estopped != true){
    update_acceleration();
    update_governor();
    update_trim();
    if(MagneticSwitch != 0){
      do_magstop();
    }
     if (PS4.connected()) {
      batteryLevelIndicator(); //Update controller LED to indicate battery level
      rangeRumbleIndicator();  //Rumble controller if connection time between controller and Arduino is too long
      update_wirelessInputBooleans();

      if(timeoutChildControl == true){
        read_joystick();
      }
      if (dpadInput != true){
        read_wireless();         //Dual-Analog Joystick Steering (WIRELESS)
      }
      if (dualAnalogInput != true){
        read_dpad();             //D-Pad Steering (WIRELESS)
      }
      if ((millis() - priorityControlTimer) > priorityControlInterval){
        timeoutChildControl = true;
      }
      
//    read_touchpad();         //Touchpad Steering (WIRELESS)
      wirelessEmergencyStop(); //Wireless Emergency Stop Function
      wirelessOptions();       //Misc. Wireless Control Options
     }
      
      else {
            read_joystick();
          }} else {
          pulse_estop_led();
        }
      
        update_motor_controller();
        disengage_fun();
      
        digitalWrite(LED_HEARTBEAT,LOW);
}

//##############################################################################
//this function is what the arduino is doing at rest
void pulse_estop_led(){

  #define ESTOP_LED_RATE 500 // milliseconds
  static byte estop_led_state = LOW;
  static unsigned long last_transition = millis();

  decelerate_left();
  decelerate_right();

  if (millis() - last_transition > ESTOP_LED_RATE){
    if (estop_led_state == HIGH){
      estop_led_state = LOW;
    } else {
      estop_led_state = HIGH;
    };
    digitalWrite(LED_ESTOP,estop_led_state);
    last_transition = millis();
  }

  if (PS4.connected()){
    if (PS4.getButtonPress(L2) && PS4.getButtonPress(R2)){
      estopped = false;
      priorityControlTimer = millis();
      Serial.println("Re-activate");
    }
  }

}

//##############################################################################
//this function is what the arduino is doing at rest
void pulse_magstop_led(){

  #define ESTOP_LED_RATE 500 // milliseconds
  static byte estop_led_state = LOW;
  static unsigned long last_transition = millis();

  decelerate_left();
  decelerate_right();

  if (millis() - last_transition > ESTOP_LED_RATE){
    if (estop_led_state == HIGH){
      estop_led_state = LOW;
    } else {
      estop_led_state = HIGH;
    };
    digitalWrite(LED_ESTOP,estop_led_state);
    last_transition = millis();
  }

  if (PS4.connected()){
    if (PS4.getButtonPress(L2) && PS4.getButtonPress(R2)){
      estopped = false;
      priorityControlTimer = millis();
      Serial.println("Re-activate");
    }
  }
  if (MagneticSwitch == 0){
    magstop = false;
  }

}

//##############################################################################

void read_joystick(){
//Sending the button info to the motors to drive the jeep
  if (digitalRead(BTN_ESTOP) == LOW){
    do_estop();
  } else if (digitalRead(BTN_FORWARD) == LOW && digitalRead(BTN_RIGHT) == HIGH && digitalRead(BTN_LEFT) == HIGH) {
    do_forward();
    engage_fun(BTN_FORWARD);
  } else if (digitalRead(BTN_REVERSE) == LOW && digitalRead(BTN_RIGHT) == HIGH && digitalRead(BTN_LEFT) == HIGH) {
    do_reverse();
    engage_fun(BTN_REVERSE);
  } else if (digitalRead(BTN_LEFT) == LOW && digitalRead(BTN_FORWARD) == HIGH && digitalRead(BTN_REVERSE) == HIGH) {
    do_left();
    engage_fun(BTN_LEFT);
  } else if (digitalRead(BTN_RIGHT) == LOW && digitalRead(BTN_FORWARD) == HIGH && digitalRead(BTN_REVERSE) == HIGH) {
    do_right();
    engage_fun(BTN_RIGHT);
  } else if (digitalRead(BTN_FORWARD) == LOW && digitalRead(BTN_RIGHT) == LOW && digitalRead(BTN_LEFT) == HIGH){
    do_forward_vector_right();
  } else if (digitalRead(BTN_FORWARD) == LOW && digitalRead(BTN_LEFT) == LOW && digitalRead(BTN_RIGHT) == HIGH){
    do_forward_vector_left();
  } else if (digitalRead(BTN_REVERSE) == LOW && digitalRead(BTN_RIGHT) == LOW && digitalRead(BTN_LEFT) == HIGH){
    do_reverse_vector_right();
  } else if (digitalRead(BTN_REVERSE) == LOW && digitalRead(BTN_LEFT) == LOW && digitalRead(BTN_RIGHT) == HIGH){
    do_reverse_vector_left();
  }else{
    decelerate_left();
    decelerate_right();
  }

}

//##############################################################################

void engage_fun(byte button){

  if ( (RELAY_FUN1_TRIGGER == button) && RELAY_FUN1_ENABLED ){
    if (millis() > fun1.cool_at){
      if (RELAY_FUN1_INVERT){
        digitalWrite(RELAY_FUN1,LOW);
      } else {
        digitalWrite(RELAY_FUN1,HIGH);
      }
      fun1.active = true;
      fun1.disable_at = millis() + RELAY_FUN1_ON_TIME;
      fun1.cool_at = fun1.disable_at + RELAY_FUN1_COOLDOWN;
    }
  }

  if ( (RELAY_FUN2_TRIGGER == button) && RELAY_FUN2_ENABLED ){
    if (millis() > fun2.cool_at){
      if (RELAY_FUN2_INVERT){
        digitalWrite(RELAY_FUN2,LOW);
      } else {
        digitalWrite(RELAY_FUN2,HIGH);
      }
      fun2.active = true;
      fun2.disable_at = millis() + RELAY_FUN2_ON_TIME;
      fun2.cool_at = fun2.disable_at + RELAY_FUN2_COOLDOWN;
    }
  }

}

//##############################################################################

void disengage_fun(){

  if (fun1.active && (fun1.disable_at < millis())){
    if (RELAY_FUN1_INVERT){
      digitalWrite(RELAY_FUN1,HIGH);
    } else {
      digitalWrite(RELAY_FUN1,LOW);
    }
    fun1.active = false;
  }

  if (fun2.active && (fun2.disable_at < millis())){
    if (RELAY_FUN2_INVERT){
      digitalWrite(RELAY_FUN2,HIGH);
    } else {
      digitalWrite(RELAY_FUN2,LOW);
    }
    fun2.active = false;
  }

}

//##############################################################################
//Updating govenor input
void update_governor(){
  int raw_pot = analogRead(POT_GOVERNOR);//pot governor is PIN A0
  //Serial.println(analogRead(POT_GOVERNOR));
  max_speed = map( raw_pot, 0, 1024, 1, 63 );//map(value, fromLow, fromHigh, toLow, toHigh)
  //Serial.println(max_speed);
}

//#############################################################################

// Updating the out used in "void update_govenor()" to produce a higher max speed
void update_trim(){

  int raw_pot = analogRead(POT_TRIM);
  int trim = map( raw_pot, 0, 1023, -(max_speed/4), (max_speed/4) );

//  Serial.println(raw_pot);
//  Serial.print( " max_speed: ");
//  Serial.print(max_speed);
//  Serial.print( " trim: ");
//  Serial.println(trim);
  
  if (trim > 0){
    left_motor.trim = 0;
    right_motor.trim = trim;
  } else {
    left_motor.trim = abs(trim);
    right_motor.trim = 0;
  }

}

//##############################################################################

void update_acceleration(){

  if (USE_ACCELERATION_POT){
    acceleration = map(analogRead(POT_ACCELERATION), 0, 1023, 1, max_speed/4 );
    deceleration = acceleration * 2;
    //acceleration = acceleration / 10;
//    Serial.println(analogRead(POT_ACCELERATION));
//    Serial.print( " acceleration: ");
//    Serial.println(acceleration);
//    Serial.print( " deceleration: ");
//    Serial.println(deceleration);
    //ACCELERATION POTENTIOMETER RANGE 1-14
  }
  
  else{
  acceleration = 1;
  deceleration = acceleration * 2;
  }
  

}

//##############################################################################

void accelerate_right(Direction direction, int amount){  //Amount == acceleration
unsigned long accelerationTimer = 0;
accelerationTimer = millis();

if (accelerationTimer - previousIncrement_right > accelerationInterval){
  
// if we are accelerating [motor] in the [direction] it's already turning,
  // all we have to do is add [amount] to the speed
  if (right_motor.direction == direction){
 
    right_motor.speed += amount;
    previousIncrement_right=millis();
    
  // otherwise we're accelerating in the opposite direction from the motor's
  // current motion, in which case we'll reduce speed by [amount]
  // and, if we've crossed 0 speed into negative territory, reverse the motor
  } else {

    right_motor.speed -= amount;
    previousIncrement_right=millis();
    
    if (right_motor.speed < 0){
      right_motor.speed = abs(right_motor.speed);
      if (right_motor.direction == REVERSE){
        right_motor.direction = FORWARD;
      } else {
        right_motor.direction = REVERSE;
      }
    }
  }
  govern_motor(&right_motor);
}
}
//##############################################################################

void accelerate_left(Direction direction, int amount){  //Amount == acceleration
unsigned long accelerationTimer = 0;
accelerationTimer = millis();

   if (accelerationTimer - previousIncrement_left > accelerationInterval){
  
// if we are accelerating [motor] in the [direction] it's already turning,
  // all we have to do is add [amount] to the speed
  if (left_motor.direction == direction){

    left_motor.speed += amount;
    previousIncrement_left=millis();
  
  // otherwise we're accelerating in the opposite direction from the motor's
  // current motion, in which case we'll reduce speed by [amount]
  // and, if we've crossed 0 speed into negative territory, reverse the motor
  } else {

    left_motor.speed -= amount;
    previousIncrement_left=millis();

    if (left_motor.speed < 0){
      left_motor.speed = abs(left_motor.speed);
      if (left_motor.direction == REVERSE){
        left_motor.direction = FORWARD;
      } else {
        left_motor.direction = REVERSE;
      }
    }
  }
  govern_motor(&left_motor);
}
}


//##############################################################################

void vector_accelerate_right(Direction direction, int amount){  //Amount == acceleration
unsigned long accelerationTimer = 0;
accelerationTimer = millis();

   if (accelerationTimer - previousIncrement_right > accelerationInterval){
  
// if we are accelerating [motor] in the [direction] it's already turning,
  // all we have to do is add [amount] to the speed
  if (right_motor.direction == direction){
 
    right_motor.speed += amount;
    previousIncrement_right = millis();
  // otherwise we're accelerating in the opposite direction from the motor's
  // current motion, in which case we'll reduce speed by [amount]
  // and, if we've crossed 0 speed into negative territory, reverse the motor
  } else {
    right_motor.speed -= amount;
    previousIncrement_right = millis();
    if (right_motor.speed < 0){
      right_motor.speed = abs(right_motor.speed);
      if (right_motor.direction == REVERSE){
        right_motor.direction = FORWARD;
      } else {
        right_motor.direction = REVERSE;
      }
    }
  }
  vector_govern_motor(&right_motor);
}
}

//##############################################################################

void vector_accelerate_left(Direction direction, int amount){  //Amount == acceleration
unsigned long accelerationTimer = 0;
accelerationTimer = millis();
if (accelerationTimer - previousIncrement_left > accelerationInterval){
 
// if we are accelerating [motor] in the [direction] it's already turning,
  // all we have to do is add [amount] to the speed
  if (left_motor.direction == direction){
    left_motor.speed += amount;
    previousIncrement_left = millis();
    
  // otherwise we're accelerating in the opposite direction from the motor's
  // current motion, in which case we'll reduce speed by [amount]
  // and, if we've crossed 0 speed into negative territory, reverse the motor
  } else {
    left_motor.speed -= amount;
    previousIncrement_left = millis();
 
    if (left_motor.speed < 0){
      left_motor.speed = abs(left_motor.speed);
      if (left_motor.direction == REVERSE){
        left_motor.direction = FORWARD;
      } else {
        left_motor.direction = REVERSE;
      }
    }
  }
  vector_govern_motor(&left_motor);
}
}
//##############################################################################

//void decelerate(Motor *motor){
//
//  // regardless of which direction the motor is turning, deceleration is simply
//  // a matter of reducing speed by deceleration
//  motor->speed -= deceleration;
//  govern_motor(motor);
//}

//##############################################################################

void decelerate_left(){
unsigned long decelerationTimer = 0;
decelerationTimer = millis();
if (decelerationTimer - previousDecrement_left > decelerationInterval){

  // regardless of which direction the motor is turning, deceleration is simply
  // a matter of reducing speed by deceleration
  left_motor.speed -= deceleration;
  previousDecrement_left = millis();
  govern_motor(&left_motor);
}
}


//##############################################################################

void decelerate_right(){
unsigned long decelerationTimer = 0;
decelerationTimer = millis();
if (decelerationTimer - previousDecrement_right > decelerationInterval){

  // regardless of which direction the motor is turning, deceleration is simply
  // a matter of reducing speed by deceleration
  right_motor.speed -= deceleration;
  previousDecrement_right = millis();
  govern_motor(&right_motor);
}
}

//##############################################################################

void govern_motor(Motor *motor){

  if (motor->speed < 0){
    motor->speed = 0;
  } else if (motor->speed > max_speed){
    motor->speed = max_speed;
  }
  
}

//##############################################################################
//This was added such that the motor speed would be halved for the appropriate motor in vector steering
void vector_govern_motor(Motor *motor){

  if (motor->speed < 0){
    motor->speed = 0;
  } else if (motor->speed > max_speed/2){
    motor->speed = max_speed/2;
  }
  
}

//##############################################################################

void do_estop(){  //Emergency Stop

  estopped = true;
  decelerate_left();
  decelerate_right();
  govern_motor(&left_motor);
  govern_motor(&right_motor);
}

//##############################################################################

void do_magstop(){  //Emergency Stop

  magstop = true;
  decelerate_left();
  decelerate_right();
  govern_motor(&left_motor);
  govern_motor(&right_motor);
}


//##############################################################################

bool left_motor_slow(Direction direction){

  if (left_motor.direction == right_motor.direction) {
    return (left_motor.speed < right_motor.speed);
  } else {
    return (left_motor.direction != direction);
  }

}

//##############################################################################

int speed_delta(){

  if (left_motor.direction == right_motor.direction){
    return abs(left_motor.speed - right_motor.speed);
  } else {
    return left_motor.speed + right_motor.speed;
  }

}

//##############################################################################

void tandem_accelerate(Direction direction){

  int delta = speed_delta();
  if (delta == 0){
    accelerate_left(direction,acceleration);
    accelerate_right(direction,acceleration);
  } else {
    if (delta > acceleration){
      delta = acceleration;
    }
    if (left_motor_slow(direction)){
      accelerate_left(direction,delta);
    } else {
      accelerate_right(direction,delta);
    }
  }

}

//##############################################################################

void do_forward(){

    accelerate_left(FORWARD,acceleration);
    accelerate_right(REVERSE,acceleration);

  Serial.println("FORWARD");

}

//##############################################################################

void do_reverse(){

    accelerate_left(REVERSE,acceleration);
    accelerate_right(FORWARD,acceleration);

  Serial.println("REVERSE");

}

//##############################################################################

void do_left(){
  tandem_accelerate(FORWARD);
  Serial.println("LEFT");

}

//##############################################################################

void do_right(){
  tandem_accelerate(REVERSE);
  Serial.println("RIGHT");
  
}

//##############################################################################

void do_forward_vector_right(){
   vector_accelerate_left(FORWARD,acceleration);
   accelerate_right(REVERSE,acceleration);
   Serial.println("+VECTOR RIGHT");

}

//##############################################################################

void do_forward_vector_left(){

  accelerate_left(FORWARD,acceleration);
  vector_accelerate_right(REVERSE,acceleration);
  Serial.println("+VECTOR LEFT");

}

//##############################################################################

void do_reverse_vector_right(){

  vector_accelerate_left(REVERSE,acceleration);
  accelerate_right(FORWARD,acceleration);
  Serial.println("-VECTOR RIGHT");

}

//##############################################################################

void do_reverse_vector_left(){

  accelerate_left(REVERSE,acceleration);
  vector_accelerate_right(FORWARD,acceleration);
  Serial.println("-VECTOR LEFT");

}

//##############################################################################

byte calculate_throttle(Motor *motor){

  byte throttle;
  if (motor->speed < motor->trim){
    throttle = motor->center_throttle;
  } else {
    if (motor->direction == FORWARD){
      throttle = motor->center_throttle + motor->speed - motor->trim;
    } else {
      throttle = motor->center_throttle - motor->speed + motor->trim;
    }
  }
//  Serial.print(motor->name);
//  Serial.print("  throttle: ");
//  Serial.print(throttle);
//  Serial.print("  speed: ");
//  Serial.print(motor->speed);
//  Serial.print("  trim: ");
//  Serial.println(motor->trim);
  return throttle;

}

//##############################################################################

void update_motor_controller(){

  byte right_motor_control, left_motor_control;

  right_motor_control = calculate_throttle(&right_motor);
  left_motor_control = calculate_throttle(&left_motor);

//  Serial.write(right_motor_control);
//  Serial.write(left_motor_control);

  soft_serial.write(right_motor_control);
  soft_serial.write(left_motor_control);

}

//##############################################################################

void update_wirelessInputBooleans(){

if (millis() - lastDualAnalogInput > 1000){
  dualAnalogInput = false;
}

if (millis() - lastDpadInput > 1000){
  dpadInput = false;
}
}

//##############################################################################


//This function will use the preset colors of green, yellow, and red to indicate battery charge level of the controller from 0-15
void batteryLevelIndicator(){
    int ds4Charge = PS4.getBatteryLevel();
    //Serial.println(ds4Charge);
    if (ds4Charge >= 11){
      PS4.setLed(Green);
    }
    else if(ds4Charge < 11 && ds4Charge >= 6){
      PS4.setLed(Yellow);
    }
    else if(ds4Charge < 6){
      PS4.setLed(Red);
    }
  }

//##############################################################################
//This function will initiate various intensities of rumble depending on the magnitude of communication time between the Arduino and the PS4 controller
void rangeRumbleIndicator(){
  //Serial.println(millis() - PS4.getLastMessageTime());
     if (((millis() - PS4.getLastMessageTime()) > 50)){
        PS4.setRumbleOn(RumbleLow);
        stopRumble = millis();
//      }else if ((millis() - PS4.getLastMessageTime()) > 1000){
//      PS4.setRumbleOn(RumbleHigh);
//      stopRumble = millis(); 
      }else if ((millis() - PS4.getLastMessageTime()) > 1000) {
      Serial.println("Lost connection to PS4 controller");
      read_joystick();
     }else if (millis() - stopRumble > 3000){
      PS4.setRumbleOff();
      }
}

//##############################################################################
//This function defines the means by which the PS4 controller can trigger the emergency stop function.
void wirelessEmergencyStop(){
      if (PS4.getButtonPress(L1) && PS4.getButtonPress(R1)){
        do_estop();
        priorityControlTimer = millis();
        Serial.println("WIRELESS ESTOP");
      }
}

//##############################################################################
//This function houses miscellaneous control options for the PS4 controller
void wirelessOptions(){
      if (PS4.getButtonClick(OPTIONS)){
        Serial.println("Controller Manually Disconnected");
        PS4.disconnect();
      }
}
//##############################################################################

void read_wireless(){
/*Reading information from the wireless controller's dual analog joysticks
 * The analog joysticks return the following:
 * x-axis: 0-255, left to right
 * y-axis: 0-255, top to bottom
 */
  if (digitalRead(BTN_ESTOP) == LOW){
    do_estop();
  } else if (PS4.getAnalogHat(LeftHatY) < 98 && PS4.getAnalogHat(RightHatX) > 98 && PS4.getAnalogHat(RightHatX) < 158) {
    do_forward();
    engage_fun(BTN_FORWARD);
    priorityControlTimer = millis();
    lastDualAnalogInput = millis();
    dualAnalogInput = true;
    timeoutChildControl = false;
    //Serial.println("WIRELESS FORWARD");
  } else if (PS4.getAnalogHat(LeftHatY) > 158 && PS4.getAnalogHat(RightHatX) > 98 && PS4.getAnalogHat(RightHatX) < 158) {
    do_reverse();
    engage_fun(BTN_REVERSE);
    priorityControlTimer = millis();
    lastDualAnalogInput = millis();
    dualAnalogInput = true;
    timeoutChildControl = false;
    //Serial.println("WIRELESS REVERSE");
  } else if (PS4.getAnalogHat(RightHatX) < 98 && PS4.getAnalogHat(LeftHatY) > 98 && PS4.getAnalogHat(LeftHatY) < 158) {
    do_left();
    engage_fun(BTN_LEFT);
    priorityControlTimer = millis();
    lastDualAnalogInput = millis();
    dualAnalogInput = true;
    timeoutChildControl = false;
    //Serial.println("WIRELESS LEFT");
  } else if (PS4.getAnalogHat(RightHatX) > 158 && PS4.getAnalogHat(LeftHatY) > 98 && PS4.getAnalogHat(LeftHatY) < 158) {
    do_right();
    engage_fun(BTN_RIGHT);
    priorityControlTimer = millis();
    lastDualAnalogInput = millis();
    dualAnalogInput = true;
    timeoutChildControl = false;
    //Serial.println("WIRELESS RIGHT");
  } else if (PS4.getAnalogHat(LeftHatY) < 98 && PS4.getAnalogHat(RightHatX) > 158){
    do_forward_vector_right();
    priorityControlTimer = millis();
    lastDualAnalogInput = millis();
    dualAnalogInput = true;
    timeoutChildControl = false;
  } else if (PS4.getAnalogHat(LeftHatY) < 98 && PS4.getAnalogHat(RightHatX) < 98){
    do_forward_vector_left();
    priorityControlTimer = millis();
    lastDualAnalogInput = millis();
    dualAnalogInput = true;
    timeoutChildControl = false;
  } else if (PS4.getAnalogHat(LeftHatY) > 158 && PS4.getAnalogHat(RightHatX) > 158){
    do_reverse_vector_right();
    priorityControlTimer = millis();
    lastDualAnalogInput = millis();
    dualAnalogInput = true;
    timeoutChildControl = false;
  } else if (PS4.getAnalogHat(LeftHatY) > 158 && PS4.getAnalogHat(RightHatX) < 98){
    do_reverse_vector_left();
    priorityControlTimer = millis();
    lastDualAnalogInput = millis();
    dualAnalogInput = true;
    timeoutChildControl = false;
  }else if (timeoutChildControl != true){
    decelerate_left();
    decelerate_right();
  }

}

//################################################################################

//void read_touchpad(){
//  /*
//   * Reading information from the wireless controller's touchpad to determine motor control, single finger only
//   * Touchpad reads the following:
//   * x-axis: 0-1919, left to right
//   * y-axis: 0-941, top to bottom
//   */
//  uint8_t touchPosition;
//  
//  /*
//   * //Uncomment this to test x/y coordinates returned by the touchpad
//  Serial.print(PS4.getX(touchPosition));
//  Serial.print(", ");
//  Serial.println(PS4.getY(touchPosition));
//   */
//  if (digitalRead(BTN_ESTOP) == LOW){
//    do_estop();
//  }  else if (PS4.isTouching(0) || PS4.isTouching(1)){
//      if (PS4.getY(touchPosition) < 470 && PS4.getX(touchPosition) > 640 && PS4.getX(touchPosition) < 1280) {
//      do_forward();
//      engage_fun(BTN_FORWARD);
//      priorityControlTimer = millis();
//      //Serial.println("TOUCHPAD FORWARD");
//    } else if (PS4.getY(touchPosition) > 470 && PS4.getX(touchPosition) > 640 && PS4.getX(touchPosition) < 1280) {
//      do_reverse();
//      engage_fun(BTN_REVERSE);
//      priorityControlTimer = millis();
//      //Serial.println("TOUCHPAD REVERSE");
//    } else if (PS4.getX(touchPosition) < 640 && PS4.getY(touchPosition) > 220 && PS4.getY(touchPosition) < 720) {
//      do_left();
//      engage_fun(BTN_LEFT);
//      priorityControlTimer = millis();
//      //Serial.println("TOUCHPAD LEFT");
//    } else if (PS4.getX(touchPosition) > 1280 && PS4.getY(touchPosition) > 220 && PS4.getY(touchPosition) < 720) {
//      do_right();
//      engage_fun(BTN_RIGHT);
//      priorityControlTimer = millis();
//      //Serial.println("TOUCHPAD RIGHT");
//    } else if (PS4.getY(touchPosition) < 370 && PS4.getX(touchPosition) > 1280){
//      do_forward_vector_right();
//      priorityControlTimer = millis();
//    } else if (PS4.getY(touchPosition) < 370 && PS4.getX(touchPosition) < 640){
//      do_forward_vector_left();
//      priorityControlTimer = millis();
//    } else if (PS4.getY(touchPosition) > 570 && PS4.getX(touchPosition) > 1280){
//      do_reverse_vector_right();
//      priorityControlTimer = millis();
//    } else if (PS4.getY(touchPosition) > 570 && PS4.getX(touchPosition) < 640){
//      do_reverse_vector_left();
//      priorityControlTimer = millis();
//    }
//    } else{
//      decelerate(&left_motor);
//      decelerate(&right_motor);
//    }
//    }

//################################################################################
void read_dpad(){
  //Reading information from the d-pad to control vehicle
  //getButtonPress will return true as long as the button is held down
  if (digitalRead(BTN_ESTOP) == LOW){
    do_estop();
  } else if (PS4.getButtonPress(UP) && !PS4.getButtonPress(RIGHT) && !PS4.getButtonPress(LEFT)) {
    do_forward();
    engage_fun(BTN_FORWARD);
    priorityControlTimer = millis();
    lastDpadInput = millis();
    dpadInput = true;
    timeoutChildControl = false;
    //Serial.println("D-PAD FORWARD");
  } else if (PS4.getButtonPress(DOWN) && !PS4.getButtonPress(RIGHT) && !PS4.getButtonPress(LEFT)) {
    do_reverse();
    engage_fun(BTN_REVERSE);
    priorityControlTimer = millis();
    lastDpadInput = millis();
    dpadInput = true;
    timeoutChildControl = false;
    //Serial.println("D-PAD REVERSE");
  } else if (PS4.getButtonPress(LEFT) && !PS4.getButtonPress(UP) && !PS4.getButtonPress(DOWN)) {
    do_left();
    engage_fun(BTN_LEFT);
    priorityControlTimer = millis();
    lastDpadInput = millis();
    dpadInput = true;
    timeoutChildControl = false;
    //Serial.println("D-PAD LEFT");
  } else if (PS4.getButtonPress(RIGHT) && !PS4.getButtonPress(UP) && !PS4.getButtonPress(DOWN)) {
    do_right();
    engage_fun(BTN_RIGHT);
    priorityControlTimer = millis();
    lastDpadInput = millis();
    dpadInput = true;
    timeoutChildControl = false;
    //Serial.println("D-PAD RIGHT");
  } else if (PS4.getButtonPress(UP) && PS4.getButtonPress(RIGHT)){
    do_forward_vector_right();
    priorityControlTimer = millis();
    lastDpadInput = millis();
    timeoutChildControl = false;
    dpadInput = true;
  } else if (PS4.getButtonPress(UP) && PS4.getButtonPress(LEFT)){
    do_forward_vector_left();
    priorityControlTimer = millis();
    lastDpadInput = millis();
    dpadInput = true;
    timeoutChildControl = false;
  } else if (PS4.getButtonPress(DOWN) && PS4.getButtonPress(RIGHT)){
    do_reverse_vector_right();
    priorityControlTimer = millis();
    lastDpadInput = millis();
    timeoutChildControl = false;
    dpadInput = true;
  } else if (PS4.getButtonPress(DOWN) && PS4.getButtonPress(LEFT)){
    do_reverse_vector_left();
    priorityControlTimer = millis();
    lastDpadInput = millis();
    timeoutChildControl = false;
    dpadInput = true;
  }
  else if (timeoutChildControl != true){
    decelerate_left();
    decelerate_right();
  }
}
