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

#define RELAY_FUN1            0
#define RELAY_FUN2            1

//##############################################################################
// The values in this section only can be modified to alter the behavior of the
// individual ride-on toy.

#define USE_ACCELERATION_POT  true  //If there is no installed acceleration potentiometer, set to 'false'



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
//#define USB_XFER_TIMEOUT 50

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
byte max_speed; //"byte" means max_speed will be any number 0->255
byte acceleration = BASE_ACCELERATION;
byte deceleration = BASE_DECELERATION;
Motor left_motor;  // here we are assigning new variables to the struct "motors" that will (cont next line) 
Motor right_motor; //(cont) include the data points contained in the struct "motor"
Relay fun1; // here we are assigning new variables to the struct "Relay" that will (cont next line)
Relay fun2; //(cont) include the data points contained in the struct "Relay"


int controlSwitch = 0; //This value controlSwitch will control the Switch/Case function for altering control methods
unsigned long accelerationTimer;
unsigned long previousIncrement = 0;
const unsigned long accelerationInterval = 250; //The time in milliseconds that must pass before void accelerate is called

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

// initialize global variables
  estopped = false;
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
  if (estopped != true){
    update_acceleration();
    update_governor();
    update_trim();


    if (PS4.connected()) {
     batteryLevelIndicator();
     Serial.rint("Connection Time in milliseconds: ");
     Serial.println(PS4.getLastMessageTime());
      if (millis() - PS4.getLastMessageTime() > 1000) {
        Serial.println("PS4 Connection Buffering");
        read_joystick();
        if (millis() - PS4.getLastMessageTime() > 30000){
          Serial.println("PS4 Connection Lost - Controller Disconnected");
          //PS4.disconnect();
        }
      } else {

/*The following if statements control the switch function such that the following button presses enable the respective functions:
 *    Wireless Control defaults to case 0: Dual Analog Joystick Steering
 *    Circle: Touchpad Steering
 *    Cross: D-Pad Steering
 *    Square: On-Board Child Steering
 *    Triangle: Dual Analog Joystick Steering
 *    
 *    L1+R1: Emergency Stop
 */
    if (PS4.getButtonClick(TRIANGLE)){
        controlSwitch = 0;
      }
      else if (PS4.getButtonClick(CIRCLE)){
        controlSwitch = 1;
      }
      else if (PS4.getButtonClick(CROSS)){
        controlSwitch = 2;
      }
      else if (PS4.getButtonClick(SQUARE)){
        controlSwitch = 3;
      }
      else if (PS4.getButtonPress(L1) && PS4.getButtonPress(R1)){
        do_estop();
        Serial.println("WIRELESS ESTOP");
      }
      else if (PS4.getButtonClick(OPTIONS)){
        Serial.println("Controller Manually Disconnected");
        PS4.disconnect();
      }
      switch(controlSwitch) {
        default:controlSwitch = 0;
        case 0: read_wireless();
                break;
        case 1: read_touchpad();
                break;
        case 2: read_dpad();
                break;
        case 3: read_joystick();
                break;
      }
  }
} else {
      read_joystick();
      //Serial.println("READ_JOYSTICK");
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

  decelerate(&left_motor);
  decelerate(&right_motor);

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
      Serial.println("Re-activate");
    }
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
    decelerate(&left_motor);
    decelerate(&right_motor);
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
    acceleration = map(analogRead(POT_ACCELERATION), 0, 1023, 1, (max_speed)/4 );
    deceleration = acceleration * 2;
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

void accelerate(Motor *motor, Direction direction, int amount){  //Amount == acceleration
accelerationTimer = millis();
  // if we are accelerating [motor] in the [direction] it's already turning,
  // all we have to do is add [amount] to the speed
  if (motor->direction == direction){
    //if (accelerationTimer - previousIncrement > accelerationInterval){
    motor->speed += amount;
    //previousIncrement=millis();
    //}
  // otherwise we're accelerating in the opposite direction from the motor's
  // current motion, in which case we'll reduce speed by [amount]
  // and, if we've crossed 0 speed into negative territory, reverse the motor
  } else {
    //if (accelerationTimer - previousIncrement > accelerationInterval){
    motor->speed -= amount;
    //previousIncrement=millis();
    //}
    if (motor->speed < 0){
      motor->speed = abs(motor->speed);
      if (motor->direction == REVERSE){
        motor->direction = FORWARD;
      } else {
        motor->direction = REVERSE;
      }
    }
  }
  govern_motor(motor);
}



//##############################################################################

void vector_accelerate(Motor *motor, Direction direction, int amount){  //Amount == acceleration
accelerationTimer = millis();
  // if we are accelerating [motor] in the [direction] it's already turning,
  // all we have to do is add [amount] to the speed
  if (motor->direction == direction){
    //if (accelerationTimer - previousIncrement > accelerationInterval){
    motor->speed += amount;
    //previousIncrement = millis();
    //}
  // otherwise we're accelerating in the opposite direction from the motor's
  // current motion, in which case we'll reduce speed by [amount]
  // and, if we've crossed 0 speed into negative territory, reverse the motor
  } else {
    //if (accelerationTimer - previousIncrement > accelerationInterval){
    motor->speed -= amount;
    //previousIncrement = millis();
    //}
    if (motor->speed < 0){
      motor->speed = abs(motor->speed);
      if (motor->direction == REVERSE){
        motor->direction = FORWARD;
      } else {
        motor->direction = REVERSE;
      }
    }
  }
  vector_govern_motor(motor);
}
//##############################################################################

void decelerate(Motor *motor){

  // regardless of which direction the motor is turning, deceleration is simply
  // a matter of reducing speed by deceleration
  motor->speed -= deceleration;
  govern_motor(motor);
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
  decelerate(&left_motor);
  decelerate(&right_motor);
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
    accelerate(&left_motor,direction,acceleration);
    accelerate(&right_motor,direction,acceleration);
  } else {
    if (delta > acceleration){
      delta = acceleration;
    }
    if (left_motor_slow(direction)){
      accelerate(&left_motor,direction,delta);
    } else {
      accelerate(&right_motor,direction,delta);
    }
  }

}

//##############################################################################

void do_forward(){

    accelerate(&left_motor,FORWARD,acceleration);
    accelerate(&right_motor,REVERSE,acceleration);

  //Serial.println("FORWARD");

}

//##############################################################################

void do_reverse(){

    accelerate(&left_motor,REVERSE,acceleration);
    accelerate(&right_motor,FORWARD,acceleration);

  //Serial.println("REVERSE");

}

//##############################################################################

void do_left(){
  tandem_accelerate(FORWARD);
  //Serial.println("LEFT");

}

//##############################################################################

void do_right(){
  tandem_accelerate(REVERSE);
  //Serial.println("RIGHT");
  
}

//##############################################################################

void do_forward_vector_right(){
   vector_accelerate(&left_motor,FORWARD,acceleration);
   accelerate(&right_motor,REVERSE,acceleration);
   //Serial.println("+VECTOR RIGHT");

}

//##############################################################################

void do_forward_vector_left(){

  accelerate(&left_motor,FORWARD,acceleration);
  vector_accelerate(&right_motor,REVERSE,acceleration);
  //Serial.println("+VECTOR LEFT");

}

//##############################################################################

void do_reverse_vector_right(){

  vector_accelerate(&left_motor,REVERSE,acceleration);
  accelerate(&right_motor,FORWARD,acceleration);
  //Serial.println("-VECTOR RIGHT");

}

//##############################################################################

void do_reverse_vector_left(){

  accelerate(&left_motor,REVERSE,acceleration);
  vector_accelerate(&right_motor,FORWARD,acceleration);
  //Serial.println("-VECTOR LEFT");

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
//This function will use the preset colors of green, yellow, and red to indicate battery charge level of the controller from 0-15
  void batteryLevelIndicator(){
    int ds4Charge = PS4.getBatteryLevel();
    //Serial.println(ds4Charge);
    if (ds4Charge >= 8){
      PS4.setLed(Green);
    }
    else if(ds4Charge < 8 && ds4Charge >= 4){
      PS4.setLed(Yellow);
    }
    else if(ds4Charge < 4){
      PS4.setLed(Red);
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
    //Serial.println("WIRELESS FORWARD");
  } else if (PS4.getAnalogHat(LeftHatY) > 158 && PS4.getAnalogHat(RightHatX) > 98 && PS4.getAnalogHat(RightHatX) < 158) {
    do_reverse();
    engage_fun(BTN_REVERSE);
    //Serial.println("WIRELESS REVERSE");
  } else if (PS4.getAnalogHat(RightHatX) < 98 && PS4.getAnalogHat(LeftHatY) > 98 && PS4.getAnalogHat(LeftHatY) < 158) {
    do_left();
    engage_fun(BTN_LEFT);
    //Serial.println("WIRELESS LEFT");
  } else if (PS4.getAnalogHat(RightHatX) > 158 && PS4.getAnalogHat(LeftHatY) > 98 && PS4.getAnalogHat(LeftHatY) < 158) {
    do_right();
    engage_fun(BTN_RIGHT);
    //Serial.println("WIRELESS RIGHT");
  } else if (PS4.getAnalogHat(LeftHatY) < 98 && PS4.getAnalogHat(RightHatX) > 158){
    do_forward_vector_right();
  } else if (PS4.getAnalogHat(LeftHatY) < 98 && PS4.getAnalogHat(RightHatX) < 98){
    do_forward_vector_left();
  } else if (PS4.getAnalogHat(LeftHatY) > 158 && PS4.getAnalogHat(RightHatX) > 158){
    do_reverse_vector_right();
  } else if (PS4.getAnalogHat(LeftHatY) > 158 && PS4.getAnalogHat(RightHatX) < 98){
    do_reverse_vector_left();
  }else{
    decelerate(&left_motor);
    decelerate(&right_motor);
  }

}

//################################################################################

void read_touchpad(){
  /*
   * Reading information from the wireless controller's touchpad to determine motor control, single finger only
   * Touchpad reads the following:
   * x-axis: 0-1919, left to right
   * y-axis: 0-941, top to bottom
   */
  uint8_t touchPosition;
  
  /*
   * //Uncomment this to test x/y coordinates returned by the touchpad
  Serial.print(PS4.getX(touchPosition));
  Serial.print(", ");
  Serial.println(PS4.getY(touchPosition));
   */
  if (digitalRead(BTN_ESTOP) == LOW){
    do_estop();
  }  else if (PS4.isTouching(0) || PS4.isTouching(1)){
      if (PS4.getY(touchPosition) < 470 && PS4.getX(touchPosition) > 640 && PS4.getX(touchPosition) < 1280) {
      do_forward();
      engage_fun(BTN_FORWARD);
      //Serial.println("TOUCHPAD FORWARD");
    } else if (PS4.getY(touchPosition) > 470 && PS4.getX(touchPosition) > 640 && PS4.getX(touchPosition) < 1280) {
      do_reverse();
      engage_fun(BTN_REVERSE);
      //Serial.println("TOUCHPAD REVERSE");
    } else if (PS4.getX(touchPosition) < 640 && PS4.getY(touchPosition) > 220 && PS4.getY(touchPosition) < 720) {
      do_left();
      engage_fun(BTN_LEFT);
      //Serial.println("TOUCHPAD LEFT");
    } else if (PS4.getX(touchPosition) > 1280 && PS4.getY(touchPosition) > 220 && PS4.getY(touchPosition) < 720) {
      do_right();
      engage_fun(BTN_RIGHT);
      //Serial.println("TOUCHPAD RIGHT");
    } else if (PS4.getY(touchPosition) < 370 && PS4.getX(touchPosition) > 1280){
      do_forward_vector_right();
    } else if (PS4.getY(touchPosition) < 370 && PS4.getX(touchPosition) < 640){
      do_forward_vector_left();
    } else if (PS4.getY(touchPosition) > 570 && PS4.getX(touchPosition) > 1280){
      do_reverse_vector_right();
    } else if (PS4.getY(touchPosition) > 570 && PS4.getX(touchPosition) < 640){
      do_reverse_vector_left();
    }
    } else{
      decelerate(&left_motor);
      decelerate(&right_motor);
    }
    }

//################################################################################
void read_dpad(){
  //Reading information from the d-pad to control vehicle
  //getButtonPress will return true as long as the button is held down
  if (digitalRead(BTN_ESTOP) == LOW){
    do_estop();
  } else if (PS4.getButtonPress(UP) && !PS4.getButtonPress(RIGHT) && !PS4.getButtonPress(LEFT)) {
    do_forward();
    engage_fun(BTN_FORWARD);
    //Serial.println("D-PAD FORWARD");
  } else if (PS4.getButtonPress(DOWN) && !PS4.getButtonPress(RIGHT) && !PS4.getButtonPress(LEFT)) {
    do_reverse();
    engage_fun(BTN_REVERSE);
    //Serial.println("D-PAD REVERSE");
  } else if (PS4.getButtonPress(LEFT) && !PS4.getButtonPress(UP) && !PS4.getButtonPress(DOWN)) {
    do_left();
    engage_fun(BTN_LEFT);
    //Serial.println("D-PAD LEFT");
  } else if (PS4.getButtonPress(RIGHT) && !PS4.getButtonPress(UP) && !PS4.getButtonPress(DOWN)) {
    do_right();
    engage_fun(BTN_RIGHT);
    //Serial.println("D-PAD RIGHT");
  } else if (PS4.getButtonPress(UP) && PS4.getButtonPress(RIGHT)){
    do_forward_vector_right();
  } else if (PS4.getButtonPress(UP) && PS4.getButtonPress(LEFT)){
    do_forward_vector_left();
  } else if (PS4.getButtonPress(DOWN) && PS4.getButtonPress(RIGHT)){
    do_reverse_vector_right();
  } else if (PS4.getButtonPress(DOWN) && PS4.getButtonPress(LEFT)){
    do_reverse_vector_left();
  }
  else{
    decelerate(&left_motor);
    decelerate(&right_motor);
  }
}
