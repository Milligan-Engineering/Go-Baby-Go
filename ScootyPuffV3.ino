//############################################################################//
//                             SCOOTYPUFF v.2                                 //
//                       "Who's ready for safe fun?"                          //
//############################################################################//


// Arduino pin assignments

#define SERIAL_OUT           11 //assigned to pin 11
#define SERIAL_IN            10

#define BTN_ESTOP             2
#define BTN_FORWARD           6
#define BTN_REVERSE           5
#define BTN_LEFT              3
#define BTN_RIGHT             4

#define POT_GOVERNOR         A0
#define POT_ACCELERATION     A1 //Milligan Engineering: Ensure this is installed or set to 1

#define POT_TRIM             A2

#define LED_HEARTBEAT        13
#define LED_ESTOP             7

#define RELAY_FUN1            8
#define RELAY_FUN2            9

//##############################################################################
// The values in this section only can be modified to alter the behavior of the
// individual ride-on toy.

#define USE_ACCELERATION_POT  true //If there is no installed acceleration potentiometer, set to 'false'



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

#include <SoftwareSerial.h>

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
bool estopped; //estopped is either true or false
byte max_speed; //"byte" means max_speed will be any number 0->255
byte acceleration = BASE_ACCELERATION;
byte deceleration = BASE_DECELERATION;
Motor left_motor;  // here we are assigning new variables to the struct "motors" that will (cont next line) 
Motor right_motor; //(cont) include the data points contained in the struct "motor"
Relay fun1; // here we are assigning new variables to the struct "Relay" that will (cont next line)
Relay fun2; //(cont) include the data points contained in the struct "Relay"

//##############################################################################

void setup(){

  // configure Arduino pins to the pins that were listed when we used "#define" with the
  // variables in lines 27-44 at the beginning of the program
  
  //"input_pullup" means that when estopped is not pressed the arduino (cont nxt line
  //(cont )will see a high instead of low. this is there so that the the switches and
  // sensors will not be thrown off by electromagnetic intereferance.

  //"input" denotes that the variable on the otherside of the comma is an input value
  // meaning that it will recieve the va;lue for that variable from an outside source
  // i.e. the physical control system in this case.

  //"output" denoes that the value of the resistor is determined inside the program and then sent to the arduino
  
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

}

//##############################################################################

void loop(){

  digitalWrite(LED_HEARTBEAT,HIGH);

  if (estopped != true){
    update_governor();
    update_trim();
    update_acceleration();
    read_buttons();
  } else {
    pulse_estop_led();
  }

  update_motor_controller();
  disengage_fun();

  digitalWrite(LED_HEARTBEAT,LOW);
  delay(LOOP_DELAY);

}

//##############################################################################
//this function is what the arduino is doing at rest
void pulse_estop_led(){

  #define ESTOP_LED_RATE 500 // milliseconds
  static byte estop_led_state = LOW;
  static unsigned long last_transition = millis();

  if (millis() - last_transition > ESTOP_LED_RATE){
    if (estop_led_state == HIGH){
      estop_led_state = LOW;
    } else {
      estop_led_state = HIGH;
    };
    digitalWrite(LED_ESTOP,estop_led_state);
    last_transition = millis();
  }

}

//##############################################################################

void read_buttons(){
//Sending the button info to the motors to drive the jeep
  if (digitalRead(BTN_ESTOP) == LOW){
    do_estop();
  } else if (digitalRead(BTN_FORWARD) == LOW) {
    do_forward();
    engage_fun(BTN_FORWARD);
  } else if (digitalRead(BTN_REVERSE) == LOW) {
    do_reverse();
    engage_fun(BTN_REVERSE);
  } else if (digitalRead(BTN_LEFT) == LOW) {
    do_left();
    engage_fun(BTN_LEFT);
  } else if (digitalRead(BTN_RIGHT) == LOW) {
    do_right();
    engage_fun(BTN_RIGHT);
  } else{
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
  max_speed = map( raw_pot, 0, 1024, 1, 63 );//map(value, fromLow, fromHigh, toLow, toHigh)
}

//#############################################################################

// Updating the out used in "void update_govenor()" to produce a higher max speed
void update_trim(){

  int raw_pot = analogRead(POT_TRIM);
  int trim = map( raw_pot, 0, 1023, -(max_speed/4), (max_speed/4) );

  //Serial.println(raw_pot);
  Serial.print( " max_speed: ");
  Serial.print(max_speed);
  Serial.print( " trim: ");
  Serial.println(trim);
  
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
    acceleration = map(analogRead(POT_ACCELERATION), 0, 1023, 1, (max_speed/4) );
    deceleration = acceleration * 2;
    Serial.print( " acceleration: ");
    Serial.println(acceleration);
    Serial.print( " deceleration: ");
    Serial.println(deceleration);
  }
  
  else{
  acceleration = 1;
  deceleration = acceleration * 2;
  }

}

//##############################################################################

void accelerate(Motor *motor, Direction direction, int amount){  //Amount == acceleration

  // if we are accelerating [motor] in the [direction] it's already turning,
  // all we have to do is add [amount] to the speed
  if (motor->direction == direction){
    motor->speed += amount;
  // otherwise we're accelerating in the opposite direction from the motor's
  // current motion, in which case we'll reduce speed by [amount]
  // and, if we've crossed 0 speed into negative territory, reverse the motor
  } else {
    motor->speed -= amount;
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

void do_estop(){

  estopped = true;
  left_motor.speed = 0;
  right_motor.speed = 0;

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

  tandem_accelerate(FORWARD);

}

//##############################################################################

void do_reverse(){

  tandem_accelerate(REVERSE);

}

//##############################################################################

void do_left(){

  accelerate(&left_motor,REVERSE,acceleration);
  accelerate(&right_motor,FORWARD,acceleration);

}

//##############################################################################

void do_right(){

  accelerate(&left_motor,FORWARD,acceleration);
  accelerate(&right_motor,REVERSE,acceleration);

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
