/* 
***DOCUMENTATION SHEET***

this code makes a motor controller for eight (number can be modified using NUM_MOTORS constant) afro 30A ESCs. it 
can be interacted with through pins 0 and 1 (default rx and tx), so it can also be tested with the IDE monitor. on startup, if all goes well, 
you should hear a beep from each ESC, then five seconds later each motor will spin at 10% power for 500 ms, one at a time. this startup
sequence can be disabled by setting the constant STARTING_SEQUENCE to 0 (1 is enabled, 0 is disabled). obviously, make sure you have it disabled
when you actually go into the water. if instead you start hearing frequent beeps after eight seconds, refer to the ESC's manual, 
as it means it's not receiving any signals. also keep in mind they need to receive this signal immediatly after their startup, 
so you should reset them and the arduino at the same time when starting.

there is a debug constant named SEND_RESPONSE; if set to 1, it will send a 0 for an invalid motor, a 1 for an out of bounds speed, and
a 2 for a success. if set to 2, it will give all above data, as well as echo the full buffer minus the colon once it has fully processed it,
("Full input: (your string here)") and all data it parses ("Parsed data: (speed, motor)"). keep in mind that if either data point is
invalid, it will be parsed as 0. otherwise, it will not send any data back

the format is as follows:
(speed as float from -1 to 1, works as a percentage. max two decimal points),(motor as int from 2-9):
eg. 0.5,3:
this turns the second motor at 50%. also, the colon is REQUIRED, and keep in mind the motors start at 2, not 0.
commands can also be chained,
eg. 0.5,3:0.1,4:0.6,7:
keep in mind the motor will always keep whatever its previous command is until it gets a new one, so to turn it off you need to send 0 to
all motors.

the code isn't too complicated, so for any questions/problems, you should be able to look over the code and find the answer. there are also
comments which should hopefully explain anything you don't understand


in case of a bad speed input, the speed will be set to 0, so the motor you are trying to set will be stopped (assuming a valid motor). if the motor
is invalid, the motor number will be set to 0, and as the motor numbers begin at 2, nothing will happen
*/


/*
TODO
- out of bounds speed fix, not just set to 0 (not sure how to do this but its prob possible)
- config about starting pin
- more responses/data
- document all responses better
*/




// ### CONFIG BEGIN ###

#define NUM_MOTORS 4
#define STARTUP_SEQUENCE 0
// 0 off, 1 on
#define SEND_RESPONSE 1
// 0 no response, 1 end status response, 2 full data

#define MAX_MOTOR_SPEED 30
// the maximum motor speed as a percent of 100

// manual override config
#define OVERRIDE_PIN 8
#define LEFT_RIGHT_MANUAL_PIN 9
#define FORWARD_BACKWARD_MANUAL_PIN 10
#define OVERRIDE_DEFAULT_PWM 1000
#define OVERRIDE_ACTIVATED_PWM 5000
// these PWM values are just guesstimated. they define what should be counted as 'on' and what as 'off'
// all PWM values are in terms of microseconds
#define LEFT_RIGHT_MANUAL_MIN_PWM 1000
#define LEFT_RIGHT_MANUAL_MAX_PWM 5000
#define FORWARD_BACKWARD_MANUAL_MIN_PWM 1000
#define FORWARD_BACKWARD_MANUAL_MAX_PWM 5000


int manual_directions[4][NUM_MOTORS] = {
  {1, 1, 1, 1},
  {-1, 1, -1, 1},
  {-1, -1, -1, -1},
  {1, -1, 1, -1}
};
// the direction each motor should spin in for each configuration, 1 is forward, -1 is backward. first forward, then right, then backward, then left

// changing these variables (especially INVALID_MOTOR_RESPONSE) may cause issues, use with caution
#define INVALID_MOTOR_RESPONSE 0
#define OUT_OF_BOUNDS_SPEED_RESPONSE 1
#define SUCCESS_RESPONSE 2

// ### CONFIG END ###






#define MAXBUF 256 // maximum size for the buffer


#include <Servo.h>
#include <math.h>


Servo motors[NUM_MOTORS];


void forward(float percent, int motor) {
  int speed = map(constrain(floor(percent*100.0), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED), -100, 100, 0, 180);
  if(speed >= 0 && speed <= 180) {
    motors[motor].write(speed);
  }
}

void setup() {
  Serial.begin(9600); // default serial opens on pins 0 and 1, you can also use the serial monitor
  for(int i = 0; i < NUM_MOTORS; i++) {
    motors[i].attach(i+2, 1000, 2000); // we start at two because pins 0 and 1 are used for serial
    forward(0.5, i); // send neutral signal to motors, required for the afro 30A ESC startup- this needs to be modified for different ESCs
  }
  delay(5000); // wait five seconds to give time for ESC startup
  if(STARTUP_SEQUENCE == 1) {
    for(int i = 0; i < NUM_MOTORS; i++) {
      forward(0.6, i);
      delay(500);
      forward(0.5, i); // turn motor on, wait 500ms, turn off
    }
  }
}

char buf[MAXBUF];
int charsinbuf = 0;

int channel_pins[3] = {OVERRIDE_PIN, LEFT_RIGHT_MANUAL_PIN, FORWARD_BACKWARD_MANUAL_PIN};
long channel_peaks[3] = {0, 0, 0};
long channel_signals[3] = {-1, -1, -1};
// [0] -> override, [1] -> left-right, [2] -> forward-backward

void loop() {

  charsinbuf = 0; // reset buffer

  // the point of having a custom buffer is that if you were to try to read character-by-character from the serial, you would read 
  // faster than it is sent, and the code would think halfway through is the whole command; this way, we can reliably 
  // get the entire command as one string

  for(int i = 0; i < 3; i++) {
    int pin = channel_pins[i];
    long peak = channel_peaks[i];
    bool signal = digitalRead(pin);
    if(signal && (peak == 0)) {
      channel_peaks[i] = micros();
    }
    if(!signal && (peak != 0)) {
      long current = micros();
      channel_signals[i] = current - peak;
      channel_peaks[i] = 0;
    }
  }
  
  // we can't expect it to be exact, so as long as it's halfway there we'll count it
  if(channel_signals[0] > (OVERRIDE_ACTIVATED_PWM - OVERRIDE_DEFAULT_PWM) / 2) {
    float strafe_percent = (channel_signals[1] - LEFT_RIGHT_MANUAL_MIN_PWM) / (LEFT_RIGHT_MANUAL_MAX_PWM - LEFT_RIGHT_MANUAL_MIN_PWM);
    float forward_percent = (channel_signals[2] - FORWARD_BACKWARD_MANUAL_MIN_PWM) / (FORWARD_BACKWARD_MANUAL_MAX_PWM - FORWARD_BACKWARD_MANUAL_MIN_PWM);

    if(strafe_percent < 0.1) {
      strafe_percent = 0.0;
    }
    if(forward_percent < 0.1) {
      forward_percent = 0.0;
    }

    float radius = sqrt(strafe_percent*strafe_percent + forward_percent*forward_percent);
    
    if(radius < 0.01) {
      for(int j = 0; j < NUM_MOTORS; j++) {
        forward(0.0, j);
      }
      return;
    }
    

    float angle = atan2(forward_percent, strafe_percent);
    float next_step = ceil(angle/M_PI_2) * M_PI_2;
    float prev_step = floor(angle/M_PI_2) * M_PI_2;
    float t = (angle - prev_step) / (next_step - prev_step);
    
    float prev_directions[NUM_MOTORS] = manual_directions[round(prev_step / M_PI_2) % 4];
    float next_directions[NUM_MOTORS] = manual_directions[round(next_step / M_PI_2) % 4];

    for(int j = 0; j < NUM_MOTORS; j++) {
      float direction = prev_directions[j] + t * (next_directions[j] - prev_directions[j]);
      forward(direction * radius, j);
    }
    return;
  }
  

  while((charsinbuf == 0) || (buf[charsinbuf-1] != ':')) { 
    // until we find a colon (end character), we also ignore the first time because it would try to read out-of-bounds (index -1)
    if(Serial.available() > 0) {
      buf[charsinbuf] = Serial.read();
      charsinbuf++;
    }
    if (charsinbuf == MAXBUF) { // reset if it hits the max
      charsinbuf = 0;
    }
  }

  buf[charsinbuf-1] = '\0'; // get rid of the colon, and replace it with the string end character
  if(SEND_RESPONSE == 2) {
    Serial.print("Full input: ");
    Serial.println(buf);
  }


  char *spd = strtok(buf, ","); // get the speed as a string from everything up to the first comma
  float speed = atof(spd); // try to turn it into a float, if it isn't numerical it becomes 0.0
  char *mtr = strtok(NULL, ","); // using NULL finds the next number
  int motor = atoi(mtr); // parse int, otherwise 0

  if(SEND_RESPONSE == 2) {
    Serial.print("Parsed data: ");
    Serial.print(speed);
    Serial.print(", ");
    Serial.println(motor);
  }

  if(speed > 1.0 || speed < -1.0) {
    if(SEND_RESPONSE > 0) {
      Serial.println(OUT_OF_BOUNDS_SPEED_RESPONSE);
    }
    return;
  }

  if(motor < 2 || motor >= 2 + NUM_MOTORS) {
    if(SEND_RESPONSE > 0) {
      Serial.println(INVALID_MOTOR_RESPONSE);
    }
    return;
  }

  // two decimal places of accuracy
  // we multiply by 100 and floor it because the map function only accepts ints, so we set the bounds to -100 to 100
  forward(speed, motor-2);
  // motor is subtracted by two to bring it back to 0. the only reason to have it at 2 in the first place is to be able to differentiate from
  // the first motor (would be index 0) and the output of the atoi function given an invalid motor (would also be 0)
  // one advantage is that with this the number you send should be the same as the pin numbers of the motors
    if(SEND_RESPONSE > 0) {
      Serial.println(SUCCESS_RESPONSE);
    }
  }
