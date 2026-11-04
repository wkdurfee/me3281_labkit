/*----------------------------------------------------
 * PID position controller for lab kit
 * V 02
 * Author: W. Durfee
 --------------------------------------------------- */

/*------------------------------------------------------
  Includes
-------------------------------------------------------*/
#include <Encoder.h>

Encoder myEnc(D0,D1); //define encoder object

/*------------------------------------------------------
  Constants
-------------------------------------------------------*/
#define ENCODER_OPTIMIZE_INTERRUPTS 
#define IN1 D2 //motor driver pins
#define IN2 D3
#define PWMFREQ 10000  //in Hz (100-40000)
#define PWMLIM 1024 //PWM duty is (0-1024)
#define ENC_TO_DEG 0.25714 // 360 deg div by 1400 counts/rev
#define DEG_TO_ENC 3.88889 // inverse
#define ENC_COUNTS 1400. //encoder counts per rev (float)
#define NSAMP 200 //default number of samples per step response
#define SAMPLE_PERIOD 10 //control loop sampling period (ms)

// Commands from parent
#define DEVICE_CHECK 101 //see if device is attached
#define DO_STEP 102      //run a step response
#define SET_CONTROLLER  103
#define SET_KP          104
#define SET_KI          105
#define SET_KD          106
#define SET_STEP_SIZE   107
#define SET_FRICT_COMP  108
#define SET_END_TIME    109
#define TEST_1          120

/*------------------------------------------------------
  Global variables
-------------------------------------------------------*/
byte control_type; //from GUI
byte p_gain, i_gain, d_gain; //gain values from GUI
byte step_size; //step size from GUI, in degrees
byte end_time; //end time from GUI, in sec, 2 to 15
byte frict_comp; //friction compensation, from GUI
float target, measured; //positions, in encoder units 
float u; //motor command
float kp, ki, kd; //scaled control gains
float err, last_err, err_sum, delta_err; //for PID
float i_term; //for PID
float u_max, u_min;
int motor_command, direction;
unsigned long now, startTime;

/*****************************************************
  Functions
******************************************************/
/*------------------------------------------------------
  Flash the LED n times; a convenience function
-------------------------------------------------------*/
void flash_led(int n) {
  for (int i=0;i<n;i++){
    digitalWrite(LED_BUILTIN, HIGH);  
    delay(500);                      
    digitalWrite(LED_BUILTIN, LOW);   
    delay(500);                     

  }
}

/*------------------------------------------------------
  Drive the motor
-------------------------------------------------------*/
void run_motor(int dir,int cmd) 
//dir=(0,1); cmd=(0,1024); uses braking mode (other channel is full on)
{
  int pwm_duty = 1024 - cmd;
  if(dir) { 
    pwm(IN1, PWMFREQ, 1024);
    pwm(IN2, PWMFREQ, pwm_duty);
  }
  else {
    pwm(IN1, PWMFREQ, pwm_duty);
    pwm(IN2, PWMFREQ, 1024); 
  }
}

/*------------------------------------------------------
  Kill the motor
-------------------------------------------------------*/
void motor_off()
//turn off motor
{
  noPwm(IN1); noPwm(IN2); 
}

/*------------------------------------------------------
  Scale the PID gains that come in as [0,100]
  Scaling factors are empirical. 
-------------------------------------------------------*/
void scale_pid_gains() 
{
  float p_scale,i_scale,d_scale;
  p_scale=50.;  //scaling factorsu
  i_scale=2500.;
  d_scale=1.00;
  kp = (float)p_gain/p_scale; 
  ki = ((float)i_gain*(float)SAMPLE_PERIOD)/i_scale;
  kd = ((float)d_gain/d_scale)/(float)SAMPLE_PERIOD;
}


/*------------------------------------------------------
  Send 16-bit, signed integer data to parent, LSB first
-------------------------------------------------------*/
void send_to_parent(int data) 
{
  Serial.write((byte)(data & 0xFF));  //pick and send off lower 8 bits
  Serial.write((byte)(data >> 8));
}

/*------------------------------------------------------------------
  PID algorithm. Called every sample period. Uses global variables
-------------------------------------------------------------------*/
void compute_PID()
{
  measured = (float)myEnc.read();
  err = target - measured;
  err_sum += err;
  delta_err = err - last_err;  //also try deriv on measured (see "derivative kick")
  i_term = ki*err_sum; //compute new integral term  [later try tuning trick]
  if (i_term>u_max) i_term = u_max; //cap on integral term to prevent windup
  if (i_term<u_min) i_term = u_min;
  u = kp*err + i_term + kd*delta_err; //compute PID output
  if (u>0) u = u + frict_comp; //add friction compensation
  if (u<0) u = u - frict_comp; 
  if (u>u_max) u = u_max; //cap output
  if (u<u_min) u = u_min;
  startTime = now; //update past values
  last_err = err;
}

/*------------------------------------------------------
  Run a step response test
-------------------------------------------------------*/
void run_step() 
{ 
  unsigned int i;
  float position;
  int nsamp = (end_time) * (1000/SAMPLE_PERIOD); //number of samples per step response
  int data[nsamp] = {0}; //to hold position data
  err_sum = 0; //initialze
  last_err = 0; 
  myEnc.write(0); //reset encoder; sets current position = 0
  scale_pid_gains(); //set the current PID gains
  target = (float)step_size * DEG_TO_ENC; //scale target to encoder units
  now = millis(); //initialize clock
  startTime = now;
  for (i=0;i<nsamp;i++) { //the control loop
    while ((now - startTime) < SAMPLE_PERIOD) { //wait for next clock tick
      now = millis();
    }
    compute_PID();
    direction = 0; //set direction
    if (u < 0) direction = 1;
    motor_command = (int)abs(u);
    run_motor(direction,motor_command); //drive motor
    position = 10 * measured * ENC_TO_DEG; //current position in deg*10 units
    data[i] = (int)position; //units = deg*10
  }
  motor_off(); //done; turn off motor
  for (i=0;i<nsamp;i++) { //send data to parent
    send_to_parent(data[i]);
  }
}

/*------------------------------------------------------
  Debugging test, not implemented
-------------------------------------------------------*/
void run_debug()
{
}

/*------------------------------------------------------
  Do the task directed by command from parent
-------------------------------------------------------*/
void handle_request(byte parent_cmd, byte parent_data) 
{
  if (parent_cmd==DEVICE_CHECK) Serial.write("y");
  if (parent_cmd == DO_STEP) run_step();
  if (parent_cmd == SET_CONTROLLER) control_type = parent_data; 
  if (parent_cmd == SET_KP) p_gain = parent_data;
  if (parent_cmd == SET_KI) i_gain = parent_data;
  if (parent_cmd == SET_KD) d_gain = parent_data;
  if (parent_cmd == SET_STEP_SIZE) step_size = parent_data;  
  if (parent_cmd == SET_FRICT_COMP) frict_comp = parent_data;   
  if (parent_cmd == SET_END_TIME) end_time = parent_data;  
  if (parent_cmd == TEST_1) run_debug();     
}

/*------------------------------------------------------
  Setup
-------------------------------------------------------*/
void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  Serial.begin(19200);
  Serial.flush();
  u_max = PWMLIM; //set control limits
  u_min = -PWMLIM;
  myEnc.write(0); //reset encoder  
}

/*------------------------------------------------------
  Loop
  Waits for a command from the user
-------------------------------------------------------*/
void loop()
{
  byte parent_cmd, parent_data;
  if (Serial.available() > 1) { //wait until 2 bytes are available
    parent_cmd  = Serial.read(); //first byte is command
    parent_data = Serial.read(); //second byte is data
    handle_request(parent_cmd,parent_data); //handle the request
  }
}

  
