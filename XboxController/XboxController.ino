
#include < avr/io.h >
#include < avr/interrupt.h >
#include < TimerOne.h >


/************************************** SENSOR INPUT PIN DEFINITIONS **/
#define input_photoInterruptor 2      // Accelerator
#define input_pushButton 7            // Handbrake
#define input_pot_steer A0            // Steering
#define input_pot_brake A1            // Braking


/*********************** OUTPUT PIN DEFINITIONS (TO XBOX CONTROLLER) **/
#define output_handbrake 4            // Handbrake (digital)
#define output_brake 5                // Brake (analog)
#define output_throttle 6             // Throttle (analog)
#define output_steering 3             // Steering (analog)


/***************************************** SENSOR CALIBRATION VALUES **/
// Photo-interruptor
static const unsigned long minSpeed_period = 20000;
static const unsigned long maxSpeed_period = 13000;
static const unsigned long periodRange = minSpeed_period - maxSpeed_period;

//Steering
static const int steerLeft = 557;
static const int steerRight = 243;
static const float steerDivider = (steerLeft - steerRight) / 255.0;

//Braking
static const int brakeOn = 1020;
static const int brakeOff = 920;
static const double brakeMult = 255 / (brakeOn - brakeOff);


/********************************************************** SETTINGS **/
// Number of times to average sensor values
static const int numAverages = 100;


/***************************************** DECLARE WORKING VARIABLES **/
unsigned long period = maxSpeed_period;
unsigned long period_avg = 0;

unsigned int steerAngle = 0;
unsigned long steerAngle_tmp = 0;

unsigned int brakeStrength = 0;
unsigned int brakeStrength_tmp = 0;

int count = 0;
int tcnt2 = 131;  // Magic number?

bool overflow = false;

double x = 0.0;
double effort = 0.0;
unsigned char throttle_val = 0;

/****************************************** SETUP INTERRUPT ROUTINES **/
ISR(INT0_vect) {
  // This fires when a sprocket passes through the photo-interruptor
  if (digitalRead(input_photoInterruptor) == 1) {
    if (overflow == false) {
      period = Timer1.read();
      if (period < maxSpeed_period) period = maxSpeed_period;
      else if (period > minSpeed_period) period = minSpeed_period;
    }
    else {
      overflow = false;
    }
    Timer1.detachInterrupt();
    TCNT1 = 1;
    Timer1.initialize(minSpeed_period << 2 + 1);
    Timer1.attachInterrupt(timerIsr, minSpeed_period << 2);
  }
}

// Overflow function for Timer1
void timerIsr() {
  // Set overflow flag and update period
  period = minSpeed_period;
  overflow = true;
}


/**************************************************** SETUP THE CHIP **/
void setup()  {
  pinMode(output_throttle, OUTPUT);
  pinMode(output_brake, OUTPUT);
  pinMode(output_handbrake, OUTPUT);
  pinMode(output_steering, OUTPUT);

  pinMode(input_photoInterruptor, INPUT);
  pinMode(input_pushButton, INPUT);
  digitalWrite(input_pushButton,1);
  analogWrite(output_throttle, 255);

  Timer1.initialize(minSpeed_period << 2 + 1);
  Timer1.attachInterrupt( timerIsr, minSpeed_period << 2);

  noInterrupts();
  //Setup pin change interrupt
  EICRA = 0b00000011; //Configure INTO to trigger on rising edge
  EIMSK = 0b00000001; //External Interrupt request 0 Enable endabled
  interrupts();

  //Serial.begin(9600); // Setup serial port for debugging purposes
}


/******************************* HELPER FOR THE THROTTLE CALCULATION **/
unsigned char scale(unsigned long scale_input) {
  x = double(scale_input);
  x = x - maxSpeed_period;
  x = periodRange - x;
  x = x / double(periodRange);
  if (x > 1.0) x = 1.0;
  if (x < 0.0) x = 0.0;
  return 255 - char(255 * x);
}

/************************************************************* START **/
void loop()  {

  // Update the outputs (ready to average)
  if (count == numAverages) {

    // Throttle
    throttle_val = scale(period_avg / numAverages);
    analogWrite(output_throttle, throttle_val);

    // Handbrake
    digitalWrite(output_handbrake,digitalRead(input_pushButton));

    // Steering
    steerAngle = steerAngle / numAverages;
    analogWrite(output_steering, steerAngle);

    // Brake
    brakeStrength = brakeStrength / numAverages;
    analogWrite(output_brake, brakeStrength);

    // Reset the averaging counter and sensor sums
    count = 0;
    steerAngle = 0;
    brakeStrength = 0;
    period_avg = 0;
  }
  else {

    //Throttle
    period_avg += period; // period is always up-to-date thanks to ISR

    //Steering
    steerAngle_tmp = analogRead(A0);
    if (steerAngle_tmp < steerRight) steerAngle_tmp = steerRight;
    else if (steerAngle_tmp > steerLeft) steerAngle_tmp = steerLeft;
    steerAngle_tmp = steerAngle_tmp - steerRight;
    steerAngle_tmp = steerAngle_tmp / steerDivider;
    steerAngle += steerAngle_tmp;

    //Braking
    brakeStrength_tmp = analogRead(A1);
    if (brakeStrength_tmp > brakeOn) brakeStrength_tmp = brakeOn;
    else if (brakeStrength_tmp < brakeOff) brakeStrength_tmp = brakeOff;
    brakeStrength_tmp = brakeStrength_tmp - brakeOff;
    brakeStrength_tmp = brakeStrength_tmp * brakeMult;
    brakeStrength_tmp = 255 - brakeStrength_tmp;
    brakeStrength += brakeStrength_tmp;

    // Handbrake is on/off, no processing required.

    count++;
  }
}

