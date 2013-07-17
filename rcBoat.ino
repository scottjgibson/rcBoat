//Arduino based remote control GoKart
//Allows for control using a standard RC TX/RX
//Arduino interprets the output of the receiver and controls
//larger motors using an external h-bridge
//Steering is accomplished using an external potentiometer 
//attached to the steering motor to feedback the current
//wheel direction

#include <RCArduinoFastLib.h>
#include <PinChangeInt.h>

//External Switches

//Input from RC Receiver
#define THROTTLE_IN_PIN 5
#define STEERING_IN_PIN 6
#define AUX_IN_PIN 7

//Outputs to Speed control mosfet
#define THROTTLE_PWM_PIN 8

//Output to direction relay (no h-bridge right now)
#define MOTOR_DIR_PIN 9


// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_STEERING 1
#define SERVO_AUX 2
#define SERVO_FRAME_SPACE 3

#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define AUX_FLAG 4
volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unAuxInShared;
uint16_t unThrottleInStart;
uint16_t unSteeringInStart;
uint16_t unAuxInStart;
uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("RCBoat 1.0");
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 1 Servos + 9 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,9*2000);
  CRCArduinoFastServos::begin();
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering,CHANGE);
  PCintPort::attachInterrupt(AUX_IN_PIN, calcAux,CHANGE);

  pinMode(THROTTLE_PWM_PIN, OUTPUT);
  analogWrite(THROTTLE_PWM_PIN, 0);

  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);

}

void loop()
{
  static int throttle;
  static int steering_set;
  static int steering_feedback;
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unAuxIn;
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
  
    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
  
    if(bUpdateFlags & AUX_FLAG)
    {
      unAuxIn = unAuxInShared;
    }
   
    bUpdateFlagsShared = 0;
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
  }

  if(bUpdateFlags & THROTTLE_FLAG)
  {
    throttle = map(unThrottleIn, 1500,2500,0,255);
    analogWrite(THROTTLE_PWM_PIN, map(unThrottleIn, 1500,2500,0,255));
  }

//Not used; attach servo directly
  if(bUpdateFlags & STEERING_FLAG)
  {
  }

//Use this channel to set the direction
  if(bUpdateFlags & AUX_FLAG)
  {
    //This channel is a switch on my transmitter
    //if switch is enabled; trigger the relay which swaps the motor polarity
    if (map(unAuxIn, 1500,2500,0,100) > 30)
    {
      digitalWrite(MOTOR_DIR_PIN, HIGH);
    }
  }

#if DEBUG
  Serial.print("throttle in (uSec):");
  Serial.println(unThrottleIn);
  Serial.print("throttle val (-255-255):");
  Serial.println(throttle);
  Serial.print("steering in (uSec):");
  Serial.print(unSteeringIn);
  Serial.print("steering in (degrees):");
  Serial.println(map(unSteeringIn, 1500, 2500, -90, 90));
  Serial.print("dir in: (uSec):");
  Serial.println(unAuxIn);
  delay(500);
#endif
  delay(10);
  bUpdateFlags = 0;
}


// simple interrupt service routine
void calcThrottle()
{
  if(PCintPort::pinState)
  {
    unThrottleInStart = TCNT1;
  }
  else
  {
    unThrottleInShared = (TCNT1 - unThrottleInStart)>>1;
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(PCintPort::pinState)
  {
    unSteeringInStart = TCNT1;
  }
  else
  {
    unSteeringInShared = (TCNT1 - unSteeringInStart)>>1;

    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void calcAux()
{
  if(PCintPort::pinState)
  {
    unAuxInStart = TCNT1;
  }
  else
  {
    unAuxInShared = (TCNT1 - unAuxInStart)>>1;
    bUpdateFlagsShared |= AUX_FLAG;  }
}
