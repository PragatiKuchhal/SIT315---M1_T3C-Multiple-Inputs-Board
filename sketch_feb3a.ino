#include <avr/interrupt.h>

//Define LED pin as Digital pin 4
const uint8_t ledPin = 4;

//Define Red Led Pin as Digital pin 13
const uint8_t RedLedPin = 13;

//Define Blue Led Pin as Digital pin 12
const uint8_t BlueLedPin = 12;

//Define Green Led Pin as Digital pin 8
const uint8_t GreenLedPin = 8;

//Define Meter Pin as Analog Pin 4
const uint8_t meterPin = A4;

//Define PIR Sensor Pin as Digital pin 6
const uint8_t pirPin = 6;

//Define Push Button as Digital pin 2
const uint8_t pushButton = 2;

//Define Ultrasonic Sensor Pin as Digital pin 7
const uint8_t ultrasonicPin = 7;

//Variable to read potentiometer value
double potentiometerValue;

//Variable to read PIR state
int pirState;

//Variable to read button state
int buttonState;

//Variable to read ultrasonic state
int ultrasonicState;

//put your setup code here, to run once:
void setup()
{
  //Disable all interrupts
  cli();
  
  //Enables Ports C and D Pin Change Interrupts
  PCICR |= 0b00000110;
  
  //Activate the interrupts on pin A4
  PCMSK1 |= 0b00010000;
  
  //Activate the interrupts on pin D2,D6 and D7
  PCMSK2 |= 0b11000100;
  
  //Enable all interrupts
  sei();
  
  //Initialize LED Pin as Output
  pinMode(ledPin, OUTPUT);
  
  //Initialize Red Led Pin as Output
  pinMode(RedLedPin, OUTPUT);
  
  //Initialize Blue Led Pin as Output
  pinMode(BlueLedPin, OUTPUT);
  
  //Initialize Green Led Pin as Output
  pinMode(GreenLedPin, OUTPUT);
  
  //Initialize Meter Pin as Input
  pinMode(meterPin, INPUT);
  
  //Initialize PIR Sensor Pin as Input
  pinMode(pirPin, INPUT);
  
  //Initialize Push Button as Input
  pinMode(pushButton, INPUT);
  
  //Initialize Ultrasonic Sensor Pin as Input
  pinMode(ultrasonicPin, INPUT);
  
  //Serial data transmission begins at 9600 bps
  Serial.begin(9600);
  
  //Read potentiometer value
  potentiometerValue = analogRead(meterPin);
  
  //Read PIR state
  pirState = digitalRead(pirPin);
  
  //Read button state
  buttonState = digitalRead(pushButton);
  
  //Read ultrasonic state
  ultrasonicState = digitalRead(ultrasonicPin);
  
  //Maps potentiometer reading to frequency
  potentiometerValue = potentiometerValue/1023;
  
  //Configure timer frequency using potentiometer
  startTimer(potentiometerValue);
}

//put your main code here, to run repeatedly
void loop()
{
}

void startTimer(double timerFrequency)
{
  //Disable all interrupts
  noInterrupts();
  
  //Set entire TCCRIA register to 0
  TCCR1A = 0;
  //SET entire TCCRIB register to 0
  TCCR1B = 0;
  //Initialize counter value to 0
  TCNT1 = 0;
  
  //Prescale value taken is 1024
  //Arduino uno frequency is 16MHz
  //OCIRA = 16000000 / (prescale value * frequency we need)
  
  //Compare match register 16MHz/1024/timerFrequency
  OCR1A = 16000000 / (1024 * timerFrequency);
  
  //CTC mode
  TCCR1B |= (1 << WGM12);
  
  //1024 prescaler)
  TCCR1B |= (1 << CS12) | (1 << CS10);
  
  //Using OCIE1A bit, interrupt is enabled
  TIMSK1 |= (1 << OCIE1A);
  
  //Enable aa interrupts
  interrupts();
}

//Timer compare interrupt service routine
ISR (TIMER1_COMPA_vect)
{
  //Toggle LED pin
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
}

//Pin Change Interrupt Port D
ISR (PCINT2_vect)
{
  if (pirState == 0)
  {
    Serial.print("Pir Motion State: ");
    Serial.println(pirState);
    //Toggle Red LED pin
    digitalWrite(RedLedPin, digitalRead(RedLedPin) ^ 1);
  }
  
  if (ultrasonicState == 0)
  {
    Serial.print("Ultrasonic Motion State: ");
    Serial.println(ultrasonicState);
    //Toggle Green LED pin
    digitalWrite(GreenLedPin, digitalRead(GreenLedPin) ^ 1);
  }
  
  if (buttonState == 1)
  {
    Serial.print("Button State: ");
    Serial.println(buttonState);
    //Toggle Blue LED pin
    digitalWrite(BlueLedPin, digitalRead(BlueLedPin) ^ 1);
  }
}

//Pin Change Interrupt Port C
ISR (PCINT1_vect)
{
  Serial.print("Potentiometer Value: ");
  Serial.println(potentiometerValue);
  //Toggle LED pin
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
}
  