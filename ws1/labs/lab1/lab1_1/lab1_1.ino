// lab1_1.ino 
//IoT workshop by dew.ninja
// July 2019
// For LOLIN32 board
// test PWM and external interrupt

const int PWMOut = 15;    // use GPIO15 as PWM
const int Din = 14;      // GPIO14 as digital input
//const int ADCin = A3;   //   only one ADC input available
// timer assignment
const byte PWMch=1;
//const int LED_ONBOARD = 5;
unsigned long us_new, us_old,dtimeh, dtimel;  // variables used to capture period
bool pwmlogic;      // logic of pwm


int pwmval = 1023;       // constant pwm value 25%


void setup() {
  pinMode(Din, INPUT);
  pinMode(PWMOut, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // ------ add PWM and external interrupt setup here -----------
  ledcSetup(PWMch, 5000, 12);   // PWM output
  ledcAttachPin(PWMOut, PWMch);
  attachInterrupt(Din, measureP, CHANGE);
  // -------------------------------------
  Serial.begin(115200);
  ledcWrite(PWMch, pwmval);
  
}

void loop() {
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN)); // toggle square wave output

  Serial.print("PWM : high period = ");
  Serial.print(dtimeh);
  Serial.print(" microseconds");
  
  Serial.print(", low period = ");
  Serial.print(dtimel);
  Serial.println(" microseconds");   
  delay(1000);    // delay 100 millisecs
}

// measure period of PWM
void measureP(void)
{
  us_old = us_new;
  us_new = micros();
  pwmlogic = digitalRead(Din);  
  if (!pwmlogic) // compute high period
  {
    dtimeh = us_new - us_old;
  }
  else  
  {
    dtimel = us_new - us_old;
  }
}
