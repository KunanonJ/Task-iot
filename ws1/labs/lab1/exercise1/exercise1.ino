// exercise1.ino 
// modify lab1_2.ino such that
// red LED blinks "independent of" on-board LED
//IoT workshop by dew.ninja
// July 2019
// For LOLIN32 board
// test timer interrupt

// timer assignment
const byte Timer0=0;
const int REDLED = 19;

float T = 0.05;   // in second
long T_us;      // period in microsecond
int divider=80;
unsigned long ms_new, ms_old, tperiod;  // varialbes used to captuer period
hw_timer_t * timer = NULL;

void setup() {
 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(REDLED, OUTPUT);
  Serial.begin(115200);
  timer_init();

}

void loop() {
  digitalWrite(REDLED,!digitalRead(REDLED)); 
  Serial.print("Timer period = ");
  Serial.print(tperiod);
  Serial.println(" milliseconds");
  delay(1000);    // print every 1 sec
}

void IRAM_ATTR onTimer() {

  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN)); // toggle square wave output
  ms_old = ms_new;
  ms_new = millis();
  tperiod = ms_new - ms_old;

}

void timer_init(void)   {
  T_us = long(T*1000000);
  timer = timerBegin(Timer0, divider, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, T_us, true);
  timerAlarmEnable(timer);
}
