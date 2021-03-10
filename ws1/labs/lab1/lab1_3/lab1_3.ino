// Lab1_3.ino
// ESP32-IoT workshop by dew.ninja
// July 2019
// for Wemos LOLIN32

// *** Add OLED display ***** 

// **** include and global declaration *******

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire,OLED_RESET);

// ***********************************************

const int PWMOut = 16;    // use GPIO16 as PWM
const int ADCin = A3;   //   use A3 as plant output y
const int SQWOut = LED_BUILTIN;  // square-wave output, also LED onboard
// timer channels
const byte PWMch=1;
const byte REDch = 2;
const byte GREENch = 3;
const byte BLUEch = 4;

const int PWMMAX = 4095;    // 12-bit PWM
const int PWMMID = 2047;

// ******************* RGB pin assignments and values **************
const int RLED=19, GLED=18, BLED=17;   // pin numbers for RGB led
int Rval=0, Gval=0, Bval=0;   // values for RGB color
// *********************************************************************


float y = 0;     // plant output (in volts)
float r = 1;     // command value (in volts)
float u = r;      // controller output (in volts)
float raw2v = 3.3/4095;   // 12-bit raw unit to volt

int adcval = 0;         // ADC input value
int pwmval = 0;       // pwm value
int dacval = 0;

// initialize RGB-LED
void RGBled_init(void)
{
    // ******* RGB LED outputs *****************
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  ledcSetup(REDch, 5000, 12);   // Red LED
  ledcSetup(GREENch, 5000, 12);   // Green LED
  ledcSetup(BLUEch, 5000, 12);   // Blue LED

  ledcAttachPin(RLED, REDch);
  ledcAttachPin(GLED, GREENch);
  ledcAttachPin(BLED, BLUEch);  

  
}
// initialize PWM output
void PWM_init(void)
{
  pinMode(PWMOut, OUTPUT);

  ledcSetup(PWMch, 5000, 12);   // 12-bit PWM output
  ledcAttachPin(PWMOut, PWMch);  
    
}

void OLED_init(void)
{
  // ******* setup OLED display **********************
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c); //0x78>>1);
  display.display();  // display Adafruit logo
  delay(2000);
  display.clearDisplay();
  // ************************************************  
}


void setup() {
  pinMode(SQWOut, OUTPUT);
  PWM_init();
  RGBled_init();
  Serial.begin(115200);
  // ----------Print lab number to serial -------
  Serial.println(" ");
  Serial.println("--- Lab 1.3 : using OLED display ---");
  Serial.println("--- ESP32-IoT workshop by dew.ninja ---");
  // ----------------------------------------
  

  
  pwmval = int(u/raw2v);

  OLED_init();
  
}

void loop() {
  digitalWrite(SQWOut,!digitalRead(SQWOut)); // toggle on-board LED
  adcval = analogRead(ADCin);      // read analog voltage
  y = raw2v*adcval;   // convert to volts
  ledcWrite(PWMch, pwmval);       // write to PWM
  dacval = pwmval>>4; 
  dacWrite(DAC1, dacval);
  // ************* send data to OLED ********
  showOLED(r,y,u);  
  // ******************************************
  esp_y2rgb();
  delay(100);    // delay 100 millisecs
}

// ***************** send data to OLED *********************
void showOLED(float r, float y, float u)   {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("r: ");
  display.print(r);
  display.println(" V");
  display.print("y: ");
  display.print(y);
  display.println(" V");
  display.print("u: ");
  display.print(u);
  display.println(" V");
  display.setTextSize(1);
  display.setTextColor(WHITE);  
  display.println("-----------------");
  display.println("ESP32-IoT : dew.ninja");
  display.display();
  display.clearDisplay();

}
// ***********************************************************

// set RGB led to specified color
void lidRGBled(int rval, int gval, int bval)
{ 
  
  ledcWrite(REDch, rval);
  ledcWrite(GREENch, gval);
  ledcWrite(BLUEch, bval); 
}

// lid RGB led according to output level
void esp_y2rgb(void)
{
  float yt;
  int ypwm, rval, gval, bval;

  if (y<=1.5)   {
    yt=y/1.5;
    rval =0;
    bval = int(PWMMAX*(1-yt));
    gval = int(PWMMAX*yt);
    lidRGBled(rval, gval, bval);
    
  }
  else if (y>1.5)  {
    yt = (y-1.5)/1.5;
    if (yt>1) yt=1;
    bval = 0;
    gval = int(PWMMAX*(1-yt));
    rval = int(PWMMAX*yt);
    lidRGBled(rval, gval, bval);
  }
}
