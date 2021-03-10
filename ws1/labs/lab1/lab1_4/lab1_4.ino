// Lab1_4.ino 
// IoT-ESP32 workshop by dew.ninja
// July 2019
// Program for NodeMCU-32S
// *** Add command interpreter ****
// Update : March 2021
//   * remove OLED commands
//   * format output to Python arrays
//   * add time data to the output

const int PWMOut = 16;    // use GPIO16 as PWM
const int ADCin = A3;   //   use A3 as plant output y
const int SQWOut = LED_BUILTIN;  // square-wave output, also LED onboard
// timer channels
const byte PWMch=1;
const byte REDch = 2;
const byte GREENch = 3;
const byte BLUEch = 4;


// ******************* RGB pin assignments and values **************
const int RLED=19, GLED=18, BLED=17;   // pin numbers for RGB led
int Rval=0, Gval=0, Bval=0;   // values for RGB color
// *********************************************************************

const int DATASIZEMAX = 5000;   // maximum data size


// voltage range
const float VMAX = 3;
const float VMID = 1.5;
const float VMIN = 0;

// PWM and ADC ranges 
const int PWMMAX = 4095;  // 12-bit PWM
const int PWMMID = 2047;
const int PWMMIN = 0;
const int ADCMAX = 4095;   // 12-bit ADC
const int ADCMID = 2047;
const int ADCMIN = 0;

float y = 0;     // plant output (in volts)
float r = 0.1, rold = 0.1;     // command value (in volts)
float u = 0.1;      // controller output (in volts)
float raw2v = 3.3/4095;   // 12-bit raw unit to volt

// --- global variables for sampling period and time data ----
float T = 0.05;   // sampling period
unsigned long T_ms;  // sampling period in milliseconds
unsigned long previous_time = 0, current_time = 0; 
float tdata = 0;  // time data sent to output


int i = 0;          // index
String rcvdstring;   // string received from serial
String cmdstring;   // command part of received string
String  parmstring; // parameter part of received string
int noparm = 0;    // flag if no parameter is passed
int parmvalint;        // parameter value     
float parmvalfloat;
int newcmd = 0;     // flag when new command received
int sepIndex;       // index of seperator
int datasize = 100;  // number of data points
int datacapt = 0;

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


void setup() {
  pinMode(SQWOut, OUTPUT);
  PWM_init();
  RGBled_init();
  Serial.begin(115200);

  // ----------Print lab number to serial -------
  Serial.println(" ");
  Serial.println("--- Lab 1.4 : add command interpreter ---");
  Serial.println("--- IoT-ESP32 workshop by dew.ninja ---");
  // ----------------------------------------
  
  T_ms = 1000*T;
  pwmval = int(u/raw2v);
  
}

void loop() {
  current_time = millis();
  if ((current_time - previous_time)>T_ms) { 
    previous_time = current_time;
    digitalWrite(SQWOut,!digitalRead(SQWOut)); // toggle on-board LED  
    // ************* check for new command from serial port ****************
    while (Serial.available() > 0 )   {
          rcvdstring = Serial.readString();
          newcmd = 1;
    }
    if (newcmd)   { // execute this part only when new command received
      cmdInt();   // invoke the interpreter
      newcmd = 0;
    }
    // *****************************************************************
    
    adcval = analogRead(ADCin);      // read analog input
    y = raw2v*adcval;          // convert to volt
    u = r;
    pwmval = int(u/raw2v);
    ledcWrite(PWMch, pwmval);       // write to PWM
    dacval = pwmval>>4; 
    dacWrite(DAC1, dacval);     // also write to DAC
    tdata += T;   // update time data
    esp_y2rgb();  // lid RGB LED
    
    if (datacapt)   {
      // send data to host
      Serial.print("[");
      Serial.print(tdata);
      Serial.print(", ");
      Serial.print(r);
      Serial.print(", ");
      Serial.print(y);   
      Serial.print(", ");
      Serial.print(u);
      Serial.println("],");
      i++;
      if (i== datasize)  {
        datacapt = 0;      // reset the command change flag
        i = 0;              // reset index
        Serial.println("])");
  
      }
    }
  }
}



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


// ************ command interpreter implementation ****************
void cmdInt(void)
{
    rcvdstring.trim();  // remove leading&trailing whitespace, if any
    // find index of separator '='
    sepIndex = rcvdstring.indexOf('=');
    if (sepIndex==-1) {
      cmdstring = rcvdstring;
      noparm = 1;
    }
    else  {
    // extract command and parameter
      cmdstring = rcvdstring.substring(0, sepIndex);
      cmdstring.trim();
      parmstring = rcvdstring.substring(sepIndex+1); 
      parmstring.trim();
      noparm = 0;
    }
    // check if received command string is a valid command
    if (cmdstring.equalsIgnoreCase("step")|cmdstring.equalsIgnoreCase("r"))   {
      if (noparm==1)   {  // step to 1
        r = 1;
        rold = r;
      }
      else  {  // step to new spedified value
         parmvalfloat = parmstring.toFloat();

        //limit step command to 0 - 3 volts
        if (parmvalfloat > VMAX) parmvalfloat = VMAX; 
        else if (parmvalfloat< VMIN) parmvalfloat = VMIN;

        r = parmvalfloat;
        rold = r;   // save previous command
      }
      Serial.println("datamat = np.array([");
      tdata = 0;   // reset time data variable
      datacapt = 1;    // set the flag to capture data
      i = 0;              // reset data index
    }
    else if (cmdstring.equalsIgnoreCase("datasize"))   {
      if (noparm==1)   {
        Serial.print("Current datasize = ");
        Serial.println(datasize);       
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint > DATASIZEMAX) parmvalint = DATASIZEMAX; // limit datasize to DATASIZEMAX
        else if (parmvalint<0) parmvalint = 0;
        datasize = parmvalint;  
        Serial.print("Datasize set to ");
        Serial.println(datasize);

      }
    }
    

    else   {
      Serial.println("Invalid command");
    }  
}

// *****************************************************************************
