// GIP2 - Programmable Envelope Generator, Version 2
//
// a simple ADSR for the Arduino based on that brilliant piece of code by m0xpd, Feb 2017
//
// see http://m0xpd.blogspot.co.uk/2017/02/signal-processing-on-arduino.html
//
// Version 2 adds a true envelope invertion via op-amp (hardware mod)
//
// MODs to the original (m0xpd) code:
// - pin definitions are now compatible with 492X library
// - two envelope biased invertions modes: semi-inverted and quasi-inverted (with non-inverted sustain level)
// The code asks for a 3 positions switch (ON-OFF-ON) on input D3 and D4 to select between two envelope modes
// MODs to the first version sketch (progEnvGen_V1a/b.ino):
// three position switch to select biased envelope modes instead of non latching switch
//
// by Barito, sept 2021


#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW);}
#define pulseLow(pin) {digitalWrite(pin, LOW); digitalWrite(pin, HIGH);}

// Pin definitions...
const int gatePin = 8;
const int loopPin = 2;
const int mode1Pin = 3; //semi invertion, biased
const int mode2Pin = 4; //quasi invertion, biased
// MCP4921...
// (pin allocations for convenience in hardware hook-up)
const int DAC_CS = 10;
const int DAC_SCK = 13;
const int DAC_SDI = 11;
const int DAC_LDAC = 9;
byte upper_byte = 0x10;
byte lower_byte = 0;

float alpha=0.7;   // this is the pole location ('time constant') used for the first-order difference equation
double alpha1=0.9;  // initial value for attack
double alpha2=0.9;  // initial value for decay
double alpha3=0.95; // initial value for release

float envelope = 0.0;  // initialise the envelope
float CV0 = 0.0;       // result of reads from potentiometers (yes - it will only be an int, but helps with the casting!)
float CV1 = 0.0;
int CV2 = 0;
float CV3 = 0.0;

int drive = 0;
int sustain_Level = 0;
int scan = 0;
boolean note_active = false;
boolean loop_mode = false;
int invert_mode = 0; //0 - non inverted, 1 - semi inverted, 2 - quasi inverted
boolean trigger = false;
boolean decay = false;
boolean release_done = true;

// subroutine to set DAC on MCP4921
void Set_DAC_4921(int DC_Value){
    lower_byte=DC_Value&0xff;
    upper_byte=(DC_Value>>8)&0x0f;
    bitSet(upper_byte,4);
    bitSet(upper_byte,5);   
    digitalWrite(DAC_CS,LOW);
    tfr_byte(upper_byte);
    tfr_byte(lower_byte);
    digitalWrite(DAC_SDI,LOW);   
    digitalWrite(DAC_CS,HIGH);
    pulseLow(DAC_LDAC);
}
// transfers a byte, a bit at a time, LSB first to the DAC
void tfr_byte(byte data)
{
  for (int i=0; i<8; i++, data<<=1) {
    digitalWrite(DAC_SDI, data & 0x80);
    pulseHigh(DAC_SCK);   //after each bit sent, CLK is pulsed high
  }
}
void setup() {  
  pinMode(DAC_CS, OUTPUT);
  pinMode(DAC_SCK, OUTPUT);
  pinMode(DAC_SDI, OUTPUT);  
  pinMode(DAC_LDAC, OUTPUT);  
  pinMode(gatePin, INPUT); //This is served by a pulldown resistor
  pinMode(loopPin, INPUT_PULLUP);   
  pinMode(mode1Pin, INPUT_PULLUP);
  pinMode(mode2Pin, INPUT_PULLUP);  
  digitalWrite(DAC_CS,HIGH);  
  digitalWrite(DAC_LDAC,HIGH); 
  digitalWrite(DAC_SCK,LOW);
  digitalWrite(DAC_SDI,HIGH); 
  Set_DAC_4921(0); //initialize DAC
}

void loop() {
    boolean gate=digitalRead(gatePin);        // read the gate input every time through the loop
    update_params(scan);                      // scan only one of the other inputs each pass 
    
    boolean trigger = gate || (loop_mode && release_done);  // trigger an ADSR even if there's a gate OR if we're in loop mode
    while(trigger){  
      if(note_active==false){                     // if a note isn't active and we're triggered, then start one!
        decay = false;
        if(invert_mode == 0){                     // if the envelope is not inverted
          //envelope = 0;                           // envelope starts at zero
          drive = 4096;                           // and drives toward full value
        }
        else{                                     // else, if envelope is inverted, drive toward zero
          //envelope = 4096;                        // envelope starts at max
          drive = 0;                              // and drives toward zero value
        }
        alpha = alpha1;                           // set 'time constant' alpha1 for attack phase (set by the potentiometer position)
        note_active = true;                       // set the note_active flag
      }
      if(decay == false && ((envelope>4000 && drive==4096) || (envelope<96 && drive==0))){// if we are in attack phase and we've reached envelope >4000 with drive= 4096 (non invertion) or envelope < 96 with drive = 0 (invertion), we must be at the end of attack phase
                                                            // so switch to decay...
      decay = true;                                         // set decay flag
      drive = sustain_Level;                                // drive toward sustain level
      alpha = alpha2;                                       // and set 'time constant' alpha2 for decay phase
      }
    
      envelope = ((1.0-alpha)*drive+alpha*envelope);   // implement difference equation: y(k) = (1 - alpha) * x(k) + alpha * y(k-1)
      Set_DAC_4921(round(envelope));                   // and output the envelope to the DAC
      
      if(loop_mode == true && decay == true){          // in loop mode, break out at the end of the decay
        if(invert_mode == 0 && envelope < (float)(sustain_Level+1.0)){ 
          decay = false;
          break;
        }
        else if(invert_mode > 0 && envelope > (float)(sustain_Level-1.0)){ 
          decay = false;
          break;
        }
      }
      gate = digitalRead(gatePin);                      // read the gate pin (remember we're in the while loop)
      trigger = gate || (loop_mode && release_done);    // and re-evaluate the trigger function
    } //WHILE CLOSE
    
    if(note_active == true){                // this is the start of the release phase
      if(invert_mode < 2){
        drive=0;                            // drive towards zero
      }
      else {
        drive = 4095;                       //drive towards 5V
      }
    alpha = alpha3;                         // set 'time constant' alpha3 for release phase
    note_active = false;                    // turn off note_active flag
    release_done = false;                   // and set release_flag done false
  }
  
    envelope = ((1.0-alpha3)*drive+alpha3*envelope); // implement the difference equation again (outside the while loop)
    Set_DAC_4921(round(envelope));                   // and output envelope
    gate = digitalRead(gatePin);                     // watch out for a new note
    scan+=1;                                         // prepare to look at a new parameter input
    if((invert_mode < 2 && envelope < 4) || (invert_mode == 2 && envelope > 4091)){// is the release phase ended?
      release_done = true;                           // yes - so flag it
    }
    if(scan >= 6){                                   // increment the scan pointer
      scan = 0;
    }
}

void update_params(int scan){             // read the input parameters
switch (scan){
  case 0:
  CV0 = analogRead(0);                    // get the attack pole location
  alpha1 = 0.999*cos((1023-CV0)/795);
  alpha1 = sqrt(alpha1);  
  break;
  case 1:
  CV1 = analogRead(1);                    // get the release pole location
  alpha2 = 0.999*cos((1023-CV1)/795);
  alpha2 = sqrt(alpha2);   
  break; 
  case 2:
  CV2 = analogRead(2);                    // get the (integer) sustain level
  sustain_Level = CV2<<2;
  break;
  case 3:
  CV3 = analogRead(3);                    // get the release pole location (potentially closer to 1.0)
  alpha3 = 0.99999*cos((1023-CV3)/795);
  alpha3 = sqrt(alpha3);
  alpha3 = sqrt(alpha3);                  //YES: twice sqrt
  break;  
  case 4:                                 // read the loop mode input
  loop_mode = !digitalRead(loopPin);
  break;  
  case 5:                                 // read the envelope invertion mode 3 positions switch
  if(digitalRead(mode1Pin) == LOW){
    invert_mode = 1;
  }
  else if (digitalRead(mode2Pin) == LOW){
    invert_mode = 2;
  }
  else{
    invert_mode = 0;
  }
  break;
}
}
