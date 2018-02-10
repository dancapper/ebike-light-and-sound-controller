// E-Bike Lighting Controller
//
// github.com/dancapper/ebike-light-and-sound-controller
//
// license: use however you like

#define versionMajor       1
#define versionMinor       02

// Hardware configuration

// I used an Arduino Pro Mini clone, so you may prefer different pinouts

#define debug           false  // Debug over serial, recommend false unless debugging for better performance

// LDR circuit - LDR to Gnd, 47k to VCC

#define sensorPin         A0  // select the input pin for the LDR

// drivers - I used opto isolated MOSFET switch modules, high is "on"

#define hlPin             10  // select the pin for the headlight driver
#define rearPin           11  // pin for rear light

// Button functions:
// <1s = Auto / Headlight On toggle
// >1s <5s = Lights off completely
// >5s = play music

#define buttonPin          2  // input button, switch to ground, multifunction

// Horn pin activates a sound of your choosing

#define hornPin            3  // horn button, switch to ground

// LED indicates current mode; flash for flash, on for headlight, off for no lights

#define ledPin             6  // indicator led

// Sound pins - using a 1k resistor to input pins on a BY8301 module

#define soundPinStart      5  // startup sound pin
#define soundPinMusic      4  // your choice of music for effect
#define soundPinBell       7  // bell sound

// Default Values todo: make configurable on the fly

boolean flashFrontLight = true;   // Flash front light in daylight?
int rearFlashRate = 400;          // msec timer for rear light, also used for front light
int frontFlashBrightness = 128;   // Brightness of front light when flashing
int rearLowBrightness = 128;      // "off" brightness of rear light when headlight is on
int ledBrightness = 80;           // Brightness of indicator LED in darkness
int threshold = 150;              // what LDR threshold to turn on lighting
int stickiness = 20;              // how far past threshold to change (provides hysteresis)
int sampleDelay = 300;            // msec between samples
int samplesToUse = 5;             // how many samples to debounce light sensor

// Internal Variables do not alter

int sensorValue = 0;   // variable to store the value coming from the sensor
boolean lastSample;    // used to debounce light sensor
boolean currentState;  // used to debounce light sensor
int sampleCount = 0;   // used to debounce light sensor
boolean frontLightState = false; // used for flashing
boolean rearLightState = false; 
boolean autoLight = true; 
boolean power = true;
int ticks = 0;
int buttonticks = 0;
int soundticks = 0;
int sensorticks = 0;
int soundPlaying = 0;
int loopstart = 0;
int looptook = 0;

void setup() {
  delay(500);              // just a little warmup time, to allow flashing if there's an issue with interrupts etc

  if(debug){
            Serial.begin(57600);
            Serial.println("Ebike Lighting Controller - version " + String(versionMajor) + "." + String(versionMinor)); 
         }

  pinMode(hlPin, OUTPUT);
  pinMode(rearPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(hornPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin), ButtonPress, FALLING);
  attachInterrupt(digitalPinToInterrupt(hornPin), HornPress, FALLING);
  
  OCR0A = 0xAF;            // use the same timer as the millis() function
  TIMSK0 |= _BV(OCIE0A);

  // Play a startup sound
  
  PlaySound(soundPinStart);
}

void loop() {
      // Since all my functions are time critical, I'm using the timer function instead
}

ISR(TIMER0_COMPA_vect){
  if(debug) 
    loopstart = millis();
  
  ticks++;
  buttonticks++;
  soundticks++; // used to pulse sound output
  sensorticks++;

  if(ticks == rearFlashRate){
    ticks = 0;
    if(rearLightState) {
      if(frontLightState && power) {
        RL(rearLowBrightness);
      } else {
        RL(false);
        LED(false);
      }
      if(flashFrontLight &! frontLightState)
        HL(false, false);

    } else if (power) {
      RL(true);
      LED(true);
      if(flashFrontLight &! frontLightState)
        HL(frontFlashBrightness);
    }
  }
  
  if(soundticks == 100) {
    StopSounds();
  }
  
  if(sensorticks == sampleDelay) {
      sensorticks = 0;
      if(autoLight) {
        // read the value from the sensor:
        sensorValue = analogRead(sensorPin);
        if(debug) 
          Serial.println("Sensor Value: " + String(sensorValue));
        if(sensorValue < (threshold - stickiness)) {
          if(lastSample == LOW) { // two consecutive samples
            if(sampleCount == samplesToUse) {
              HL(false, true);  // headlight off
              sampleCount++;
            } else {
              sampleCount++;
            }
          } else {
            lastSample = LOW;
            sampleCount = 0;
          }
      } else if(sensorValue > (threshold + stickiness)) {
        if(lastSample == HIGH) {
          if(sampleCount == samplesToUse) {
            HL(true, true);  // headlight on
            sampleCount++;
          } else {
            sampleCount++;
          }
        } else {
          lastSample = HIGH;
          sampleCount = 0;
        }
      }  
    }
  }
  if(debug) {
    looptook = millis() - loopstart;
    if(looptook)
      Serial.println("Loop took " + String(millis() - loopstart) + "ms");
  }
}

void HornPress() {
  if (digitalRead(hornPin) == LOW)
    PlaySound(soundPinBell);
}

void ButtonPress() {
  if(debug) 
    Serial.println("button pressed");
  if(digitalRead(buttonPin) == LOW) {
    buttonticks = 0;
    attachInterrupt(digitalPinToInterrupt(buttonPin), ButtonRelease, RISING);
  }
}

void ButtonRelease() { // button released
  if(debug) 
    Serial.println("button released duration:" + String(buttonticks));
  attachInterrupt(digitalPinToInterrupt(buttonPin), ButtonPress, FALLING);
  if (buttonticks > 80 && buttonticks < 1000) {
    if(autoLight) {
      autoLight = false;
      HL(true, true);
      power = true;
    } else {
      autoLight = true;
      sampleCount = 0;
      power = true;
    }
  } else if (buttonticks >= 1000 && buttonticks <= 5000) {
    autoLight = false;
    HL(false, true);
    power = false;
  } else if (buttonticks >= 5000) {
    PlaySound(soundPinMusic);
  }
  buttonticks = 0;
}

void PlaySound(int soundPin) {
  soundticks = 0;
  StopSounds();
  soundPlaying = soundPin;
  if(debug)
    Serial.println("Sound Playing on pin: " + String(soundPin));
  pinMode(soundPin, OUTPUT);
  digitalWrite(soundPin, LOW);
}

void StopSounds() {
  if(soundPlaying) {
    if(debug)
      Serial.println("Stopping Sound on pin: " + String(soundPlaying));
    pinMode(soundPlaying, INPUT);
    soundPlaying = 0;
  }
}

void LED(boolean ledState) {
  if(ledState) {
    if(frontLightState)
      analogWrite(ledPin, ledBrightness);
    else
      digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

void HL(boolean hlState, boolean set) {
  if(hlState) {
    if(debug)
      Serial.println("Headlight On");
    if(set) {
      frontLightState = true;
      LED(true);
    }
    digitalWrite(hlPin, HIGH);
  } else {
    if(debug)
      Serial.println("Headlight Off");
    if(set) {
      frontLightState = false;
      LED(false);
    }
    digitalWrite(hlPin, LOW); // lights out
  }
}

void HL(int hlState) {
  if(debug)
    Serial.println("Headlight set to " + String(hlState));
  analogWrite(hlPin, hlState);
}
   
void RL(boolean rlState) {
  if(debug)
    Serial.println("Rear light is " + String(rlState));
  digitalWrite(rearPin, rlState);
  rearLightState = rlState;
}

void RL(int rlState) {
  if(debug)
    Serial.println("Rear Light set to " + String(rlState));
  analogWrite(rearPin, rlState);
  rearLightState = false;
}
  
