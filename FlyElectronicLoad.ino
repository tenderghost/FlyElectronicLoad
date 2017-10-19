/**
 * FlyElectronicLoad by tenderghost@gmail.com
 * Using arduino pro mini
 * Using https://www.arduino.cc/en/Tutorial/Debounce for pushbutton to ignore noise
 * TODO 1 add auto charge function!
 * TODO 2 add internal resistence test funciton!
 * TODO 3 add discharge curve report furnction!
 */

// Pin definations
uint8_t BAT_VOLTAGE_PIN = A0; // Monitoring battery voltage
uint8_t MOS_PWM_PIN = 9; // control battery discharge speed
uint8_t CUR_VOLTAGE_PIN = A1; // PIN for sensoring battery voltage drop to get voltage, using 20mR sampling resistor.
uint8_t PUSH_BTN_PIN = 8; // PIN for push button

float rSensor = 0.056; // 50mR current sense resistor

// Global dischage pwm speed for battery dischage, 0-255
int dischagePWMSpeed = 0;

// Global variables for store battery voltage and current.
float batVoltage = 0.0;
float batDischageCurrent = 0.0;

// Record the battery capacity, in 
float batteryCapacity = 0.0;

// Debounce begin
// Variables will change:
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
// Debounce end

// 记录采样时间，间隔1秒采样，避免采样频率过高，影响按钮控制
unsigned long lastSampleTime = 0;
unsigned long sampleDelay = 1000; // in milliseconds

const float LiChargedVtg = 4.2;
const float LiCutoffVtg = 3.0;

const unsigned int VOLT_SAMPLE_COUNT = 500; // Voltage or Current sample times per loop

// Electronic load running state constants
const char STATE_WATING = 'w';    // Waiting for command
const char STATE_RUNNING = 'r';   // Running state
const char STATE_DISCHARGE_COMPLETE = 'c'; // Discharge complete
char currentState = STATE_WATING;

void setup() {
  // put your setup code here, to run once:
  pinMode(PUSH_BTN_PIN, INPUT);
  pinMode(MOS_PWM_PIN, OUTPUT);
  //pinMode(BAT_VOLTAGE_PIN, INPUT);
  //pinMode(CUR_VOLTAGE_PIN, INPUT);

  Serial.begin(115200);
}

void loop() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(PUSH_BTN_PIN);
  
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // button state change to LOW means the button was pushed.
      if (buttonState == LOW) {
        Serial.println("Push button pushed.");

        // Change current Arduino state
        if (currentState == STATE_WATING) {
          currentState = STATE_RUNNING;
        } else {  // discharging
          currentState = STATE_WATING;
        }

      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;

  // ==========================================
  // DISCHAGE OR STOP DISCHAGE CODE START HERE!
  if (currentState == STATE_RUNNING) {
    constantCurrent(1.0); // TEST CODE FOR CONSTANT CURRENT MODE
  } else {
    setPMWSpeed(0);
  }

  // Print voltage and current
  if ((millis() - lastSampleTime) > sampleDelay) {
    lastSampleTime = millis();

    // Read battery voltage and dischage current, values store to batVoltage and batDischageCurrent variables.
    readVoltAndCurrent();

    Serial.print("Battery voltage:");
    Serial.println(batVoltage);
    Serial.print("Dischage current:");
    Serial.println(batDischageCurrent);
    
    // Get battery capacity!
    if (currentState == STATE_RUNNING) {
      batteryCapacity += (batDischageCurrent * sampleDelay) / 3600.0;
      Serial.print("Battery capacity:");
      Serial.println(batteryCapacity);
    }

    Serial.println("===============");

    // Check if the battery was dischage complete
    if (batVoltage < LiCutoffVtg) {
      stopDischarge();
    }
  }
}

// change dischage current to desire ampire
// the desire ampire should less or bigger than 0.1
void constantCurrent(float desireAmpire) {
  readVoltAndCurrent();

  if (batDischageCurrent < desireAmpire - 0.05) {
    int newPMWSpeed = dischagePWMSpeed + 1;
    if (newPMWSpeed > 255) {
      newPMWSpeed = 255; // should NOT bigger than 255
    }
    setPMWSpeed(newPMWSpeed);
  } else if (batDischageCurrent > desireAmpire + 0.05) {
    int newPMWSpeed = dischagePWMSpeed - 1;
    if (newPMWSpeed < 0) {
      newPMWSpeed = 0; // should NOT less than zero
    }
    setPMWSpeed(newPMWSpeed);
  } else {
    /*
    Serial.print("discharge current is: ");
    Serial.print(batDischageCurrent);
    Serial.println(", and is OK!");
    */
  }
}

void stopDischarge() {
  setPMWSpeed(0);
  currentState = STATE_DISCHARGE_COMPLETE;
  Serial.println("Discharge complete!!!");
}


/**
   Read voltage and current from battery.
   batVoltage - battery voltage
   batDischageCurrent - battery dischage current.
*/
void readVoltAndCurrent() {
    // Calculate battery voltage.
    unsigned long totalVoltage = 0;
    for (int i = 0; i < VOLT_SAMPLE_COUNT; i++) {
      totalVoltage += analogRead(BAT_VOLTAGE_PIN);
    }
    float fltVoltPinValue = totalVoltage / VOLT_SAMPLE_COUNT;
    batVoltage = 2.0 * 5.0 * (fltVoltPinValue / 1023.0f);
  
//    Serial.print("Battery voltage:");
//    Serial.println(batVoltage);
  
    // get sampling resistor voltage drop to calculate current.
    totalVoltage = 0;
    for (int i = 0; i < VOLT_SAMPLE_COUNT; i++) {
      totalVoltage += analogRead(CUR_VOLTAGE_PIN);
    }
  
    float devidedVoltageDrop = 1.0 * totalVoltage / VOLT_SAMPLE_COUNT;
    //float voltageDrop = 5.0 * samplerResVoltDrop / 1023.0;
//    Serial.print("Voltage drop:");
//    Serial.println(devidedVoltageDrop);
  
    batDischageCurrent = (5.0 * devidedVoltageDrop) / (1023.0 * rSensor);
//    Serial.print("Dischage Current:");
//    Serial.println(batDischageCurrent);
}

void setPMWSpeed(int speed) {
  if (speed > 255) {
    speed = 255; // set maxmium speed for 255
  }

  if (dischagePWMSpeed != speed) {
    analogWrite(MOS_PWM_PIN, speed);
    Serial.print("Current PWM is ");
    Serial.println(speed);
    dischagePWMSpeed = speed;
  }
}

long readVcc() { // from https://code.google.com/p/tinkerit/wiki/SecretVoltmeter
  int result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // Read 1.1V reference against AVcc
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  result = ADCL;
  result |= ADCH << 8;
  //Original code below
  //result = 1125300L/ result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  //Calibrated code below
  result = 1106706L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000*(4940/5023)
  return result; // Vcc in millivolts
}