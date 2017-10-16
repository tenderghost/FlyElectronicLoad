/**
 * FlyElectronicLoad by tenderghost@gmail.com
 * Using arduino pro mini
 */

// Pin definations
uint8_t BAT_VOLTAGE_PIN = A0; // Monitoring battery voltage
uint8_t MOS_PWM_PIN = 9; // control battery discharge speed
uint8_t CUR_VOLTAGE_PIN = A1; // PIN for sensoring battery voltage drop to get voltage, using 20mR sampling resistor.
uint8_t PUSH_BTN_PIN = 8; // PIN for push button

float rSensor = 0.044; // 50mR current sense resistor

// Global dischage pwm speed for battery dischage, 0-255
int dischagePWMSpeed = 0;

// Global variables for store battery voltage and current.
float batVoltage = 0.0;
float batDischageCurrent = 0.0;

const float LiChargedVtg = 4.2;
const float LiCutoffVtg = 3.0;
const unsigned int VOLT_SAMPLE_COUNT = 1000; // Voltage or Current sample times per loop

// Electronic load running state constants
const unsigned int STATE_WATING = 81;    // Waiting for command
const unsigned int STATE_RUNNING = 82;   // Running state
unsigned int currentState = STATE_WATING;

void setup() {
  // put your setup code here, to run once:
  pinMode(MOS_PWM_PIN, OUTPUT);
  //pinMode(BAT_VOLTAGE_PIN, INPUT);
  //pinMode(CUR_VOLTAGE_PIN, INPUT);

  Serial.begin(115200);
}

void loop() {
  int val = digitalRead(PUSH_BTN_PIN);  // read input value
  if (val == LOW) {         // check if the input is LOW (button pushed)
    Serial.println("Push button pushed.");
  }
}

void loop_old() {
  // Read battery voltage and dischage current, read them from batVoltage and batDischageCurrent variables.
  readVoltAndCurrent();

  // test code
  // set dischage current to 0.5A
  if (batDischageCurrent < 0.5) {
    Serial.println("Test code for change current under 0.5A");
    int newPMWSpeed = dischagePWMSpeed + 1;
    setPMWSpeed(newPMWSpeed);
  }

  Serial.println("===============");

  delay(100);
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
    float batVoltage = 2.0 * 5.0 * (fltVoltPinValue / 1023.0f);
  
    Serial.print("Battery voltage:");
    Serial.println(batVoltage);
  
    // get sampling resistor voltage drop to calculate current.
    totalVoltage = 0;
    for (int i = 0; i < VOLT_SAMPLE_COUNT; i++) {
      totalVoltage += analogRead(CUR_VOLTAGE_PIN);
    }
  
    float devidedVoltageDrop = 1.0 * totalVoltage / VOLT_SAMPLE_COUNT;
    //float voltageDrop = 5.0 * samplerResVoltDrop / 1023.0;
    Serial.print("Voltage drop:");
    Serial.println(devidedVoltageDrop);
  
    batDischageCurrent = (5.0 * devidedVoltageDrop) / (1023.0 * rSensor);
    Serial.print("Dischage Current:");
    Serial.println(batDischageCurrent);
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