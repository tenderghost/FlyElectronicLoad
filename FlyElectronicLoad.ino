/**
 * FlyElectronicLoad by tenderghost@gmail.com
 * Using arduino pro mini
 */

// Pin definations
uint8_t BAT_VOLTAGE_PIN = A0; // Monitoring battery voltage
uint8_t MOS_PWM_PIN = 9; // control battery discharge speed
uint8_t CUR_VOLTAGE_PIN = A1; // PIN for sensoring battery voltage drop to get voltage, using 20mR sampling resistor.

float rSensor = 0.044; // 50mR current sense resistor



int pwmSpeed = 0;

// Global variables for store battery voltage and current.
float batVoltage = 0.0;
float batDischageCurrent = 0.0;

unsigned long totalVoltage = 0;

const float LiChargedVtg = 4.2;
const float LiCutoffVtg = 3.0;
const unsigned int CUR_SAMPLE_COUNT = 1000; // 电流采样1000次, Sample times per loop

void setup() {
  // put your setup code here, to run once:
  pinMode(MOS_PWM_PIN, OUTPUT);
  //pinMode(BAT_VOLTAGE_PIN, INPUT);
  //pinMode(CUR_VOLTAGE_PIN, INPUT);

  setPMWSpeed(155); // TODO remove it!

  Serial.begin(115200);
}

void loop() {
  // Calculate battery voltage.
  totalVoltage = 0;
  for (int i = 0; i < CUR_SAMPLE_COUNT; i++) {
    totalVoltage += analogRead(BAT_VOLTAGE_PIN);
  }
  float fltVoltPinValue = totalVoltage / CUR_SAMPLE_COUNT;
  float batVoltage = 2.0 * 5.0 * (fltVoltPinValue / 1023.0f);

  Serial.print("Battery voltage:");
  Serial.println(batVoltage);

  // get sampling resistor voltage drop to calculate current.
  totalVoltage = 0;
  for (int i = 0; i < CUR_SAMPLE_COUNT; i++) {
    totalVoltage += analogRead(CUR_VOLTAGE_PIN);
  }

  float devidedVoltageDrop = 1.0 * totalVoltage / CUR_SAMPLE_COUNT;
  //float voltageDrop = 5.0 * samplerResVoltDrop / 1023.0;
  Serial.print("Voltage drop:");
  Serial.println(devidedVoltageDrop);

  float current = (5.0 * devidedVoltageDrop) / (1023.0 * rSensor);
  Serial.print("Current:");
  Serial.println(current);

  // 将电流控制在0.5A以内
  if (current > 0.5) {
    Serial.println("Test code for change current under 0.5A");
    int newPMWSpeed = pwmSpeed - 5;
    setPMWSpeed(newPMWSpeed);
  }

  Serial.println("===============");

  delay(1000);
}

/**
   Read voltage and current from battery.
   batVoltage - battery voltage
   batDischageCurrent - battery dischage current.
*/
void readVoltAndCurrent() {

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

void setPMWSpeed(int speed) {
  if (pwmSpeed != speed) {
    analogWrite(MOS_PWM_PIN, speed);
    Serial.print("Current PWM is ");
    Serial.println(speed);
    pwmSpeed = speed;
  }
}

