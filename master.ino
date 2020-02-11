//==============Incluindo biblioteca =========================
#include <Wire.h> //Inclusão de Biblioteca

#include <SimpleModbusSlave.h>

//=========== Incluindo configuração Arduino ====================
//==============Declaração Variáveis Corrente AC ==================
#include "EmonLib.h" //Inclusão de Biblioteca
#define CURRENT_CAL 0.1101 // Valor de calibração (deve ser ajustado em paralelo com multimetro medindo a corrente da carga)
EnergyMonitor emon1; //Cria uma instância

enum
{
  Igc,
  Icb,
  Ici,
  Iin,
  Vgc,
  Vcb,
  Vci,
  Vin,
  temp,
  irrad,
  HOLDING_REGS_SIZE
};
unsigned int holdingRegs[HOLDING_REGS_SIZE];


int currentAnalogInputPin = 3;          // Analog input pin reading the current sensor
const int sensitivity = 66;               // mV/A, 185 for ASC714 -5A +5A, 66 for ASC714 -30A +30A
const int hz = 60;                         // frequency of power line
const int voltage = 127;               // Voltage of the current being measured
const int rawACTuning = 21;               // fine tuning of 0A AC reading! 512 = 0V


void setup()
{

  Wire.begin();
  modbus_configure(&Serial, 9600, SERIAL_8N1, 1, 2, HOLDING_REGS_SIZE, holdingRegs);
  modbus_update_comms(9600, SERIAL_8N1, 1);
  //Serial.begin(9600);
  emon1.current(A3, CURRENT_CAL);
}
void loop()
{

  //Igc, Icb, Ici, Iin
  //Vgc, Vcb, Vci, Vin
  //temp, irrad

  int mIgc; //A0
  int mIcb; //A1
  int mIci; //A2
  int mIin; //A3

  mIgc = analogRead(A0);
  mIcb = analogRead(A1);
  mIci = analogRead(A2);
  mIin = (int)(1000 * readCurrent(false));

  int e1_Vgc; //e1_A0
  int e1_Vcb; //e1_A1
  int e1_Vci; //e1_A2
  int e1_Vin; //e1_A3
  escravo1(e1_Vgc, e1_Vcb, e1_Vci, e1_Vin, 8, 16);

  int e2_temp;  //e2_A0
  int e2_irrad; //e2_A1
  escravo2(e2_temp, e2_irrad, 9, 8);

  /*float tp = (float)millis() / 1000 * 0.5;
    float ft = 3.1415 * 0.16666666666;*/

  holdingRegs [Igc] = mIgc; //(int)(512 * cos(tp + ft * 1) + 512);
  holdingRegs [Icb] = mIcb; //(int)(512 * cos(tp + ft * 2) + 512);
  holdingRegs [Ici] = mIci; //(int)(512 * cos(tp + ft * 3) + 512);
  holdingRegs [Iin] = mIin; //(int)(512 * cos(tp + ft * 4) + 512);

  holdingRegs [Vgc] = e1_Vgc;
  holdingRegs [Vcb] = e1_Vcb;
  holdingRegs [Vci] = e1_Vci;
  holdingRegs [Vin] = e1_Vin;

  holdingRegs [ temp] = e2_temp;
  holdingRegs [irrad] = e2_irrad;

  /*Serial.println();
    Serial.println(holdingRegs [Igc]);
    Serial.println(holdingRegs [Icb]);
    Serial.println(holdingRegs [Ici]);
    Serial.println(holdingRegs [Iin]);*/
  //Serial.println(Iin);

  modbus_update();




}

float media(int n, int pino, float constante) {
  float saida = 0;
  for (int i = 0; i < n; i++) {
    saida += (float)analogRead(pino);
  }
  return (saida / n) * constante;
}

void escravo1(int &vgc, int &vcb, int &vci, int &vin, byte dispEndereco, byte numBytes) {
  String todasVariaveis = "";

  Wire.requestFrom(dispEndereco, numBytes);

  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    todasVariaveis += c;
  }

  vgc = (todasVariaveis.substring(0, 4)).toInt();
  vcb = (todasVariaveis.substring(4, 8)).toInt();
  vci = (todasVariaveis.substring(8, 12)).toInt();
  vin = (todasVariaveis.substring(12, 16)).toInt();
}

void escravo2(int &temperatura, int &irradiancia, byte dispEndereco, byte numBytes) {
  String todasVariaveis = "";

  Wire.requestFrom(dispEndereco, numBytes);

  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    todasVariaveis += c;
  }

  temperatura = (todasVariaveis.substring(0, 4)).toInt();
  irradiancia = (todasVariaveis.substring(4, 8)).toInt();

}

// Calculate current read with Allegro ACS714
// print: true to print measurement info
// Returns the sensed current in A
double readCurrent(bool print)
{
  //Short version: mA = ((analogRead * 5000 / 1024) - 2500 ) / 66
  //Short version: mA = ((analogReadAmplitude/2) * 5000 / 1024) / 66
  int analogReadAmplitude = 0, min = 512, max = 0, filter = 4;
  unsigned long start = millis();
  do {
    int val = 0;
    for (int i = 0; i < filter; i++)
      val += analogRead(currentAnalogInputPin);
    val = (val / filter);     // fine tuning of 0A AC reading! 512 = 0V
    if (max < val) max = val;
    if (val < min) min = val;
  } while (millis() - start <= 1100 / hz);   //10% + to ensure p2p is acquired
  analogReadAmplitude = (max - min) / 2;
  long internalVcc = readInternalVcc();                         // should be around 5000
  double sensedVoltage = (analogReadAmplitude * internalVcc) / 1024;     // (analogReadAmplitude/2) * 5000 / 1024               -> 0: 0               1024: 5000
  double sensedCurrent = sensedVoltage / sensitivity;          // ((analogReadAmplitude/2) * 5000 / 1024) / 66     -> 0: -37,9A     1024: 37,9A
  if (print) {
    Serial.print("internalVcc mV: ");
    Serial.print(internalVcc);
    Serial.print(", AnalogIn: ");
    Serial.print(analogReadAmplitude);
    Serial.print(", Sensed Voltage mV: ");
    Serial.print(doubleToStr(sensedVoltage, 1));
    Serial.print(", VariationFromNull A: ");
    Serial.print(doubleToStr(sensedCurrent, 2));       // display Current
    Serial.print(", power W: ");
    Serial.println(doubleToStr(currentToPower(sensedCurrent), 2));
  }
  return sensedCurrent;
}
// Read 1.1V reference against AVcc
// Returns a voltage in mV that will be close to 5000 for the Arduino Yún
long readInternalVcc()
{
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0) ;
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
// Returns power in Watt (W)
// Watt = Ampere * Volt
double currentToPower(double current) {
  return current * voltage;
}
// Converts a double to a string
String doubleToStr(double val, byte precision)
{
  String out = String(int(val));                   // Print int part
  if ( precision > 0) {                    // Print decimal part
    out += ".";
    unsigned long frac, mult = 1;
    byte padding = precision - 1;
    while (precision--) mult *= 10;
    if (val >= 0) frac = (val - int(val)) * mult; else frac = (int(val) - val) * mult;
    unsigned long frac1 = frac;
    while (frac1 /= 10) padding--;
    while (padding--) out += "0";
    out += String(frac, DEC) ;
  }
  return out;
}
