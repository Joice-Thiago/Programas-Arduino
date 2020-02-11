#include <Wire.h>
#define AMOSTRAS 12 //Quantidade de Amostras para utilizadas para média tensão AC

int sensorPin1 = A0; // Tensão Vgc
int sensorPin2 = A1; // Tensão Vcb
int sensorPin3 = A2; // Tensão Vci
int sensorPin4 = A3; // Tensão Vin
int sensorValue1 = 0; //Valor de Temperatura em Bits
int sensorValue2 = 0; //Valor de Irradiância em Bits
int sensorValue3 = 0; //Valor de Corrente AC em Bits
int sensorValue4 = 0; // Valor de Tensão AC em Bits

//==============Declaração Variaveis Tensão AC=====================
float tensaoAC; //Declaração tensão AC
float aRef = 5; // Referência para Arduino
int relacaoA3 = 69.255180; //Relação para o Sensor

//
void setup()
{
  Wire.begin(8);
  //Serial.begin(9600);
  Wire.onRequest(requestEvent);
}

//===================Configuração Tensão AC ========================

float lePorta(uint8_t portaAnalogica) {
  float total = 0;
  for (int i = 0; i < AMOSTRAS; i++) {
    total += 1.0 * analogRead(portaAnalogica);
    delay(5);
  }
  return total / (float)AMOSTRAS;
}
//===================Configuração Corrente AC ======================
void loop() {
  tensaoAC = lePorta(A3);
  sensorValue4 = (int)tensaoAC;

}

void requestEvent()
{
  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);
  sensorValue3 = analogRead(sensorPin3);
  sensorValue4 = analogRead(sensorPin4);


  char v1[4];
  char v2[4];
  char v3[4];
  char v4[4];

  /*float tp = (float)millis() / 1000 * 0.5;
  float ft = 3.1415 * 0.16666666666;*/


  IntToCharArray(v1, sensorValue1); //(int)(400 * cos(tp + ft * 1) + 512));
  IntToCharArray(v2, sensorValue2); //(int)(400 * cos(tp + ft * 2) + 512));
  IntToCharArray(v3, sensorValue3); //(int)(400 * cos(tp + ft * 3) + 512));
  IntToCharArray(v4, sensorValue4); //(int)(400 * cos(tp + ft * 4) + 512));


  Wire.write(v1);
  Wire.write(v2);
  Wire.write(v3);
  Wire.write(v4);
}


void IntToCharArray(char (&varDeclarada)[4], int variavelMedida) {
  String var = (String)variavelMedida;
  String saida = "";

  if (variavelMedida >= 0 and variavelMedida < 10) {
    saida = "000" + var;
  } else if (variavelMedida >= 10 and variavelMedida < 100) {
    saida = "00" + var;
  } else if (variavelMedida >= 100 and variavelMedida < 1000) {
    saida = "0" + var;
  } else if (variavelMedida >= 1000 and variavelMedida < 1024) {
    saida = var;
  } else if (variavelMedida > 1023 or variavelMedida < 0)
    saida = "0000";

  for (int x = 0; x < 4; x++) {
    varDeclarada[x] = saida.charAt(x);
  }
}
