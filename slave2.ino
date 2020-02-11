#include <Wire.h>

int sensorPin1 = A0; // Pino Temperatura
int sensorPin2 = A1; // pino Irradiância

int sensorValue1 = 0; //Valor de Temperatura em Bits
int sensorValue2 = 0; //Valor de Irradiância em Bits

void setup()
{
  Wire.begin(9);
  Wire.onRequest(requestEvent);
}

void loop() {

}

void requestEvent()
{
  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);

  char v1[4];
  char v2[4];

  /*float tp = (float)millis() / 1000 * 0.5;
  float ft = 3.1415 * 0.16666666666;*/

  IntToCharArray(v1, sensorValue1); //(int)(300 * cos(tp + ft * 1) + 512));
  IntToCharArray(v2, sensorValue2); //(int)(300 * cos(tp + ft * 1) + 512));

  Wire.write(v1);
  Wire.write(v2);
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
