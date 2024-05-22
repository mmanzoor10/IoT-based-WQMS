int trigPin = 9;    // Trigger
int echoPin = 8;    // Echo
long duration, cm, inches;

String values;
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
float   temperatures ;
/******************************* tds sensor *************************************/
#define TdsSensorPin A2
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30         // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

/*********************************** ph sensor ********************************/
const int analogInPin = A1 ;
int sensorValue = 0;
unsigned long int avgValue;
float b;
int buf[10], temp;
float phValue;

float pph;

/********************************** Turbidity sensor ****************************************/
int Turbidity_Sensor_Pin = A0;
float Turbidity_Sensor_Voltage;
int samples = 600;
float ntu; // Nephelometric Turbidity Units
int count = 0;
/************************************* Uart *****************************************/

void setup()
{
  Serial.begin(9600);

  pinMode(TdsSensorPin, INPUT);

  sensors.begin();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}
float ph (float voltage) {
  return 7 + ((2.5 - voltage) / 0.18);
}

float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}

void loop()
{ /*************** ntu ********************/
 Turbidity_Sensor_Voltage = 0;
  for (int i = 0; i < samples; i++)
  {
    Turbidity_Sensor_Voltage += ((float)analogRead(Turbidity_Sensor_Pin) / 1023) * 5;
  }

  Turbidity_Sensor_Voltage = Turbidity_Sensor_Voltage / samples;
  Turbidity_Sensor_Voltage = round_to_dp(Turbidity_Sensor_Voltage, 2);
  if (Turbidity_Sensor_Voltage < 2.5) {
    ntu = 3000;
  } else {
    ntu = -1120.4 * square(Turbidity_Sensor_Voltage) + 5742.3 * Turbidity_Sensor_Voltage - 4352.9;
  }

  delay(10);
  /*************************************/
  sensors.requestTemperatures();
  temperatures = (sensors.getTempCByIndex(0));
  // Serial.print("temperatures:");
  // Serial.print(sensors.getTempCByIndex(0));
  // Serial.print("c");

  /****************** ph ***********************/
  for (int i = 0; i < 10; i++)
  {
    buf[i] = analogRead(analogInPin);
    delay(10);
  }
  for (int i = 0; i < 9; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buf[i] > buf[j])
      {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 2; i < 6; i++)
    avgValue += buf[i];
  float pHVol = (float)avgValue * 5.0 / 1024 / 6;
  float phValue = -5.70 * pHVol + 23.34;
  pph = phValue;
  //Serial.print("pH = ");
  //Serial.print(phValue);

  tds(); //Serial.println(" ");
   DIS();
 values =  String(pph) + ',' + String(ntu) + ',' + String(temperatures) + ',' + String(tdsValue)+ ',' + String(inches) ;
  Serial.println(values);
delay(1000);


}

void tds()
{
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    // Serial.print("TDS Value:");
    // Serial.print(tdsValue, 0);
    // Serial.print("ppm");

  }
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}


 
void DIS() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
  

  
  delay(250);
}
