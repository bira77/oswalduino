#include <LiquidCrystal.h>//Inclui a biblioteca do LCD   //ALTERACOES BIRA
// constants for temperature calculation
#define TEMP_R      9870.0      // Fixed resistance in the voltage divider
#define TEMP_C1     1.009249522e-03
#define TEMP_C2     2.378405444e-04
#define TEMP_C3     2.019202697e-07
#define BUTTON_PIN  31   // the number of the pushbutton pin

/*
#define PIN_LED_VERMELHO    37
#define PIN_LED_VERDE       39
#define PIN_LED_AMARELO     41
*/

#define PIN_BUZZER_MOTORTEMP     9 // MOTOR TEMPERATURE
#define PIN_BUZZER_WATERLEVEL    10 // VASE EXPANSION

// Analog input pin for thermistor voltage
#define PIN_THERMISTOR_0     A1
#define PIN_THERMISTOR_1   A2     //ALTERACOES BIRA

// TEMPERATURE LIMITS CONFIGURATION
#define HIGH_TEMPERATURE    30 // VALUE OF HIGH TEMP
#define MID_TEMPERATURE     28 // VALUE OF MID TEMP

// variables will change:
int buttonState=0;   // variable for reading the pushbutton status
LiquidCrystal lcd(12, 11, 5, 4, 3, 2); //Configura os pinos do Arduino para se comunicar com o LCD      //ALTERACOES BIRA
/********************************************************************************************************
    DECLARAÇÃO DAS FUNÇÕES
*********************************************************************************************************/
void setRGBNIVELColor (int color);
void setRGBTEMPLColor (int color);

/********************************************************************************************************
    TEMPERATURE READING
*********************************************************************************************************/

int temperatureSensor0, temperatureSensor1;     // Temperatura        //ALTERACOES BIRA
void measureTemperature()
{
    // MEASURE TEMPERATURE FROM SENSOR 0
    int Vo, Vo1; // Integer value of voltage reading
    float logRt, Rt, logRt1, Rt1;

    //Medição e cálculo do sensor 0
    Vo = analogRead(PIN_THERMISTOR_0);
    Rt = TEMP_R * (1023.0 / (float)Vo - 1.0);
    logRt = log(Rt);
    temperatureSensor0 = (int) ((1.0 / (TEMP_C1 + TEMP_C2*logRt + TEMP_C3*logRt*logRt*logRt)) - 273.15);
    
    
    //Serial.print(" ");
    //Serial.print(Vo);
    //Serial.print(" ");
    //Serial.print(Rt);
    //Serial.print(" ");
    //Serial.println(temperatureSensor0);
    //Serial.print(" ");
    delay(1000);

    // for X-th sensor
    // replace X for sensor index

    //Medição e cálculo do sensor 1        //ALTERACOES BIRA
    Vo1 = analogRead(PIN_THERMISTOR_1);
    Rt1 = TEMP_R * (1023.0 / (float)Vo1 - 1.0);
    logRt1 = log(Rt1);
    temperatureSensor1 = (int) ((1.0 / (TEMP_C1 + TEMP_C2*logRt1 + TEMP_C3*logRt1*logRt1*logRt1)) - 273.15);
    temperatureSensor0 = (int) ((1.0 / (TEMP_C1 + TEMP_C2*logRt +  TEMP_C3*logRt *logRt *logRt )) - 273.15);
    


}

/********************************************************************************************************
    ACTIONS ON DIFFERENT TEMPERATURES
*********************************************************************************************************/
void actionHighTemperature0()
{
    digitalWrite (PIN_BUZZER_MOTORTEMP, HIGH); // BUZZER_SINAL SONORO DISPARA (REPRESENTA AQUI POR LED VERMELHO)
    delay(50);
    digitalWrite (PIN_BUZZER_MOTORTEMP, LOW); // BUZZER_SINAL SONORO DISPARA (REPRESENTA AQUI POR LED VERMELHO)
    Serial.print("TEMP MOTOR MUTO ALTA =  ");
    Serial.println(temperatureSensor0);
    Serial.println(" ");
}

void actionMidTemperature0()
{
    digitalWrite (PIN_BUZZER_MOTORTEMP, HIGH); // BUZZER_SINAL SONORO SILENCIADO (REPRESENTA AQUI POR LED VERMELHO)
    delay(500);
    digitalWrite (PIN_BUZZER_MOTORTEMP, LOW); // BUZZER_SINAL SONORO SILENCIADO (REPRESENTA AQUI POR LED VERMELHO)
    Serial.print("TEMP MOTOR ACIMA NORMAL =  ");
    Serial.println(temperatureSensor0);
    Serial.println(" ");
    
}

void actionLowTemperature0()
{
    digitalWrite (PIN_BUZZER_MOTORTEMP, LOW); // BUZZER_SINAL SONORO SILENCIADO
    Serial.print("TEMP MOTOR NORMAL =  ");
    Serial.println(temperatureSensor0);
    Serial.println(" ");
}

/********************************************************************************************************
    MAIN FUNCTIONS
*********************************************************************************************************/

void setup()
{
    Serial.begin(9600);    // open serial port and set data rate to 9600 bps
    Serial.println("Thermistor temperature measurement:");
    Serial.println("\n Vo Rt T (C)");
   
    pinMode(PIN_BUZZER_MOTORTEMP, OUTPUT);  // Pino 11 declarado como saída BUZZER
    pinMode(PIN_BUZZER_WATERLEVEL, OUTPUT); // Pino 12 declarado como saída BUZZER
  
    digitalWrite(PIN_BUZZER_MOTORTEMP, LOW); 
    digitalWrite(PIN_BUZZER_WATERLEVEL, LOW);
    
    // initialize the pushbutton pin as an input:
    pinMode(BUTTON_PIN, INPUT);
    
    lcd.clear();      //ALTERACOES BIRA
    lcd.begin(16, 4); //Inicia o LCD com dimensões 16x4(Colunas x Linhas)      //ALTERACOES BIRA
   
}

  // -- loop() is repeated indefinitely
void loop()
{
    measureTemperature();
    delay(100);
    
    lcd.clear();
    //Mostra Temperatura Sensor 0 no LCD
    lcd.setCursor(0, 0); //Posiciona o cursor na décima quarta coluna(13) e na segunda linha(1) do LCD
    lcd.print("Temp0");
    lcd.print(": ");
    lcd.print(temperatureSensor0); //Escreve o valor atual da variável de contagem no LCD
    lcd.print((char)223);
    lcd.print("C");

    
    //Mostra Temperatura Sensor 1 no LCD
    lcd.setCursor(0, 1); //Posiciona o cursor na décima quarta coluna(13) e na segunda linha(1) do LCD
    lcd.print("Temp1");
    lcd.print(": ");
    lcd.print(temperatureSensor1); //Escreve o valor atual da variável de contagem no LCD
    lcd.print((char)223);
    lcd.print("C");
    
    
    //Mostra Temperatura Sensor 1 no LCD
    lcd.setCursor(1, 2); //Posiciona o cursor na décima quarta coluna(13) e na segunda linha(1) do LCD
    lcd.print("Temp1");
    lcd.print(": ");
    //lcd.print(temperatureSensor1); //Escreve o valor atual da variável de contagem no LCD
    //lcd.print((char)223);
    //lcd.print("C");
   
  /*  
    lcd.clear();
  //Posiciona o cursor na coluna 3, linha 0;
  lcd.setCursor(3, 0);
  //Envia o texto entre aspas para o LCD
  lcd.print("FILIPEFLOP");
  lcd.setCursor(3, 1);
  lcd.print(" LCD 16x2");
  delay(5000);
  */
  


    // read the state of the pushbutton value:
    buttonState = digitalRead(BUTTON_PIN);

    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH:
    if (buttonState == HIGH)
    {
     Serial.println("NIVEL RESERVATORIO BAIXO");
     digitalWrite(PIN_BUZZER_WATERLEVEL, HIGH);
     lcd.setCursor(0, 3); 
     lcd.print("RES. OK");
    }
    else
    {
     Serial.println("NIVEL RESERVATORIO NORMAL");
     digitalWrite(PIN_BUZZER_WATERLEVEL, LOW);
     lcd.setCursor(0, 3); 
     lcd.print("RES. BAIXO");
    }
    //
    // temperature state-machine
    //
    if (temperatureSensor0 > HIGH_TEMPERATURE)
    {
        actionHighTemperature0();
    }
    else if ((temperatureSensor0 >= MID_TEMPERATURE) && (temperatureSensor0 <= HIGH_TEMPERATURE))
    {
        actionMidTemperature0();
    }
    else 
    {
        // temperatureSensor0 < 28
        actionLowTemperature0();
    }
}
