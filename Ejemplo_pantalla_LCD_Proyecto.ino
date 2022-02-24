#include <LiquidCrystal.h>
LiquidCrystal lcd(12,11,5,4,3,2);

const int switchPin = 6; 
int switchState = 0;
int prevSwitchState = 0;
int reply;
float df_X = 0;                   // Distancia inicial y final que queremos en el experimento
int Angulofinal = 0;   // Ánulos iniciales, será lo que elija el usuario en este experimento
const float pi = 3.14159265;  // pi number
float rad = 0;
unsigned long tiempo1=0;
unsigned long tiempo2=0;
unsigned long diferenciatiempo=0;

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16,2);
  pinMode(switchPin, INPUT);
  lcd.print("Para el");
  lcd.setCursor(0,1);
  lcd.print("Angulo dado");

}

void loop() {
  // put your main code here, to run repeatedly:
  switchState = digitalRead(switchPin);

  if (switchState != prevSwitchState){
    if (switchState == LOW) {
      reply = random(8);

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("La distancia es:");
      lcd.setCursor(0,1);

      switch(reply){
        case 0:
//        AnguloFinal=reply;
        df_X= 700*tan(reply);
        lcd.print(df_X);
        break;
        case 1:
//        AnguloFinal=reply;
        tiempo1 = millis();
        rad= (reply * pi)/180;
        df_X= 700*tan(rad);
        tiempo2= millis();
        lcd.print(df_X);
        diferenciatiempo= tiempo1-tiempo2;
        lcd.setCursor(9,1);
        lcd.print(diferenciatiempo);
        break;
        case 2:
//        AnguloFinal=reply;
        tiempo1 = millis();
        rad= (reply * pi)/180;
        df_X= 700*tan(rad);
        tiempo2= millis();
        lcd.print(df_X);
        diferenciatiempo= tiempo1-tiempo2;
        lcd.setCursor(9,1);
        lcd.print(diferenciatiempo);
        break;
        case 4:
        tiempo1 = millis();
        rad= (reply * pi)/180;
        df_X= 700*tan(rad);
        tiempo2= millis();
        lcd.print(df_X);
        diferenciatiempo= tiempo1-tiempo2;
        lcd.setCursor(9,1);
        lcd.print(diferenciatiempo);
        break;
        case 5:
        tiempo1 = millis();
        rad= (reply * pi)/180;
        df_X= 700*tan(rad);
        tiempo2= millis();
        lcd.print(df_X);
        diferenciatiempo= tiempo1-tiempo2;
        lcd.setCursor(9,1);
        lcd.print(diferenciatiempo);
        break;
        case 6:
        tiempo1 = millis();
        rad= (reply * pi)/180;
        df_X= 700*tan(rad);
        tiempo2= millis();
        lcd.print(df_X);
        diferenciatiempo= tiempo1-tiempo2;
        lcd.setCursor(9,1);
        lcd.print(diferenciatiempo);
        break;
        case 7:
        tiempo1 = millis();
        rad= (reply * pi)/180;
        df_X= 700*tan(rad);
        tiempo2= millis();
        lcd.print(df_X);
        diferenciatiempo= tiempo1-tiempo2;
        lcd.setCursor(9,1);
        lcd.print(diferenciatiempo);
        break;
        
      }
    }
  }
 prevSwitchState = switchState;
}
