//CÓDIGO FINAL ARDUINO PARA SIMULACIÓN GEOLÓGICA CON MOVIMIENTO ANGULAR//Falta cambiar los vectores y la sensibilidad

#include <TimerOne.h>       // Libreria del timer 1
#include <TimerThree.h>     // Libreria del timer 3
#include <TimerFour.h>      // Libreria del timer 4

#include <LiquidCrystal.h>  // Libreria de la pantalla LCD
LiquidCrystal lcd(16, 17, 23, 25, 27, 29);

//Pines de la shield
#define LCD_PINS_RS 16      // LCD control conectado a GADGETS3D  shield LCDRS
#define LCD_PINS_ENABLE 17  // LCD enable pin conectado a GADGETS3D shield LCDE
#define LCD_PINS_D4 23      // LCD signal pin, conectado a GADGETS3D shield LCD4
#define LCD_PINS_D5 25      // LCD signal pin, conectado a GADGETS3D shield LCD5
#define LCD_PINS_D6 27      // LCD signal pin, conectado a GADGETS3D shield LCD6
#define LCD_PINS_D7 29      // LCD signal pin, conectado a GADGETS3D shield LCD7
#define X_MIN_PIN 3         // PIN para el fin de carrera colocado al inicio del recorrido (1: presionado (true))
#define X_MAX_PIN 2         // PIN para el fin de carrera colocado al inicio del recorrido (1: presionado (true))

#define X_STEP_PIN 54       // PIN de los pasos del controlador DRV8825 del motor paso a paso
#define X_DIR_PIN 55        // PIN de la direccion del controlador DRV8825 del motor paso a paso
#define X_ENABLE_PIN 38     // PIN del enable del controlador DRV8825 del motor paso a paso

#define BTN_EN1 31          // Encoder, conectado a GADGETS3D shield S_E1
#define BTN_EN2 33          // Encoder, cconectado a GADGETS3D shield S_E2
#define BTN_ENC 35          // Encoder Click, connected to Gadgets3D shield S_EC

// VARIABLES MEDIDA POSICION

bool fc_inic_X = true;      // Valor del final de carrera situacion al inicio del experimento
bool fc_fin_X = true;       // Valor del final de carrera situacion al final del experimento

const float L = 254.78;     // Corresponde a la distancia máxima, es decir, cuando el ángulo es 20 grados
const float LMin = -123.43; // Corresponde a la distancia mínima, es decir, cuando el ángulo es -10 grados
int Lb = 700;               // Longitud entre la bisagra especial y la bisagra principal
const float pi = 3.14159265;// pi number
int AMax = 20;              // Ángulo máximo que se puede llevar a cabo
int AMin = -10;             // Ángulo minimo que se puede llevar a cabo

// VARIABLES MEDIDA TIEMPO 

int horas = 0;              // Variable que cuenta las horas del experimento
int minutos = 0;            // Variable que cuenta los minutos del experimento
volatile int segundos = 0;  // Variable que cuenta los segundos del experimento

// VARIABLES MOVIMIENTO MOTOR

unsigned long t_mediomicropaso = 0;   // Variable en la que introducimo el tiempo de cada medio micropaso
unsigned long t_mediopaso = 0;        // Variable en la que introducimo el tiempo de cada medio paso
int v = 1;                            // Velocidad del sistema
int Vmax = 100;                        // Posición 100 de los vectores de medio paso y micro paso(1000 cm/h)
int posicion = 0;                     //Cogerá v si estoy en el experimento normal o Vmax si estoy yendo a 0
volatile int nivel = LOW;             // Nivel del motor paso a paso
unsigned long micropasos = 0;         // Numero de micropasos dados
unsigned long n_mediospasos = 0;      // Numero de medios pasos cuando el experimento empieza en 0 mm
volatile int n_mediospasos2 = 0;      // Numero de medios pasos cuando el experimento no empieza en 0 mm
float avance_mediospasos = 0;         // Avance a partir de los medios pasos dados por el motor conocido su avance por cada medio paso cuando el experimento empieza en 0 mm
float avance_mediospasos2 = 0;        // Avance a partir de los medios pasos dados por el motor conocido su avance por cada medio paso cuando el experimento no empieza en 0 mm
float di_X, df_X = 0;                 // Distancia inicial y final que queremos en el experimento. Lo he cambiado por float para que pueda tener decimales
float ang_i, ang_f = 0;               // Ánulos iniciales, será lo que elija el usuario en este experimento
float rad, radf=0;                    // Variable para pasar el número de grados a radianes para poder utilizar la tangente

// El siguiente vector muestra los tiempos de cada medio micropaso dependiendo de la velocidad seleccionado. Este vector se encuentra en un .csv en el GitHub del autor del trabajo
// El último elemento corresponde a la mayor velocidad que puede alcanzar para que pueda ir hasta el ángulo inicial indicado a una velocidad mayor
float vectort_mediomicropaso[] = {0,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,28};

// El siguiente vector muestra los tiempos de cada medio paso dependiendo de la velocidad seleccionado. Este vector se encuentra en un .csv en el GitHub del autor del trabajo
// El último elemento corresponde a la mayor velocidad que puede alcanzar para que pueda ir hasta el ángulo inicial indicado a una velocidad mayor
float vectort_mediopaso[] = {0,7127426.627,3563713.314,2375808.876,1781856.657,1425485.325,1187904.438,1018203.804,890928.3284,791936.2919,712742.6627,647947.8752,593952.2189,548263.5867,509101.9019,475161.7751,445464.1642,419260.3898,395968.146,375127.7172,356371.3314,339401.268,323973.9376,309888.1142,296976.1095,285097.0651,274131.7934,263978.764,254550.951,245773.332,237580.8876,229916.988,222732.0821,215982.6251,209630.1949,203640.7608,197984.073,192633.1521,187563.8586,182754.5289,178185.6657,173839.6738,169700.634,165754.1076,161986.9688,158387.2584,154944.0571,151647.375,148488.0547,145457.6863,142548.5325,139753.4633,137065.8967,134479.7477,131989.382,129589.575,127275.4755,125042.5724,122886.666,120803.8411,118790.4438,116843.0595,114958.494,113133.756,111366.0411,109652.7173,107991.3125,106379.5019,104815.0975,103296.0381,101820.3804,100386.2905,98992.03649,97635.98119,96316.57604,95032.35503,93781.92931,92563.98217,91377.26445,90220.59022,89092.83284,87992.92132,86919.83692,85872.60997,84850.31699,83852.07797,82877.0538,81924.44399,80993.4844,80083.44525,79193.62919,78323.36953,77472.02856,76638.99599,75823.68752,75025.54344,74244.02737,73478.62502,72728.84313,71994.20836,900};


// VARIABLES MAQUINA DE ESTADOS

int inicio = 0;                       // Variable que inicia el experimento cuando comienza en el sensor de principio de carrera
int inicio_experimento = 0;           // Variable que inicia el experimento cuando comienza en di_X mm
int inicio_contadortiempo = 0;       // Variable que inicia el experimento cuando no comienza en 0 mm
int fin = 0;                          // Variable que para el experimento cuando llega al final de carrera 
int volatile estado = 0;              // Variables del estado


// VARIABLES LCD

int fila, columna = 0;                                             // Variable de fila y columna en la pantalla lcd
bool btn_en1, btn_en2, btn_enc, btn_en1_prev, btn_en2_prev ;       // Variables de lectura directa del codificador giratorio mecanico
bool direccion = false;                                            // Direccion del codificador
bool derecha, izquierda, pulsador = false;                         // Variables de lectura del codificador interpretadas
int i = 0;                                                         // Contador de pulsos

// SIGNOS CREADOS

byte empty[8] =
{
  B00000,
  B00100,
  B01010,
  B10001,
  B10001,
  B01010,
  B00100,
};
byte full[8] =
{
  B00000,
  B00100,
  B01110,
  B11111,
  B11111,
  B01110,
  B00100,
};
byte micro[8] =
{
  B00000,
  B10001,
  B10001,
  B11011,
  B10101,
  B10000,
  B10000,
};
byte arrow[8] =
{
  B00000,
  B00001,
  B00001,
  B00001,
  B01001,
  B11111,
  B01000,
};

void setup() {

  pinMode(BTN_EN1, INPUT_PULLUP);     // Encoder 1
  pinMode(BTN_EN2, INPUT_PULLUP);     // Encoder 2
  pinMode(BTN_ENC, INPUT_PULLUP);     // Encoder Swith
  
  pinMode(X_MIN_PIN, INPUT_PULLUP);   // Fin de carrera inicio  
  pinMode(X_MAX_PIN, INPUT_PULLUP);   // Fin de carrera final           

  lcd.begin(20, 4);   // 20 columnas y 4 filas

  pinMode(X_STEP_PIN , OUTPUT);
  pinMode(X_DIR_PIN , OUTPUT);
  pinMode(X_ENABLE_PIN , OUTPUT);

  btn_en1 , btn_en1_prev = digitalRead(BTN_EN1);
  btn_en2 , btn_en2_prev = digitalRead(BTN_EN2);
  btn_enc = digitalRead(BTN_ENC);

  lcd.createChar(0, empty);   // 0: numero de carácter; empty: matriz que contiene los pixeles del carácter
  lcd.createChar(1, full);    // 1: numero de carácter; full: matriz que contiene los pixeles del carácter
  lcd.createChar(2, micro);   // 2: numero de carácter; micro: matriz que contiene los pixeles del carácter
  lcd.createChar(3, arrow);   // 3: número de carácter; arrow: matriz que contiene los pixeles del carácter
 
  digitalWrite(X_ENABLE_PIN , LOW);         // Habilitación a nivel bajo del motor paso a paso

//  attachInterrupt(digitalPinToInterrupt(sensor1), encoder, RISING);     // Función de la interrupcion del encoder

  Timer3.initialize(1000000);              // Inicialización de la interrupcion del contador de segundos
  Timer3.attachInterrupt(Temporizador);    // Función de la interrupcion del contador de segundos
  
}

////////////////// PROCESOS ENCODER /////////////////////

void leer_encoder()
{
  btn_en1 = digitalRead(BTN_EN1);
  btn_en2 = digitalRead(BTN_EN2);
  digitalWrite(X_DIR_PIN, direccion);

  if (btn_en1 != btn_en1_prev || btn_en2 != btn_en2_prev)
  {
    if (btn_en2 == false & btn_en1 == false & btn_en2_prev == true & btn_en1_prev == false)
    {
      derecha = true;
      izquierda = false;
    }
    else if ( btn_en2 == false & btn_en1 == false & btn_en2_prev == false & btn_en1_prev == true )
    {
      derecha = false;
      izquierda = true;
    }
    else
    {
      derecha = false;
      izquierda = false;
    }
  }
  else
  {
    derecha = false;
    izquierda = false;
  }
  btn_en1_prev = btn_en1;
  btn_en2_prev = btn_en2;
}

void leer_pulso()
{
  btn_enc = digitalRead(BTN_ENC);

  if (btn_enc == false) //Detector de flanco del pulsador
  {
    i++;
  }
  if (i >= 80)
  {
    pulsador = true;
    i = 0;
    delay(200);
  }
  else
  {
    pulsador = false;
  }
}

//////////////////////// ESTADO 0 /////////////////////////

void pantalla_inicio()
{
  lcd.clear();
  lcd.setCursor(2, 0);    // posiciona el cursor en la columna 1 fila 0
  lcd.print("ENSAYO DE MODELO");
  lcd.setCursor(3, 1);    // posiciona el cursor en la columna 1 fila 1
  lcd.print("ANALOGO SIMPLE");

  lcd.setCursor(0, 3);    // posiciona el cursor en la columna 1 fila 3
  lcd.print("Iniciar Experimento");
  
}


//////////////// ESTADO 1 ////////////////

void menu()
{
  fila, columna = 0;
  di_X = 0;
  df_X = 0;
  ang_i = 0;
  ang_f = 0;
  v = 1;

  lcd.setCursor(1, 0);
  lcd.print("ang.ini");
  lcd.setCursor(12, 0);
  lcd.print(ang_i);
  lcd.setCursor(18, 0);
  lcd.print(char(223));//para que me aparezcan los grados de unidad de medida
  lcd.setCursor(1, 1);
  lcd.print("ang.fin");
  lcd.setCursor(12, 1);
  lcd.print(ang_f);
  lcd.setCursor(18, 1);
  lcd.print(char(223));
  lcd.setCursor(1, 2);
  lcd.print("velocidad");
  lcd.setCursor(12, 2);
  lcd.print(v);
  lcd.setCursor(16, 2);
  lcd.print(char(223));
  lcd.setCursor(17, 2);
  lcd.print("/h");
  lcd.setCursor(1, 3);
  lcd.print("Iniciar experimento");
  lcd.setCursor(0, 0);
  lcd.write(byte(0));
  lcd.setCursor(0, 1);
  lcd.write(byte(0));
  lcd.setCursor(0, 2);
  lcd.write(byte(0));
  lcd.setCursor(0, 3);
  lcd.write(byte(0));
  lcd.setCursor(0, fila);
  lcd.write(byte(1));
}

//////////////// ESTADO 2 ////////////////

void DefinicionDeVariables()
{
  switch (fila) // fila es la variable vertical de la pantalla, indica que variable se esta manipulando
  {
    case 0: // modificar el ángulo inicial
      if (pulsador == true  and columna < 2) // columna es una variable que avanza en horizontal por la pantalla, cuando vale 0 podemos cambia en vertical con derecha o izquierda, cuando no aumentamos la variable con la que estemos trabajando
      {
        columna++;
      }
      else if (pulsador == true and columna == 2) //hay 3 opciones
      {
        columna = 0;
      }
      switch (columna)
      {
        case 0: //opción 1: seleccionar variable
          if (derecha == true )
          {
            fila = 1;
            lcd.setCursor(0, 0);
            lcd.write(byte(0));
            lcd.setCursor(0, 1);
            lcd.write(byte(1));
            lcd.setCursor(0, 2);
            lcd.write(byte(0));
            lcd.setCursor(0, 3);
            lcd.write(byte(0));
          }
          else if (izquierda == true)
          {
            fila = 3;
            lcd.setCursor(0, 0);
            lcd.write(byte(0));
            lcd.setCursor(0, 1);
            lcd.write(byte(0));
            lcd.setCursor(0, 2);
            lcd.write(byte(0));
            lcd.setCursor(0, 3);
            lcd.write(byte(1));
          }
          break;
        case 1: //opción 2: ir sumando y restando de 0.1 en 0.1
          if (derecha == true and ang_i +0.1 < AMax)
          {
            ang_i = ang_i + 0.1;
            lcd.setCursor(12, 0);
            lcd.print("   ");
            lcd.setCursor(12, 0);
            lcd.print(ang_i);
          }
          else if (izquierda == true and ang_i -0.1 > AMin)
          {
            ang_i = ang_i - 0.1;
            lcd.setCursor(12, 0);
            lcd.print("   ");
            lcd.setCursor(12, 0);
            lcd.print(ang_i);
          }
          break;
          //rad= (ang_i * pi)/180;//Hago el cambio aquí para que sólo lo realice una vez
          //di_X= Lb*tan(rad); //¿Hay algún problema en poner los rad más de una vez? Yo creo que no porque sería secuencial
          
        default://opción 3: ir sumando y restando de 1 en 1
          if (derecha == true and ang_i +1 < AMax)
          {
            ang_i = ang_i + 1;
            lcd.setCursor(12, 0);
            lcd.print("   ");
            lcd.setCursor(12, 0);
            lcd.print(ang_i);
          }
          else if (izquierda == true and ang_i -1 > AMin)
          {
            ang_i = ang_i - 1;
            //rad= (ang_i * pi)/180;
            //di_X= Lb*tan(rad);
            lcd.setCursor(12, 0);
            lcd.print("   ");
            lcd.setCursor(12, 0);
            lcd.print(ang_i); 
          }
          break;
          rad= (ang_i * pi)/180;
          di_X= Lb*tan(rad);
        
      }
      break;
    case 1: // modificar el ángulo final
      if (pulsador == true  and columna < 2) // columna es una variable que avanza en horizontal por la pantalla, cuando vale 0 podemos cambia en vertical con derecha o izquierda, cuando no aumentamos la variable con la que estemos trabajando
      {
        columna++;
      }
      else if (pulsador == true and columna == 2)
      {
        columna = 0;
      }
      switch (columna)
      {
        case 0: //opción 1: seleccionar variable
          if (derecha == true )
          {
            fila = 2;
            lcd.setCursor(0, 0);
            lcd.write(byte(0));
            lcd.setCursor(0, 1);
            lcd.write(byte(0));
            lcd.setCursor(0, 2);
            lcd.write(byte(1));
            lcd.setCursor(0, 3);
            lcd.write(byte(0));
          }
          else if (izquierda == true)
          {
            fila = 0;
            lcd.setCursor(0, 0);
            lcd.write(byte(1));
            lcd.setCursor(0, 1);
            lcd.write(byte(0));
            lcd.setCursor(0, 2);
            lcd.write(byte(0));
            lcd.setCursor(0, 3);
            lcd.write(byte(0));
          }
          break;
        case 1: //opción 2: ir sumando y restando de 0.1 en 0.1
          if (derecha == true and ang_f + 0.1 < AMax)
          {
            ang_f = ang_f + 0.1;
            lcd.setCursor(12, 1);
            lcd.print("   ");
            lcd.setCursor(12, 1);
            lcd.print(ang_f);
          }
          else if (izquierda == true and ang_f -0.1 > AMin)
          {
            ang_f = ang_f - 0.1;
            //radf= (ang_f * pi)/180;
            //df_X= Lb*tan(radf);
            lcd.setCursor(12, 1);
            lcd.print("   ");
            lcd.setCursor(12, 1);
            lcd.print(ang_f);
          }
          break;

        default: //opción 3: ir sumando y restando de 1 en 1
          if (derecha == true and ang_f +1 < AMax)
          {
            ang_f = ang_f + 1;
            //radf= (ang_f * pi)/180;
            //df_X= Lb*tan(radf);
            lcd.setCursor(12, 1);
            lcd.print("   ");
            lcd.setCursor(12, 1);
            lcd.print(ang_f);
          }
          else if (izquierda == true and ang_f -1 > AMin)
          {
            ang_f = ang_f - 1;
            //radf= (ang_f * pi)/180;
            //df_X= Lb*tan(radf);
            lcd.setCursor(12, 1);
            lcd.print("   ");
            lcd.setCursor(12, 1);
            lcd.print(ang_f);
          }
          break;
          radf= (ang_f * pi)/180;
          df_X= Lb*tan(radf);
      }
      break;
    case 2: // modificar la velocidad
      if (pulsador == true  and columna < 1) // columna es una variable que avanza en horizontal por la pantalla, cuando vale 0 podemos cambia en vertical con derecha o izquierda, cuando no aumentamos la variable con la que estemos trabajando
      {
        columna++;
      }
      else if (pulsador == true and columna == 1) //hay 2 opciones
      {
        columna = 0;
      }
      switch (columna)
      {
        case 0: //opción 1: seleccionar variable
          if (derecha == true )
          {
            fila = 3;
            lcd.setCursor(0, 0);
            lcd.write(byte(0));
            lcd.setCursor(0, 1);
            lcd.write(byte(0));
            lcd.setCursor(0, 2);
            lcd.write(byte(0));
            lcd.setCursor(0, 3);
            lcd.write(byte(1));
          }
          else if (izquierda == true)
          {
            fila = 1;
            lcd.setCursor(0, 0);
            lcd.write(byte(0));
            lcd.setCursor(0, 1);
            lcd.write(byte(1));
            lcd.setCursor(0, 2);
            lcd.write(byte(0));
            lcd.setCursor(0, 3);
            lcd.write(byte(0));
          }
          break;
        default: // opción 2: modificar la velocidad de 0.1 en 0.1 o de 1 en 1 en función de la velocidad
          if (izquierda == true and v - 1 >= 0)
          {
              v = v - 1;//Me permite guardar la velocidad angular como si estuviera multiplicada por 10 para que sea la posición que tengo que coger del vector de micropasos y medios pasos. El decir, si yo tengo w=0.2 grados/h la v será 2 que corresponde a la posición 2 del vector.
              lcd.setCursor(12, 2);
              lcd.print(v/10);//Mediante la división obtenemos las unidades. Es decir si la w angular es 3.2 al dividir v=32 entre 10 me muestra únicamente el 3
              lcd.setCursor(13, 2);
              lcd.print(".");
              lcd.setCursor(14, 2);
              lcd.print(v%10);//Mediante el %10 obtenemos las décimas. Es decir si la w angular es 3.2 al utilizar %10 me dará el resto de dividir v entre 10. Por tanto para v=32 me muestra únicamente el 2
          }
          else if (derecha == true and v < 99)
          {
              v = v + 1;
              lcd.setCursor(12, 2);
              lcd.print(v);
              lcd.setCursor(12, 2);
              lcd.print(v/10);//ponerlo dentro o fuera
              lcd.setCursor(13, 2);
              lcd.print(".");
              lcd.setCursor(14, 2);
              lcd.print(v%10);
          }
          
          
//        if (v <1 && v >=0){
//        lcd.setCursor(13, 2);
//        lcd.print(" ");
//        }
        if (v <10 && v >=0){//Pongo menor que 10 porque la máxima velocidad angular es 9.9
        lcd.setCursor(15, 2);//Creo que tendría que poner 15 para que ponga el punto de los decimales. Antes estaba puesto el 14
        lcd.print(" ");
        }          
          break;
      }
      break;
      
    default: // iniciar experimento
    
      if (pulsador == true)
      {
          rad= (ang_i * pi)/180;
          di_X= Lb*tan(rad);
          radf= (ang_f * pi)/180;
          df_X= Lb*tan(radf);
          //if ((inicio == 0 && fin ==0)or(inicio == 1 && fin ==0 && (di_X > (0.0025*n_mediospasos))))   { //LO HE CAMBIADO. no tiene sentido poner esto 
            posicion = Vmax;//Prueba para ver si se mueve a la máxima para ir a inicio o a distancia inicial
//          }
//          else { 
//             posicion = v; // para el experimento lleva la velocidad indicada por el usuario
//          }
          t_mediomicropaso = ((unsigned long)vectort_mediomicropaso[posicion]);     
          Timer1.attachInterrupt(Micropasos);             // Funcion de la interrupcion de los micropasos
          Timer1.initialize(t_mediomicropaso);            // Inicializacion de la interrupcion de los micropasos
          Timer4.attachInterrupt(MedioPaso);              // Funcion de la interrupcion de los medios pasos
          t_mediopaso = ((unsigned long)(vectort_mediopaso[posicion]));  
          Timer4.initialize(t_mediopaso);                 // Inicializacion de la interrupcion de los medios pasos
         
          lcd.clear();
          estado = 3;
        
      }
      else if (derecha == true )
      {
        fila = 0;
        columna = 0;
        lcd.setCursor(0, 0);
        lcd.write(byte(1));
        lcd.setCursor(0, 1);
        lcd.write(byte(0));
        lcd.setCursor(0, 2);
        lcd.write(byte(0));
        lcd.setCursor(0, 3);
        lcd.write(byte(0));
      }
      else if (izquierda == true)
      {
        fila = 2;
        columna = 0;
        lcd.setCursor(0, 0);
        lcd.write(byte(0));
        lcd.setCursor(0, 1);
        lcd.write(byte(0));
        lcd.setCursor(0, 2);
        lcd.write(byte(1));
        lcd.setCursor(0, 3);
        lcd.write(byte(0));
      }
      break;
  }
}

//////////////// ESTADO 3 ////////////////

void experimento() {

  fc_inic_X = digitalRead(X_MIN_PIN);
  fc_fin_X = digitalRead(X_MAX_PIN);
  
  if (fc_inic_X == true )
  {
    inicio = 1;
  }

  if (fc_fin_X == true )
  {
    fin = 1;
  }

  if (inicio == 1) {
      inicio_experimento = 1;
    if  (di_X ==0 or di_X <= (0.0025*n_mediospasos)) { 
      inicio_contadortiempo = 1;
      posicion = v; // para el experimento lleva la velocidad indicada
      t_mediomicropaso = ((unsigned long)vectort_mediomicropaso[posicion]);     
      Timer1.attachInterrupt(Micropasos);             // Funcion de la interrupcion de los micropasos
      Timer1.initialize(t_mediomicropaso);            // Inicializacion de la interrupcion de los micropasos
      Timer4.attachInterrupt(MedioPaso);              // Funcion de la interrupcion de los medios pasos
      t_mediopaso = ((unsigned long)(vectort_mediopaso[posicion]));  
      Timer4.initialize(t_mediopaso);                 // Inicializacion de la interrupcion de los medios pasos
    }
   
  }  

// MOVIMIENTO MOTOR

  if (inicio == 1 && fin == 0) {  //indica la dirección de movimiento del motor, high hacia delante y low hacia atrás
    //añadir en el if de arriba : or (df_X < di_X)
       digitalWrite(X_DIR_PIN , HIGH);
  }
  else { 
       digitalWrite(X_DIR_PIN , LOW);
  }   

// POSICION

  if (inicio_experimento == 1) {
   
    lcd.setCursor(0, 2);
    lcd.print("Mediospasos: ");
    lcd.setCursor(12, 2);    
    lcd.print(n_mediospasos);

    avance_mediospasos = (0.00005*n_mediospasos);       // Avance de cada medio paso del motor = 0.00005 mm, ya que el eje del motor tiene 2 mm/vuelta y una reduccion de 100
    avance_mediospasos2 = (0.00005*n_mediospasos2);     //Este segundo es para los casos en los que no empiezo a una distancia 0 sino que empiezo a distancia x=5mm por ejemplo

    lcd.setCursor(0, 3);
    lcd.print("Avance: ");
    lcd.setCursor(7, 3);
    
    if (di_X == 0){     
    lcd.print(avance_mediospasos);}
    else{
    lcd.print(avance_mediospasos2);
    }
    
    lcd.setCursor(11, 3);
    lcd.print("mm");
      
  }
  
// TIEMPO 
  
  if (inicio == 0 && fin ==0) {
    
    lcd.setCursor(0, 0);
    lcd.print("MOVIENDO");
    lcd.setCursor(0, 1);
    lcd.print("A CERO");
    lcd.setCursor(0, 2);
    lcd.print(n_mediospasos);
    lcd.setCursor(0, 3);
    lcd.print(di_X);
    
    }
  else if (inicio_contadortiempo == 1 && fin ==0 && (avance_mediospasos < df_X)){  //para que valiese para contar cuando vaya hacia atrás también habría que poner: (avance_mediospasos < abs(dif)) siendo dif=df_X-di_X)
      
      if (segundos == 60)    {
      segundos = 0;
      minutos++;           }
      if (minutos == 60)     {
      minutos = 0;
      horas++;             }
      lcd.setCursor(0, 1);
      if (horas < 10)        {
      lcd.print("0");      }
      lcd.print(horas);
      lcd.print(":");
      lcd.setCursor(3, 1);
      if (minutos < 10)      {
      lcd.print("0");      }
      lcd.print(minutos);
      lcd.print(":");
      lcd.setCursor(6, 1);
      if (segundos < 10)     {
      lcd.print("0");        }
      lcd.print(segundos);
    }
}

void Temporizador() {
  
  if (inicio_experimento == 1 && fin ==0) {//                             <- INTERRUPCION CRONOMETRO
    segundos++;
  }
  
}

void Micropasos() {
    if (estado == 3 && (avance_mediospasos < df_X) && fin == 0){//creo que habría que poner aquí tambien lo de arriba y en la función medios pasos también
      if (micropasos < 16){
        digitalWrite(X_STEP_PIN , nivel);
        if (nivel == LOW){       //                             <- INTERRUPCION MOVIMIENTO MOTOR TIEMPO MEDIO MICROPASO
           nivel = HIGH;
           micropasos++;}
        else{
           nivel = LOW;
        }
      }
    }
}

void MedioPaso() {
  
  micropasos = 0;
  
    if (inicio == 1 && fin ==0 && (avance_mediospasos < df_X)){ //        <- INTERRUPCION MOVIMIENTO MOTOR TIEMPO TOTAL MEDIO PASO
    n_mediospasos++;
    }
    
    if ((0.00005*n_mediospasos > di_X)&& (avance_mediospasos < df_X)){  //No cuento los medios pasos hasta que no estoy en la distancia inicial.
    n_mediospasos2++;
    }
}

void loop() {// código que repite continuamente según el estado en el que esté
  
  switch (estado){//Máquina de estados
    case 0:
      pantalla_inicio();
      estado = 1;
      break;
            
    case 1:
      leer_pulso();
  
      if (pulsador == true) { 
        lcd.clear();
        menu();
        estado = 2;
      }
      
      break;

    case 2:
      leer_pulso();
      leer_encoder();
      DefinicionDeVariables();
      break;
   
    case 3: 
      experimento();
      break;    
  }
  
}
