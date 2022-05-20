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
float LMin = -123.429;      // Corresponde a la distancia mínima, es decir, cuando el ángulo es -10 grados. Corresponde además al lugar dónde irá colocado el sensor inicio de carrera
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
float rad, radf, radMin=0;            // Variable para pasar el número de grados a radianes para poder utilizar la tangente

// El siguiente vector muestra los tiempos de cada medio micropaso dependiendo de la velocidad seleccionado. Este vector se encuentra en un .csv en el GitHub del autor del trabajo
// El último elemento corresponde a la mayor velocidad que puede alcanzar para que pueda ir hasta el ángulo inicial indicado a una velocidad mayor
float vectort_mediomicropaso[] = {0,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,76.80416624,75.50240071,74.24402737,73.02691216,71.84905874,70.70859749,69.60377566,68.53294834,67.49457033,66.48718869,65.50943591,64.5600238,63.63773774,62.74143158,61.87002281,61.02248825,60.19786003,59.39522189,58.61370582,57.85248886,57.11079028,56.38786889,55.68302053,54.99557583,54.32489807,53.67038123,53.03144812,52.40754873,51.79815863,51.20277749,50.62092775,50.05215328,49.49601824,48.95210596,48.42001785,47.89937249,47.3898047,46.89096465,46.4025171,45.92414064,45.45552696,44.99638022,11.25};

// El siguiente vector muestra los tiempos de cada medio paso dependiendo de la velocidad seleccionado. Este vector se encuentra en un .csv en el GitHub del autor del trabajo
// El último elemento corresponde a la mayor velocidad que puede alcanzar para que pueda ir hasta el ángulo inicial indicado a una velocidad mayor
float vectort_mediopaso[] = {0,142548.5325,71274.26627,47516.17751,35637.13314,28509.70651,23758.08876,20364.07608,17818.56657,15838.72584,14254.85325,12958.9575,11879.04438,10965.27173,10182.03804,9503.235503,8909.283284,8385.207797,7919.362919,7502.554344,7127.426627,6788.025359,6479.478752,6197.762285,5939.522189,5701.941302,5482.635867,5279.575279,5091.019019,4915.466639,4751.617751,4598.339759,4454.641642,4319.652501,4192.603898,4072.815216,3959.68146,3852.663042,3751.277172,3655.090578,3563.713314,3476.793477,3394.01268,3315.082152,3239.739376,3167.745168,3098.881142,3032.947501,2969.761095,2909.153725,2850.970651,2795.069266,2741.317934,2689.594954,2639.78764,2591.791501,2545.50951,2500.851448,2457.73332,2416.076823,2375.808876,2336.861189,2299.16988,2262.67512,2227.320821,2193.054347,2159.826251,2127.590038,2096.301949,2065.920762,2036.407608,2007.72581,1979.84073,1952.719624,1926.331521,1900.647101,1875.638586,1851.279643,1827.545289,1804.411804,1781.856657,1759.858426,1738.396738,1717.452199,1697.00634,1677.041559,1657.541076,1638.48888,1619.869688,1601.668905,1583.872584,1566.467391,1549.440571,1532.77992,1516.47375,1500.510869,1484.880547,1469.5725,1454.576863,1439.884167,360};


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
          rad= (ang_i * pi)/180;//Conversión de los grados a radianes
          di_X= Lb*tan(rad);// conversión de los radianes a la distancia en mm ya que de esta forma es más sencillo establecer posteriormente la relación entre el avance de los pasos con la distancia 
          radf= (ang_f * pi)/180;
          df_X= Lb*tan(radf);
          radMin= (AMin * pi)/180;
          LMin= Lb*tan(radMin);//Para asegurar la conversión de la distancia a la que se encuentra el sensor inicial.
          posicion = Vmax;//Prueba para ver si se mueve a la máxima para ir a inicio o a distancia inicial
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
    if  (di_X == LMin or di_X <= (0.00005*n_mediospasos + LMin)) { 
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
    avance_mediospasos2 = (avance_mediospasos + LMin - di_X);     //Este segundo es para los casos en los que no empiezo con ángulo inicial de -10 grados sino que empiezo a distancia x=5mm por ejemplo

    lcd.setCursor(0, 3);
    lcd.print("Avance: ");
    lcd.setCursor(7, 3);
    
    if (di_X == LMin){     
    lcd.print(avance_mediospasos);}
    else{
      if ((avance_mediospasos + LMin) >= di_X){
          lcd.print(avance_mediospasos2);}
      else{
          lcd.print(0);
          }
    }
    
    lcd.setCursor(12, 3);
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
  else if (inicio_contadortiempo == 1 && fin ==0 && (avance_mediospasos + LMin <= df_X)){  //para que valiese para contar cuando vaya hacia atrás también habría que poner: (avance_mediospasos < abs(dif)) siendo dif=df_X-di_X)
      
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
    if (estado == 3 && (avance_mediospasos + LMin <= df_X) && fin == 0){
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
  
    if (inicio == 1 && fin ==0 && (avance_mediospasos + LMin <= df_X)){ //        <- INTERRUPCION MOVIMIENTO MOTOR TIEMPO TOTAL MEDIO PASO
    n_mediospasos++;
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
