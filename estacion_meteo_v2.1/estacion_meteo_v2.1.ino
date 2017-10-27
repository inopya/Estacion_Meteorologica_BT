/*
#       _\|/_   A ver..., ¿que tenemos por aqui?
#       (O-O)        
# ---oOO-(_)-OOo---------------------------------
 
 
##########################################################
# ****************************************************** #
# *           DOMOTICA PARA PRINCIPIANTES              * #
# *   EST. METEO. con BT y almacenamiento en EEPROM    * #
# *         (acceso al SENSOR DHT11 sin libreria)      * #
# *                                                    * #
# *            Autor: Eulogio López Cayuela            * #
# *                                                    * #
# *          Versión 2.1     Fecha: 21/01/2017         * #
# ****************************************************** #
##########################################################
*/

/*   
    NOTAS SOBRE ESTA VERSION
    
    ===== OPCIONES DISPONIBLES ===== 
 - Muestra datos completos por BT 
 - Muestra datos simplificados en el LCD (presion relativa, temperatura y humedad)
 - Permite cambiar la altitud mediante comandos BT (para el calculo de presion relativa)
 - Guarda la informacion de altitud programada en las dos primeras direcciones de la EEPROM (0,1)
 - Guarda la Temp MAx y Min en las posiciones 2 y 3 de la EEPROM
 - Guarda registros de temperaturas, presion y humedad apartir de la sexta posicion (5) de la EEPROM
 - Comando BT (m) para guardar una marca horaria y los datos de ese momento en la EEPROM
 - listado de datos guardados atraves de BT
 - Listado de datos guardados >>> al puerto serie (se puede dar la orden  por BT o serie)
 - Borrado completo de la EEPROM (respetando las 5 primeras posiciones para no borrar la altitud)
 - Borrado parcial de la EEPROM, solo las posiciones que contienen datos de P,T y H. (respeta la altitud)
 - Posibilidad de cancelar el proceso de marca horaria, cambio de altitud o borrados de EEPROM mediante el comando 'x'
 - Posibilidad de envio de una muestra al puerto Serie. Comando serie (*)
 - Posibilidad de envio de los datos almacenados en la eeprom, al puerto serie. Comando serie (+)
 - ELIMINADA el calculo de la sensacion termica para liberar espacio y mejorar la estabilidad
 - ELIMINADOS muchos caracteres superfluos de los mensajes para liberar 
    casi 200 bytes de RAM y mas de 1500 de memoria de programa evitandose asi inestabilidades
 - AÑADIDA opcion 'v' para mostras solo los ultimos (12) registros   
 - AÑADIDAS banderas para indicar USO y DISPONIBILIDAD de EEPROM evitando que el
    puntero EEPROM se reinicie cuando alcanza el punto maximo y deciciendo si se desean 
    almacenar datos o no en la EEPROM, protegiendola de escrituras innecesarias.
 - AÑADIDA rutina para la prediccon de tiempo a unas cuantas (3) horas vista (basadas en tendencias de presion)
 - AÑADIDA bandera para el control y registro de temperaturas max y min
*/


//------------------------------------------------------
//algunas definiciones personales para mi comodidad al escribir codigo
//------------------------------------------------------
#define AND &&
#define OR ||
#define NOT !
#define ANDbit &
#define ORbit |
#define XORbit ^
#define NOTbit ~
//------------------------------------------------------
//Otras definiciones para pines y variables
//------------------------------------------------------
#define LCD_AZUL_ADDR    0x27  // Direccion I2C de nuestro LCD color azul
#define LCD_VERDE_ADDR   0x3F  // Direccion I2C de nuestro LCD color verde
#define PIN_LED 13             // Led on Board


//------------------------------------------------------
//Definiciones para pines y variables del sensor DHT11
//------------------------------------------------------
#define DHT11_PIN 8 // pin 8, para conectar el sensor DHT11


//------------------------------------------------------
//Importamos las librerias necesarias
//------------------------------------------------------
#include <Wire.h>               // libreria para comunicaciones I2C
#include <LiquidCrystal_I2C.h>  // liquidCrystal library
#include <SFE_BMP180.h>         // libreria para el sensor de presion y temperatura
#include <EEPROM.h>             // libreria para el manejo de la memria EEPROM del MicroControlador

#include <SoftwareSerial.h>     // librería que permite establecer
                                // comunicación serie en pines distintos de RX y TX.

//------------------------------------------------------
// Creamos las instancia de los objetos:
//------------------------------------------------------

//Creamos el objeto 'lcd' como una instancia del tipo "LiquidCrystal_I2C"
//                             addr, en,rw,rs,d4,d5,d6,d7,bl, blpol
LiquidCrystal_I2C lcd(LCD_AZUL_ADDR,  2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

//Creamos el objeto sensor BMP180
SFE_BMP180 sensorBMP180;  //Creamos el objeto 'sensorBMP180' como una instancia del tipo "SFE_BMP180"

//Conectaremos los pins RXD,TDX del módulo Bluetooth a los pines 3 y 4.
SoftwareSerial BT(3,4); // [3 RX arduino ---> TX modulo BT] y [4 TX arduino---> RX modulo BT]
                        // La patilla RX del modulo conectada mediante un divisor de tension para que este a 3'3V 

float ALTITUD = 407.0;    // Altitud de Sorbas en metros

float Temperatura = 0;    //variable para la temperatura leida del BMP180
float TempeMAXIMA = -200; //variable para la temperatura maxima y minima
float TempeMINIMA = 200;  //inicializadas a valores actualizables :)
float PresionRelativaCotaCero = 0;
float PresionABS = 0;     //variable para la presion(absoluta) leida del BMP180

float HumedadREL = 0;     //variable para la humedad leida del DH11
byte TemperaturaDH11 = 0; //variable para la temperatura leida del DH11

byte regMAX = 2;          //dimesionamos para 3 elementos--> (0,1,2)                       
int registroPresion []={0.0,0.0,0.0}; //registro usado en la prediccion del tiempo a tres horas vista
int registroTendencia []={0,0};

//variable para mantener la posicion de la memoria EEPROM que esta disponible para escritura
int punteroEEPROM = 5;

#define MINUTOS  60 //60  //cambiar este valos para modificar la frecuencia del registro de datos

//numero de segundos entre registros salvados a la EEPROM usado en la rutina de grabacion
unsigned long MILISEGUNDOS = MINUTOS * 60000; //no modificar esto. Cambiar los minutos en el #define

boolean FLAG_luz_lcd = true;
boolean FLAG_eeprom_disponible = true;  //indica si queda eeprom disponible para almacenar datos
boolean FLAG_usar_eeprom = false;       //bandera para permitir o no, usar la memoria eeprom
                                        //se puede modificar por BT 
                                        //y se desactiva automaticamante si se llena la eeprom
                                        //para evitar sobreescrituras 
boolean FLAG_control_MaxMin = false;   //activar si se desea monitorizar temperaturas max y min
                                        //y su registro en eeprom


//------------------------------------------------------
//Definiciones para pines y variables del sensor BT
//------------------------------------------------------

char CADENA[6]; //Array de caracteres para el manejo de la informacion del puerto BT
                //Solo vamos a necesitar una, pero si no me hace cosas raras
                //se comparte apra lectura 'fast'  y slow. 

int indiceBUFFER = 0; // mantiene el 'puntero' para el almacenacioento en CADENA[] de caracteres leidos por BT 


//*******************************************************
//         FUNCION DE CONFIGURACION
//*******************************************************

void setup()
{
  BT.begin(9600);
  Serial.begin(9600);  //Serial.begin(19200);
  
  sensorBMP180.begin();
//  if (!sensorBMP180.begin()){
//    Serial.println("ERROR iniciando BMP180");
//  }

  lcd.begin (16,2);    //Inicializar lcd 
  //Mensaje inicial para demostrar que el LCD esta bien conectado y comunicado
  lcd.clear();         //Reset del display 
  lcd.setBacklight(true);     //Activamos la retroiluminacion
  
  lcd.setCursor(0, 0);
  //lcd.print("0123456789ABCDEF");
  lcd.print("T: ");  
  lcd.setCursor(9, 0);
  lcd.print("H: ");
  lcd.setCursor(15, 0);
  lcd.print("%");
  lcd.setCursor(0, 1);
  //lcd.print("0123456789ABCDEF");
  lcd.print("P rel:        mb");
  delay(100); 

  
  //apagamos el led 'On Board'
  pinMode(13, OUTPUT); //PIN13 como salida
  digitalWrite(13, LOW); //apagar PIN13
  //hacemos un presentacion en LCD y grabacion en EEPRON, 
  //ya que hasta llegar al loop tardamos mas de los 250ms que usamos como punto de corte para mostar y grabar
  int altitud_p1 = EEPROM.read(0);
  int altitud_p2 = EEPROM.read(1); 
  ALTITUD = float(altitud_p1*100 + altitud_p2);  
  //LEER LOS SENSORES y ACTUALIZAR VARIABLES

  HumedadREL = sensorDHT11(DHT11_PIN); //captura la informacion del sensor de humedad para tener uan referencia
                                       //ya que en la rutina de consulta hay una condicion para evitar
                                       //grances fluctuaciones de la humedad producto de errores de acceso al sensor
                                       //y la variable se inicializa en CERO

  consultarSensores();
  mostarDatosLCD_v3();

  punteroEEPROM = establecerPunteroEEPROM();  //buscamos la ubicacion de memoria libre para guardar los datos
  if (punteroEEPROM==5 AND FLAG_usar_eeprom == true){
    almacenarDatos(HumedadREL, Temperatura, PresionRelativaCotaCero);
  }
}



//*******************************************************
//            BUCLE PRINCIPAL DEL PROGRAMA
//*******************************************************

void loop()
{                
  unsigned long tiempoActual = millis();  //para controlar el refresco del LCD
  leerPuertoBT_fast();
  int ordenPC = leerPuertoSerie();
  if (ordenPC == 1){
    mostarDatosEnPuertoSerie();   //muestra datos actuales por el puerto serie
  }
  if (ordenPC == 2){
    mostarEEPROMporSERIAL();      //vuelca el contenido de la eeprom en el puerto serie del PC
  }

  //control del tiempo entre lecturas de los sensores y muestra de datos por LCD
  if(tiempoActual%2500 < 250){
    //LEER LOS SENSORES y ACTUALIZAR VARIABLES
    consultarSensores();
    mostarDatosLCD_v3();
    if (FLAG_control_MaxMin == true){
      comprobarMaxMin(); //revisar si hay un nuevo record en cuyo caso se graba en eeprom
    }

    //consultar la deficinion de variables para modificar los tiempos de grabacion en EEPROM
    if(tiempoActual%MILISEGUNDOS < 250) { //cada hora grabamos en la EEPROM los datos
      if(FLAG_usar_eeprom == true){
        almacenarDatos(HumedadREL, Temperatura, PresionRelativaCotaCero);
      }
      predecirTiempo();
    }
    delay(250);
  }
}



//#####################################################################################################
// BLOQUE DE FUNCIONES: LECTURAS (BT), TOMA DE DECISIONES, ACCESO EEPROM...
//#####################################################################################################

//========================================================
// FUNCION PARA ACCESO Y LECTURA DEL PUERTO SERIE
//========================================================

int leerPuertoSerie()
{
 /*
  * Funcion para atender ordenes a través del puerto serie
  * Lee parametros desde el puerto serie:
  * - Si el parametro es '*' se inicia la lectura de datos
  * - Si el parametro es 'x' ordena que cancele el proceso de lectura
  * (Ampliable en un futuro para mas cosas
  */

  int lectura = 'x';
  if (Serial.available() > 0) {
     // leer byte:
    lectura = Serial.read();
    if (lectura == '*'){return 1;} //indicar que mande una muestra de datos al puerto serie
    if (lectura == '+'){return 2;} //indicar que muestre le contenido almacenado en la eeprom
    else {return 0;}  //retorna un comando nulo
  }
}



//========================================================
// FUNCION PARA ACCESO Y LECTURA 'rapida' DEL PUERTO BLUETOOTH
//========================================================

void leerPuertoBT_fast()
{
/*
 * Version rapida de la funcion para leer el puerto (BT) que no necesita INTRO,
 * pero que solo responde a ordenes de un unico caracter, como es obvio
 */

  if(BT.available()) {
    char dato=BT.read();     //Leemos y asignamos los datos carácter a carácter en la variable "dato"
    CADENA[0] = dato;        //Guardamos cada "dato" en el array "cadena", 
                             //que es el que se usa para comparar con la lsita de ordenes
                             //por se compatibles con la version lenta, sin cambiar nada mas

    if(dato) {               //Si se reciben datos, tratamos de interpretar el comando
      interpretarComando();  //lo pongo como una funcion exterior por si añado muchas opciones
                             // que esta funcion siga siendo sencilla y clara
      BT.write("\r");        //Enviamos un retorno de carro de la app.
      //CADENA[0]=0;         //no necesito esta linea para borrar el buffer, se sobreescribe
    }    
  }
}



//*******************************************************
//   FUNCION PARA ACCESO Y LECTURA 'lenta' DEL PUERTO BLUETOOTH
//*******************************************************
 
void leerPuertoBT_slow()
{
 /*
  * Version basica de lectura del pueto BT La llamo 'lenta' porque es obligatorio
  * presionar intro para enviar el comando
  * se consigue de esta forma introducir comandos complejos de mas de un caracter.
  * Necesarios para introducur parametros como la altitud...
  */

  //Si hay datos disponibles en el receptor BT
  indiceBUFFER = 0;
  boolean bucle = true;
  boolean marcaHoraria;
  if (CADENA[0] == 'm'){marcaHoraria = true;}
  if (CADENA[0] == 'a'){marcaHoraria = false;}
  while (bucle == true){
    if(BT.available()) {
      char dato=BT.read(); //Leemos y asignamos los datos carácter a carácter en la variable "dato"
      if(dato == 'x') {cleanBuffer();BT.write("\n\r");return;}       //sale y reinicia la lectura del blueTooth
      if(dato == 'c') { borrarEEPROM();cleanBuffer();return;}        //borrar EEPROM e inicializar puntero
      if(dato == 'p') { borrarEEPROMparcial();cleanBuffer();return;} //borra la EEPROM ocupada e inicializar puntero

      CADENA[indiceBUFFER++] = dato; //Guardamos cada "dato" colocandolo sucesivamente en el array "cadena"
      //Cuando reciba una nueva línea (al pulsar enter en la app) entra en la función
      if(dato=='\n' AND marcaHoraria == true) {
        grabarEEPROM(punteroEEPROM,   101); //byte indentificativo de marca horaria
        grabarEEPROM(punteroEEPROM+1, int(CADENA[0])-48); //restando 48 obtenemos el valor numerico quw representa el char
        grabarEEPROM(punteroEEPROM+2, int(CADENA[1])-48);
        grabarEEPROM(punteroEEPROM+3, int(CADENA[2])-48);
        grabarEEPROM(punteroEEPROM+4, int(CADENA[3])-48);
        punteroEEPROM +=5; //actualizamos el puntero
        almacenarDatos(HumedadREL, Temperatura, PresionRelativaCotaCero);
        BT.write("\r"); //Enviamos un retorno de carro de la app. La app ya crea una línea nueva
        cleanBuffer();  //Ejecutamos la función clean() para limpiar el array
        bucle = false;
        }
      if(dato=='\n' AND marcaHoraria == false) {
        int c0= int(CADENA[0])-48;
        int c1= int(CADENA[1])-48;
        int c2= int(CADENA[2])-48;
        int c3= int(CADENA[3])-48;
        int altitud_p1 = c0*10 + c1;
        int altitud_p2 = c2*10 + c3;
        ALTITUD = altitud_p1 *100 + altitud_p2; //actualizamos la altitud que se usa para los calculos       
        grabarEEPROM(0, altitud_p1); //unidades de millar y centenas
        grabarEEPROM(1, altitud_p2); //decenas y unidades
        BT.write("\r"); //Enviamos un retorno de carro de la app. La app ya crea una línea nueva
        cleanBuffer();  //Ejecutamos la función clean() para limpiar el array
        bucle = false;
        }
      }
    }
}



//*******************************************************
//   FUNCION PARA LIMPIAR EL BUFFER DE CARACTERES DEL PUERTO BT 'slow'
//*******************************************************

void cleanBuffer()
{
 /*
  * Limpia el array que hace de buffer para el puerto BT en su version 'slow'
  */

  for (int cl=0; cl<=indiceBUFFER; cl++)
  {
    CADENA[cl]=0; 
    //cadena[cl]=0;
  }
  indiceBUFFER=0;
}



//========================================================
//   FUNCION PARA INTERPRETAR LOS COMANDOS BT
//========================================================

void interpretarComando()
{
 /*
  * --- comandos v2.0, de un solo caracter, para aplicacion inmediata sin intro---
  * interpetar los comandos recibidos por BT para actuar en consecuencia.
  * (Version mejorada respecto a la primera vez que use el BT)
  * Son comandos de un solo caracter, con lo que no se necesita dar intro para que se ejecuten
  * Algunos de los comandos conllevan la introducion de parametros. 
  * En ese caso se activa la lectura 'lenta' del puerto BT
  */

  if(strstr(CADENA,"t")!=0) { mostarDatosPorBT(); }       //muestra datos actuales en el dispositivo movil
  if(strstr(CADENA,"h")!=0) { ayudaBTserie(); }           //muestra el menu de ayuda en el dispositivo movil     
  if(strstr(CADENA,"b")!=0) { mostarEEPROMporBT(0);}      //muestra el registro de datos en el dispositivo movil
  if(strstr(CADENA,"v")!=0) { mostarEEPROMporBT(12);}     //muestra las 'n'(12) ultimas horas en el dispositivo movil
  if(strstr(CADENA,"s")!=0) { mostarEEPROMporSERIAL();}   //muestra el registro de datos en el PC
  if(strstr(CADENA,"c")!=0) { leerPuertoBT_slow();}       //buscamos parametros para la orden de borrado EEPROM
  if(strstr(CADENA,"m")!=0) { leerPuertoBT_slow();}       //Activa el proceso para introducir marca horaria
  if(strstr(CADENA,"a")!=0) { leerPuertoBT_slow();}       //Activa el proceso para introducir la altitud 
  if(strstr(CADENA,"r")!=0) { resetMaxMin(); }            //Reset de los valores Max. y Min.
  if(strstr(CADENA,"e")!=0) {                             //'togle'  Habilita/Desabilita almacenamiento eeprom 
    FLAG_usar_eeprom  = !FLAG_usar_eeprom ; //'togle'
    BT.write("\r");
    if (FLAG_usar_eeprom == true){BT.write("EEPROM: ON");}
    if (FLAG_usar_eeprom == false){BT.write("EEPROM: OFF");}
    BT.write("\n\n\r");                 //Enviamos 2 lineas nuevas y un retorno de carro. 
  }
  if(strstr(CADENA,"l")!=0) {                             //'togle'  Apagar/Encender la luz del LCD
    FLAG_luz_lcd = !FLAG_luz_lcd; //'togle'
    lcd.setBacklight(FLAG_luz_lcd);
    BT.write("\r");
    if (FLAG_luz_lcd == true){BT.write("LUZ: ON");}
    if (FLAG_luz_lcd == false){BT.write("LUZ: OFF");}
    BT.write("\n\n\r");                 //Enviamos 2 lineas nuevas y un retorno de carro. 
  }
}


//#####################################################################################################


//========================================================
//  FUNCION PARA MOSTAR TEMPERATURA/HUMEDAD POR  BT
//========================================================
 
void mostarDatosPorBT()
{
 /*
  * Mustra datos actuales (realiza una consulta actual) por BT
  */

  //mostar cabecera.... (BT) 
  //para que se vea algo mientras se accede a los sensores y se hacen los calculos
  BT.write("\n\n\n\r"); //Enviamos linea nueva y un retorno de carro.
  BT.write("obteniendo lecturas...\n\n\r");
  lineaSeparadoraBT();
   
  //LEER LOS SENSORES y ACTUALIZAR VARIABLES
  consultarSensores();

  //FORMATEO de datos y ENVIO al puerto (BT)
  String cadena;       //declaramos una cadena que nos servira para las conversiones de temperatura y humedad
  char arrayTexto[8];  //declaramos un array de caracteres para las conversiones de datos

  if (FLAG_luz_lcd == true){BT.write("LUZ: ON");}
  if (FLAG_luz_lcd == false){BT.write("LUZ: OFF");}
  BT.write("\n\r");                    //Enviamos linea nueva y un retorno de carro. 

  cadena = String(TempeMAXIMA);        //convertimos el valor de la temperatura en una cadena
  cadena.toCharArray(arrayTexto,5);    //pasamos la cadena temperatura al array
  BT.write("t MAXIMA: ");
  BT.write(arrayTexto);                //mostramos la temperatura MAXIMA
  BT.write(" C\n\r");                  //añadimos caracteres para mejor visializacion  


  cadena = String(TempeMINIMA);        //convertimos el valor de la temperatura en una cadena
  cadena.toCharArray(arrayTexto,5);    //pasamos la cadena temperatura al array
  BT.write("t MIMIMA: ");
  BT.write(arrayTexto);                //mostramos la temperatura MINIMA
  BT.write(" C\n\r");                  //añadimos caracteres para mejor visializacion  

  cadena = String(HumedadREL);         //convertimos el valor de la humedad en una cadena
  cadena.toCharArray(arrayTexto,3);    //pasamos la cadena humedad al array

  BT.write("humedad: ");
  BT.write(arrayTexto);                //mostramos la humedad
  BT.write(".0 %\n\r"); 
 

  cadena = String(Temperatura);        //convertimos el valor de la temperatura en una cadena
  cadena.toCharArray(arrayTexto,5);    //pasamos la cadena temperatura al array

  BT.write("temperatura: ");
  BT.write(arrayTexto);                //mostramos la temperatura
  BT.write(" C\n\r");                  //añadimos caracteres para mejor visializacion  

  cadena = String(PresionABS);         //convertimos el valor de la presion en una cadena
  cadena.toCharArray(arrayTexto,8);    //pasamos la cadena presion al array

  BT.write("\n\r* PRESION ATMOSFERICA *\n\r");
  
  BT.write("Presion ABS: ");
  BT.write(arrayTexto);                //mostramos la presion 
  BT.write(" mBar\n\r");               //añadimos caracteres para mejor visializacion 

  BT.write("P.Relativa (ALTITUD, ");
  cadena = String(ALTITUD);            //convertimos el valor de la ALTITUD en una cadena
  cadena.toCharArray(arrayTexto,8);    //pasamos la cadena presion al array
  BT.write(arrayTexto);                //mostramos la altitud 
  BT.write(" metros): ");

  cadena = String(PresionRelativaCotaCero); //convertimos el valor de la presion en una cadena
  cadena.toCharArray(arrayTexto,8);         //pasamos la cadena presion al array

  BT.write(arrayTexto);                 //mostramos la presion 
  BT.write(" mBar\n\r");                //añadimos caracteres para mejor visializacion 

  cadena = String(TemperaturaDH11);     //convertimos el valor de la temperatura en una cadena
  cadena.toCharArray(arrayTexto,5);     //pasamos la cadena temperatura al array

  BT.write("temperatura DH11 (sin calibrar): ");
  BT.write(arrayTexto);                 //mostramos la temperatura
  BT.write(" C\n\r");                   //añadimos caracteres para mejor visializacion  
  lineaSeparadoraBT();
}


//========================================================
//  FUNCION PARA MOSTRAR DATOS POR SERIAL EN TIEMPO REAL
//========================================================

void mostarDatosEnPuertoSerie()
{
 /*
  * Mustra datos actuales (realiza una consulta actual) por el puerto serie
  */
 
  //LEER LOS SENSORES y ACTUALIZAR VARIABLES
  consultarSensores();
  
  //FORMATEO de datos y ENVIO al puerto SERIE
  formateoDatosSerie();

  //opcionalmetne podemos refrescamos el LCD para mostrar los datos actuales que van hacia Serial
  mostarDatosLCD_v3(); 
}


//========================================================
//  FUNCION PARA MOSTRAR EL MENU DE AYUDA POR BLUETOOTH
//========================================================

void ayudaBTserie()
{
 /*
  * Mostrar un menu de ayuda por BT con los comandos disponibles y una breve explicacion
  */

  BT.write("\n\n\r");
  lineaSeparadoraBT();
  BT.write("Lista de comandos:\n\r");
  lineaSeparadoraBT();

  BT.write(" t:  Datos actuales\n\r");   
  BT.write(" b:  EEPROM por BT\n\r"); 
  BT.write(" v:  Ultimos Datos\n\r");
  BT.write(" s:  EEPROM por SERIAL\n\r");
  BT.write(" r:  Reset Max Min\n\r");
  BT.write(" e:  EEPROM\n\r");   
  BT.write(" l:  Luz\n\r");     
  BT.write(" mxxxx: MARCA HORARIA\n\r"); 
  BT.write(" axxxx: ALTITUD\n\r"); 
  BT.write(" m,a,c + x: CANCELAR entrada\n\r"); 
  BT.write(" cc: Borrado COMPLETO\n\r"); 
  BT.write(" cp: Borrado PARCIAL\n\r");  
  BT.write(" h:  AYUDA\n\r"); 

  lineaSeparadoraBT();
}



//#####################################################################################################


//========================================================
//  HIGROMETRO, usando sensor DHT11  SIN LIBRERIA ESTANDAR 
//  (no se devuelven datos de temperatura, pero se generan)
//========================================================
 
byte sensorDHT11(int pin)
{
 /*
  * Acceso al DTH11 sin libreria (me daba error y no conseguia hacerla funcionar)
  * Asi que con la ayuda de las hojas de caracteristicas del sensor he creado una funcion 
  * para leer a pelo los datos generados.
  * Como no es muy preciso (el sensor) las temperaturas las desprecio (auqnue si se generan)
  * y uso las que obtengo con el barometro
  */

  float timeout = 35; //tiempo maximo en intentos de acceso al sensor o se retorna error
  //secuencia para activacion del DHT11
  pinMode(pin, OUTPUT);  //configurar el pin como salida
  digitalWrite(pin, LOW);
  delay(20);  //23 mantener en (0)val menos 18 ms
  digitalWrite(pin, HIGH);
  delayMicroseconds(30);  //mantener en (1) entre 20 y 40 us

  //escuchar respuesta del sensor
  pinMode(pin, INPUT);  //configuramos la palilla como entrada
  //Respuesta cuando esta listo para transmitir
  float inicio = millis();
  while (digitalRead(pin) == 0){
    if( millis() > inicio + timeout){
      return 0; //imposible contactar con el sensor, devolvemos 0 como valor de humedad
    }
  }
  while (digitalRead(pin) == 1){
    if( millis() > inicio + timeout){
      return 0; //imposible contactar con el sensor, devolvemos 0 como valor de humedad
    }
  }
  //una vez la respuesta es OK (secuencia 0,1), comenzamos la escucha de los 40 bits de datos que nos da el sensor

  byte lecturaSensor[5] = {0,0,0,0,0};  //reserva de memoria para 5 bytes (40 bits)
  //bucle para leer los 5 bytes
  for (int i=0; i<5; i++){
    int tempByte = 0;  //ponemos a 0 la variable que sucesivamente ira conteniendo cada byte
    //bucle para la lectura de los 8 bits de cada byte
    for (int j=0; j<8; j++){
      while (digitalRead(pin) == 0){
        if( millis() > inicio + timeout){
          return lecturaSensor[0];  //error durante la comunicacion. Devolvemos el byte[0] por si esta disponible
        }
      }
      delayMicroseconds(40);
      
      if (digitalRead(pin) == 1){
        tempByte |= 1<<(7-j);  //colocamos cada bit 1 a su lugar correspondiente dentro del byte
      }
      while (digitalRead(pin) == 1){
        if( millis() > inicio + timeout){
          return lecturaSensor[0]; //error durante la comunicacion. Devolvemos el byte[0] por si esta disponible
        }
      }
    }
    lecturaSensor[i] = tempByte;
  }
  //comprobamos el checksum de los datos obtenidos y solo si no hay errores modificamos las lecturas anteriores.
  if (lecturaSensor[4] == ((lecturaSensor[0] + lecturaSensor[1] + lecturaSensor[2] + lecturaSensor[3]) & 0xFF)){
    //Serial.println(lecturaSensor[0]); //humedad
    //Serial.println(lecturaSensor[1]); //nulo
    //Serial.println(lecturaSensor[2]); //temperatura
    //Serial.println(lecturaSensor[3]); //nulo
    //Serial.println(lecturaSensor[4]); //checksum
    TemperaturaDH11 = lecturaSensor[2];
    //HumedadREL = lecturaSensor[0]; //la humedad es el primer byte devuelto por el sensor
    return lecturaSensor[0]; 
  } 
}



//========================================================
//  BAROMETRO, usando sensor BMP180
//========================================================

void leerDatosSensorBMP180()
{
 /*
  * Acceso al barometro y actualizacion de valores actuales de presion y temperatura
  * Se pueden comentar todas las lineas Serial.rpint si se desea ahorrar algo de memoria
  * ya que solo si estamos conectados en serie reportamos el error.
  * Dejo para mas adelante alguna bandera de error que pueda ser notificada por BT
  */
  char estado;
  double T,P,p0,a;  //variables temporales para calculos internos dentre de la funcion

  /* Segun la libreria, primero se debe hacer una lectura de la temperatura para poder hacer una medida de presion.
  Se inicia el proceso de lectura de la temperatura... si se realiza sin errores, 
  se devuelve un numero de (ms) de espera, si no, la funcion devuelve 0 y devolvemos error
  */
  
  estado = sensorBMP180.startTemperature();
  if (estado != 0)
  {
    // pausa para que se complete la medicion en el sensor.
    delay(estado);

    // Obtencion de la medida de temperatura que se almacena en T:
    // Si la lectura el correcta la funcion devuelve 1, si se producen errores, devuelve 0.

    estado = sensorBMP180.getTemperature(T);
    if (estado != 0)
    {
      Temperatura = T;  //Asignacion a variable global
      
      /* Se inicia el proceso de lectura de la presion.
         El parametro para la resolucion de muestreo varia de 0 a 3 (a mas resolucion, mayor tiempo necesario).
         Si se realiza sin errores, se devuelve un numero de (ms) de espera, si no, la funcion devuelve 0.
      */

      estado = sensorBMP180.startPressure(3);
      if (estado != 0)
      { 
        delay(estado); // pausa para que se complete la medicion en el sensor.
        
        // Obtencion de la medida de Presion que se almacena en P:
        // Si la lectura el correcta la funcion devuelve 1, si se producen errores, devuelve 0.
        estado = sensorBMP180.getPressure(P,T);

        if (estado != 0)
        {
          PresionABS = P;  //Asignacion a variable global

          /* 
          El sensor devuelve presion absoluta. Para compensar el efecto de la altitud
          usamos la funcion interna de la libreria del sensor llamada: 'sealevel'
          P = presion absoluta en (mb) y ALTITUD = la altitud del punto en que estomos (m).
          Resultado: p0 = presion compensada a niveldel mar en (mb)
          */

          p0 = sensorBMP180.sealevel(P,ALTITUD); // 407 metros (SORBAS)
          PresionRelativaCotaCero= p0;  //Asignacion a variable global
        }
        else Serial.println("err P\n"); //error de presion en las lecturas/obtencion de datos
      }
      else Serial.println("err P\n");
    }
    else Serial.println("err T\n");//error de temperatura en las lecturas/obtencion de datos
  }
  else Serial.println("err T\n");
}



//========================================================
//  FUNCION PARA CONSULTAR SENSORES Y ACTUALIZAR VARIABLES
//========================================================
 
void consultarSensores()
{
 /*
  * Funcion para la consulta de sensores y actualizacion 
  * de las variables globales que contienen los datos
  */

  //LEER LOS SENSORES
  float HumedadOld = HumedadREL;  
  HumedadREL = sensorDHT11(DHT11_PIN); //captura la informacion del sensor de humedad
  
  //para evitar errores raros del sensor, que a veces me muestra valores oscilantes... 
  //creo que son problemas unicamente de mi DTH11, (pero no dispongo de otro en este momento)
  if (abs(abs(HumedadOld) - abs(HumedadREL)) > 10){ HumedadREL = HumedadOld;}  

  //captura los datos del sensor BMP180 (Temperatura, PresionABS)
  leerDatosSensorBMP180(); 
  if (Temperatura > TempeMAXIMA){TempeMAXIMA = Temperatura;}
  if (Temperatura < TempeMINIMA){TempeMINIMA = Temperatura;}
}


//========================================================
//  FUNCION PARA MOSTAR TEMPERATURA/HUMEDAD POR  LCD
//========================================================
 
void mostarDatosLCD_v3()
{
 /*
  * Funcion simplificada (respecto a la estacion meteo simple) para mostar datos en el LCD
  */

  lcd.setCursor(3, 0);
  lcd.print(Temperatura,1);

  lcd.setCursor(12, 0);
  lcd.print(HumedadREL,0);
  lcd.print(" ");
  
  lcd.setCursor(7, 1);
  if (PresionRelativaCotaCero <1000){
    lcd.print("P rel:        mb");
    lcd.setCursor(8, 1);
  }
    
  lcd.print(PresionRelativaCotaCero,1);
}



//========================================================
// CONVERTIR los datos de presion humedad y temperatura
// para ser grabados en la EEPROM
//========================================================
 
void almacenarDatos(byte humedad, float temperatura, float presion)
{
 /*
  * Dado que la epprom solo atiende a bytes...
  * Funcion para restruccurar la informacion de presion humedad y marcas horarias
  * de manera que pueda ser almacenada como bytes
  * Creo que hay funcionalidades para manejo de datos float y demas en alguna libreria eeprom
  * pero lo hago manualmente.
  */

  //formateo de los datos
  byte paqueteDatos[5] = {0,0,0,0,0};  //reserva de 5 bytes que contendran  humedad, temperatura y presion

  paqueteDatos[0] = humedad; //la humedad en el byte 0 del grupo
  int t= temperatura * 10;
  paqueteDatos[1] = t/255; //el byte mas significativo de la temepratura en byte1
  paqueteDatos[2] = t%255; //el byte menos significativo de la temepratura en byte2

  int p= presion * 10;
  paqueteDatos[3] = p/255; //el byte mas significativo de la presion en byte3
  paqueteDatos[4] = p%255;//el byte menos significativo de la presion en byte4
  
   //debemos mantener una variable global que sea el 'puntero' indicador de donde vamos escribiendo en la EEPROM
  for (int i=0; i<5; i++){
    grabarEEPROM(punteroEEPROM+i, paqueteDatos[i]);
  }
  punteroEEPROM +=5; //actualizamos el puntero
}



//========================================================
// LEER DE LA EEPROM Y CONVERTIR para mostrar por BT
//========================================================
 
void mostarEEPROMporBT(int numeroRegistros)
{
 /*
  * Funcion para mostar los datos almacenedos en la EEPROM (si los hay)
  * por el puerto serie BT
  */

  //formateo de los datos para mostrarlos por BT
  byte paqueteDatos[5] = {0,0,0,0,0};  //reserva de 5 bytes que contendrán  humedad, temperatura y presion
  String cadena; //declaramos una cadena que nos servira para las conversiones de temperatura y humedad
  char arrayTexto[8];   //declaramos un array de caracteres para las conversiones de datos 

  if (numeroRegistros == 0){
    cadena = String(int((punteroEEPROM/5)) - 1);      //convertimos el valor en una cadena 
  }
  else{
    cadena = String(numeroRegistros);  
  }
  
  cadena.toCharArray(arrayTexto,4); //pasamos la cadena humedad al array
  //mostar cabecera.... (BT)
  //para que se vea algo mientras se accede a los sensores y se hacen los calculos
  BT.write("\n\n\rMOSTRANDO ");BT.write(arrayTexto);BT.write(" REGISTROS...\n\r");

  BT.write("P.Relativa (ALTITUD, ");
  cadena = String(ALTITUD);            //convertimos el valor de la ALTITUD en una cadena
  cadena.toCharArray(arrayTexto,8);    //pasamos la cadena presion al array
  BT.write(arrayTexto);                //mostramos la altitud 
  BT.write(" metros):");
  lineaSeparadoraBT(); 
  
  int posicionInicio = 5;  //asuminos el parametro es CERO y que lo queremos leer todo
                           //las posiciones de 0 a 4 estan reservadas
  
  if (numeroRegistros > 0){ //si el parametro es un valor distinto de cero
    if (punteroEEPROM > numeroRegistros*5){
      posicionInicio = punteroEEPROM - (numeroRegistros*5);  //mostramos solo las 'numeroRegistros' ultimas posiciones
    }   
  }
  
  for (int i=posicionInicio; i<punteroEEPROM; i+=5){
    for (int j=0; j<5; j++){
      int punteroLectura = i+j;
      paqueteDatos[j] = EEPROM.read(punteroLectura);  //recuperar dato desde la posicion indicada
      //Serial.print(paqueteDatos[j]);
    }

    if (paqueteDatos[0] == 101){ //byte indentificador de una marca horaria  
      BT.write(" -- MARCA DE TIEMPO --\n\r"); 
      cadena = String(paqueteDatos[1]);    //convertimos el valor den una cadena
      cadena.toCharArray(arrayTexto,2);    //pasamos la cadena al array
      BT.write(arrayTexto);                //mostramos
      cadena = String(paqueteDatos[2]);    //convertimos el valor den una cadena
      cadena.toCharArray(arrayTexto,2);    //pasamos la cadena al array
      BT.write(arrayTexto);                //mostramos
      BT.write(" : "); 

      cadena = String(paqueteDatos[3]);    //convertimos el valor en una cadena
      cadena.toCharArray(arrayTexto,2);    //pasamos la cadena al array
      BT.write(arrayTexto);                //mostramos
      cadena = String(paqueteDatos[4]);    //convertimos el valor en una cadena
      cadena.toCharArray(arrayTexto,2);    //pasamos la cadena al array
      BT.write(arrayTexto);                //mostramos
      BT.write("\n\r"); 
    }


    if (paqueteDatos[0] != 101){        //si no es marca horaria, son datos utiles
      byte humedad = paqueteDatos[0];   //la humedad en el byte 0 del grupo
      float temperatura = float((paqueteDatos[1] * 255 + paqueteDatos[2]))/10;  
      float presion = float((paqueteDatos[3] * 255 + paqueteDatos[4])) /10;    

      //FORMATEO de datos y ENVIO al puerto (BT)  
      cadena = String(humedad);         //convertimos el valor de la humedad en una cadena
      cadena.toCharArray(arrayTexto,3); //pasamos la cadena humedad al array
    
      BT.write("humedad: ");
      BT.write(arrayTexto);             //mostramos la humedad
      BT.write(".0 %\n\r");             //añadimos caracteres para mejor visializacion

    
      cadena = String(temperatura);     //convertimos el valor de la temperatura en una cadena
      cadena.toCharArray(arrayTexto,5); //pasamos la cadena temperatura al array
    
      BT.write("temperatura: ");
      BT.write(arrayTexto);             //mostramos la temperatura
      BT.write(" C\n\r");               //añadimos caracteres para mejor visializacion  
   
      cadena = String(presion);         //convertimos el valor de la presion en una cadena
      cadena.toCharArray(arrayTexto,7); //pasamos la cadena presion al array
    
      BT.write("presion: ");
      BT.write(arrayTexto);             //mostramos la presion 
      BT.write(" mBar");                //añadimos caracteres para mejor visializacion
      lineaSeparadoraBT();
    }
  }
}



//========================================================
// LEER DE LA EEPROM Y CONVERTIR para mostrar por SERIE
//========================================================
 
void mostarEEPROMporSERIAL()
{
 /*
  * Funcion para mostar los datos almacenedos en la EEPROM (si los hay)
  * por el puerto serie para PC
  */

  //formateo de los datos para mostrarlos por puerto SERIA y poder ser recogidos en el PC
  byte paqueteDatos[5] = {0,0,0,0,0};  //reserva de 5 bytes que contendran  humedad, temperatura y presion
  Serial.println("===============");
  Serial.println(" DATOS EEPROM: ");
  Serial.println("===============");
  Serial.println();
  for (int i=5; i<punteroEEPROM; i+=5){ //1024 en lugar de punteroEEPROM si se quiere sacar todo el contenido
    for (int j=0; j<5; j++){
      int punteroLectura = i+j;
      paqueteDatos[j] = EEPROM.read(punteroLectura);  //recuperar dato desde la posicion indicada
    }
    byte humedad = paqueteDatos[0]; //la humedad en el byte 0 del grupo
    if (humedad == 255){  //indicador de un bloque de memoria libre. Por tanto, salimos
      break;
    }      
    if (humedad == 101){  //indicador de que este bloque contiene unamarca horaria y no humedad
      Serial.print("HORA ");
      Serial.print(paqueteDatos[1]);Serial.print(paqueteDatos[2]); 
      Serial.print(":");
      Serial.print(paqueteDatos[3]);Serial.println(paqueteDatos[4]);
      continue; //para no procesar los siguetnes bytes y saltar al siguiente bloque
    }
    float temperatura = float((paqueteDatos[1] * 255 + paqueteDatos[2]))/10;  
    float presion = float((paqueteDatos[3] * 255 + paqueteDatos[4])) /10;    

    //FORMATEO de datos y ENVIO al puerto SERIE
    formateoDatosSerie();
  }
}

//========================================================
// FORMATEO de datos y ENVIO al puerto SERIE
//========================================================

void formateoDatosSerie()
{
  Serial.print("Humedad: ");
  Serial.println(HumedadREL);                   //mostramos la humedad
  Serial.print("Temp: ");
  Serial.println(Temperatura);                  //mostramos la temepratura del bmp180
  Serial.print("P. Relativa: ");
  Serial.println(PresionRelativaCotaCero);      //mostramos la presion
  Serial.println("temp DH11");              //mostramos la temepratura del DTH11  
  Serial.println(TemperaturaDH11);
}



//========================================================
// ESTABLECER EL PUNTERO DE ESCRITURA EN EEPROM
//========================================================
 
int establecerPunteroEEPROM()
{
 /*
  * Funcion para establecer donde debe empezarse la escritura. 
  * Por si hay cortes de suministro y se reinicia el sistema, 
  * Para no sobreescibir valores ya guardados  busca el primer bloque de 5 bytes libres
  */
  
  if (FLAG_eeprom_disponible == false){
    return 1023;
  }
  int direccion; //inicio en la posicion 5. 
                 //Reservamos las 5 primeras posiciones. (0,1) para la altitud y (2,3,4) para las temp Max y Min
  for (direccion=5; direccion<1020; direccion+=5){
    int bloqueLibre = 0;
    for (int j=0; j<5; j++){
      bloqueLibre += leerEEPROM(direccion + j);
    }
    if (bloqueLibre == 1275){ //la suma de 5 posiciones (5*255=1275)
      //Serial.print("Puntero establecido en la direccion "); Serial.println(direccion);
      return direccion;  //establecemos el valor del puntero
    }
  }
  FLAG_eeprom_disponible = false;
  //return 5; //cambiar la linea anterior por esta  si queremos sobreescribir eeprom
}



//========================================================
// BORRADO COMPLETO DE LA EEPROM 
//========================================================

void borrarEEPROM()
{
 /*
  * Funcion para borrar la EEPROM de forma completa y sin contenplaciones
  * Pero si se respetan las 5 primeras posiciones que contienen la altitud y los max y min
  */
 
  for(int direccion=5; direccion < 1024; direccion++){
  grabarEEPROM(direccion, 255);
  }
  punteroEEPROM = 5;  //reseteamos el valor del puntero
  //Serial.print("Borrado completo de le memoria EEPROM");
  BT.write("\n\n\rBorrado total de EEPROM\n\n\r");
}



//========================================================
// BORRADO PARCIAL DE LA EEPROM 
//========================================================

void borrarEEPROMparcial()
{
 /*
  * Funcion para borrar solo las posiciones de memoria que esten usadas hasta ese momento,
  * preservando asi la memoria no usada de excesos de uso innecesarios.
  * Se respetan las 5 primeras posiciones que contienen la altitud y los max y min
  */
 
  int direccion;
  for(direccion=5; direccion < 1024; direccion++){
    if (leerEEPROM(direccion) == 255){  // 255 es un valor que nunca es generado por los datos del cliente                                   
      break;                            // con lo que si aparece, es que la posicion esta sin usar
    }
    grabarEEPROM(direccion, 255);       //dato 'nulo' que indicará que una posicion está disponible
  }
  punteroEEPROM = 5;  //reseteamos el valor del puntero (se reservan las 5 primeras posiciones)
  //Serial.print("Borradas "); Serial.print(direccion); Serial.println("posiciones de memoria EEPROM");

  String cadena; //declaramos una cadena que nos servira para las conversiones de temperatura y humedad
  char arrayTexto[8];   //declaramos un array de caracteres para las conversiones de datos 

  cadena = String(direccion-5);      //convertimos el numero de posiciones en una cadena
  cadena.toCharArray(arrayTexto,5);  //pasamos la cadena al array

  BT.write("\n\n\rBorradas ");BT.write(arrayTexto);
  BT.write(" posiciones de EEPROM\n\n\r");
}



//========================================================
// GRABAR DATOS EN LA EEPROM (direccion, valor)  
//========================================================

void grabarEEPROM(int direccion, byte dato)
{
 /*
  * Escritura en una posicion de memoria EEPROM indicada en el parametro
  */

  if(direccion >=1020 OR FLAG_usar_eeprom == false){ //comprobar si se permite la escritura en eeprom
    return;  
  }
  EEPROM.write(direccion, dato); //grabar un dato en una direccion dada de la memoria EEPROM 
}



//========================================================
// LEER DATOS DE LA EEPROM  (direccion)
//========================================================

byte leerEEPROM(int direccion) 
{
 /*
  * lectura de la posicion de memoria EEPROM indicada en el parametro
  */
   
  byte dato = EEPROM.read(direccion);  //recuperar dato desde la posicion indicada
  return dato;
}



//========================================================
// CREAR UNA LINEA  DE SEPARACION CON GUIONES
//========================================================
void lineaSeparadoraBT()

{
 /* funcion para tratar de ahorrar unos cuantos bytes de ram
  * escribiendo el separador con la llamada a una funcion 
  * y no con una linea de codigo en los sitios donde se emplea 
  */

  BT.write("\n\r------------------\n\r");
  return;
}


//========================================================
// ALMACENAR temperaturas MAX y MIN
//========================================================

void comprobarMaxMin()
{
  float TempeMAXIMAnew = TempeMAXIMA; //salvaguardamos los max y min actuales
  float TempeMINIMAnew = TempeMINIMA;
  
  leerEEPROMtempeMaxMin(); //recuperamos los datos de la eeprom

  if (TempeMAXIMA > TempeMAXIMAnew AND TempeMINIMA < TempeMINIMAnew){
    return; //si los almacenados siguen siendo record, abandomanos 
  }
  // si se supera el maximo o el minimo, se graba en la eeprom
  TempeMAXIMA = TempeMAXIMAnew; //salvaguardamos los max y min actuales
  TempeMINIMA = TempeMINIMAnew;
  alamcenarMaxMinEEPROM();
}



//========================================================
// ALMACENAR temperaturas MAX y MIN
//========================================================

void alamcenarMaxMinEEPROM()
{
  int temp;
  int restos;  
  temp = int(TempeMAXIMA * 10);
  EEPROM.write(2, (temp/10));
  restos = (temp%10) * 10;
  temp= int(TempeMINIMA * 10);
  EEPROM.write(3, (temp/10));
  restos += (temp%10);
  EEPROM.write(4, restos);
}



//========================================================
// LEER DE EEPROM temperaturas MAX y MIN
//========================================================

void leerEEPROMtempeMaxMin()
{
  float tmax = EEPROM.read(2);
  float tmin = EEPROM.read(3);
  int restos = EEPROM.read(4);
  tmin += (restos % 10)/10.0;
  tmax += ((restos-(restos % 10)) % 100)/100.0;
  TempeMAXIMA = tmax;
  TempeMINIMA = tmin;
}



//========================================================
// RESET temperaturas MAX y MIN
//========================================================

void resetMaxMin()
{
  // si se supera el maximo o el minimo, se graba en la eeprom
  TempeMAXIMA = 0; //salvaguardamos los max y min actuales
  TempeMINIMA = 99;
  alamcenarMaxMinEEPROM();
}



//========================================================
// 'PREDECIR' EL TIEMPO USANDO LA PRESION ATMOSFERICA
//========================================================

void predecirTiempo()
{
  //desplazamos las lecturas de derecha a izquierda para conservar las ultimas
  for (int i=0;i<regMAX;i++){ 
    registroPresion[i] = registroPresion[i+1];
  }
  registroPresion[regMAX] = PresionRelativaCotaCero * 10;   //multiplicamos por 10 para salvar un decimal
                                                            //pero usando un int en vez de un float y ahorrar memoria

  //solo si tenemos todos los registros
  if (registroPresion[0]>0){ 
    float presionMedia = 0.0;  // parte(1/2) del calculo de la presion media
    for (int i=0;i<=regMAX;i++){ 
      presionMedia += registroPresion[i];  
    }
    presionMedia /= ((regMAX + 1) * 10.0);  // parte (2/2) del calculo de la presion media
                                            // recordar que los valores almacenados estan multiplicados por 10  
    
    
//    Serial.print("presionMedia: ");Serial.println(presionMedia);    // --> DEBUG
    //incremento de Presion que se produce durante el periodo en estudio (n muestas)
    int tendenciaPresion;
//    tendenciaPresion = registroPresion[regMAX] - registroPresion[0]; //calculada entre extremos de las muestras
    tendenciaPresion = registroPresion[regMAX]- presionMedia*10; //calculada respecto a la media
    //Serial.print("tendencia: ");Serial.println(tendenciaPresion);   // --> DEBUG
    char* estadoMeteo = "";
    char* evolucionnMeteo = "";
    if(PresionRelativaCotaCero <= 1015){
      estadoMeteo = "B ";  //borrasca
    }
    else{
      estadoMeteo = "A ";  //anticiclon
    }
    
    if(tendenciaPresion <= -30){
      evolucionnMeteo = "--  ";
    }
    if(tendenciaPresion > -30 AND tendenciaPresion <= -17){
      evolucionnMeteo = "-   ";
    }
    if(tendenciaPresion > -17 AND tendenciaPresion < 0){
      evolucionnMeteo = "x   ";
    }
    if(tendenciaPresion >= 0 AND tendenciaPresion <= 10){
      evolucionnMeteo = "=   ";
    }
    if(tendenciaPresion > 10){
      evolucionnMeteo = "+   ";
    }
    //Serial.print("prevision: ");Serial.print(estadoMeteo);Serial.println(evolucionnMeteo);  // --> DEBUG
    lcd.setCursor(0, 1);
    lcd.print(estadoMeteo);lcd.print(evolucionnMeteo);
  }  
}

//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************
