/*
  HenHouse V1.0
  LE MELLEC Julien 22/11/2023

  board:
    - RTC
    - (Aborted) SD Card
    - DC motor 12V
    - 2 limit switches
    - 1 Error LED
    - Photo resistor
    - ACD input, fuel gauge
    - 1 BP open/close door

  Fonctionnalités:
   - Boucle while, sortie si Vbat < 9,5V
   - Si BpPorte Down => OpenClose Door
   - Toutes les 5 minutes, faire un get de l'heure
     - Si 00:07:00 < h < 00:07:05 && porte fermée => ouvrir porte
     - Si 00:19:30 < h < 00:19:35 && porte ouverte => fermer porte
     - Récupérer la valeur de la photo résistance
     - Récupérer la valeur Vbat
     - (Aborted) Inscrire la valeur de la PhotoRes, Vbat et l'heure dans la carte SD
   - Si doorError = 1 => toogle ErrorLed

  Fonction OpenClose door:
   - Variables de la classe:
    - Door error
    - Actual door state
   - Arguments:
    - Command, can be (open, close, invert)

*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>//https://www.arduino.cc/en/reference/wire
#include <DS3231.h>//https://github.com/NorthernWidget/DS3231
#include <time.h>
#include <math.h>
#include <avr/wdt.h>

// Coordonnées pour Vannes (Latitude et Longitude)
#define LATITUDE 47.6582
#define LONGITUDE -2.7608
#define TIMEZONE 1  // UTC+1 pour CET, ajustez pour l'heure d'été si nécessaire

// Constantes photo résistance
#define RES_DIV_PHOTO       47200
#define R_PHOTO_PIN         A1
#define V_ALIM              5

// Constantes limit switchs
#define LIMIT_SW_DOWN_PIN   2
#define LIMIT_SW_UP_PIN     3

// Constantes fuel gauge
#define FUELGAUGE_PIN       A2

// Constante BP nav
#define MULTI_BP_PIN        A0

// Constante Led de témoins d'erreur
#define ERROR_LED_PIN       7
#define WARNING_LED_PIN     8

// Constantes DC motor
#define BORNE_ENA           6 
#define BORNE_IN1           5
#define BORNE_IN2           4
#define MOTOR_SPEED         255 //between 0 & 255
#define TIMEOUT_MOVE_UP     25
#define TIMEOUT_MOVE_DOWN   12

// Déclaration de la sortie de commande du mosfet qui alimente l'excitateur
#define MOSFET_EXCITATEUR_PIN  17

// Déclare la fonction de réinitialisation à l'adresse 0
void(* resetFunc) (void) = 0; 

// Variables DC motor
char motorState = 'U'; // M=Monter, D=Descendre, 'O'=Ouvrir, 'F'=Fermer 'H'=Haut, 'B'=Bas, U=Unknown, 'E'=Erreur
char previousMotorAction = 'U'; // M=Monter, D=Descendre, U=Unknown
short startActionTime = 0;
void manageDcMotor();

//RTC Variables
DS3231 Clock;
byte year;
byte month;
byte day;
byte hour;
byte minute;
byte second;
bool Century = false;
bool h12 ;
bool PM ;
void init_RTC();
void readRTC(bool log=false);

// Heure lever et coucher du soleil
int sunriseHour;
int sunriseMinute;
int sunsetHour;
int sunsetMinute;

// Photo res var
int valeurDeLentreeAnalogique;          // Contiendra la valeur lue sur l'entrée analogique ; pour rappel, il s'agit d'une valeur 10 bits (0..1023)
float tensionDeLentreeAnalogique;       // Contiendra la valeur de calcul de tension, exprimée en volt, à partir de la "valeurDeLentreeAnalogique"
long valeurOhmiqueDeLaLDR;              // Contiendra la valeur ohmique calculée de la photorésistance, à partir de la "tensionDeLentreeAnalogique"
void getPhotoResVal();

// Timer var
byte varCompteur = 0; // La variable compteur

// Error managment var
byte errorState = 0;
byte onOffErrorLedState = 0;
byte onOffWarningLedState = 0;

// Multi BP managing
byte bp_first_read_ok = 0;
void multiBpManaging();

void setup_pinMode();
void setup_timer();

void setup () {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  init_RTC();
  setup_pinMode();
  setup_timer();
  
  // Initialisation de l'heure de lever et coucher du soleil
  calculateSunriseSunset(2026, month, day, sunriseHour, sunriseMinute, sunsetHour, sunsetMinute);

  Serial.println("Démarrage avec Watchdog Timer avec un délais de 8 secondes.");
  wdt_enable(WDTO_4S);

  // Stopper le monteur si éventuellement il tourne encore
  stopMotor();

  // allumage de la clutore
  digitalWrite(MOSFET_EXCITATEUR_PIN, HIGH);

  // Remise à zéro du bit d'erreur de l'oscillateur du DS3231
  clearOscillatorStopFlag();
}

bool readOscillatorStopFlag()
{
    Wire.beginTransmission(0x68);   // Adresse I2C du DS3231
    Wire.write(0x0F);               // Registre Status
    Wire.endTransmission();

    Wire.requestFrom(0x68, 1);

    if (Wire.available()) {
        byte status = Wire.read();
        return bitRead(status, 7);  // Bit 7 = OSF
    }

    return false;
}

void clearOscillatorStopFlag()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.endTransmission();

    Wire.requestFrom(0x68, 1);

    if (Wire.available()) {
        byte status = Wire.read();
        status &= ~(1 << 7);   // Efface le bit OSF

        Wire.beginTransmission(0x68);
        Wire.write(0x0F);
        Wire.write(status);
        Wire.endTransmission();
    }
}

void timerFunction2s() {
  readRTC(false);
  Serial.print(year, DEC);
  Serial.print("-");
  Serial.print(month, DEC);
  Serial.print("-");
  Serial.print(day, DEC);
  Serial.print(" ");
  Serial.print(hour, DEC); //24-hr
  Serial.print(":");
  Serial.print(minute, DEC);
  Serial.print(":");
  Serial.println(second, DEC);

  //getPhotoResVal();
  Serial.print("Machine d'état : ");
  Serial.println(motorState);

  // errorManaging use minut and hour var, get by readRTC
  errorManaging();

  // programmation de l'ouverture de la porte en fonction de l'heure ou d'autres paramètres
  doorManaging();

  bool osf = readOscillatorStopFlag();

  Serial.print("OSF : ");
  Serial.println(osf ? "STOP" : "OK");

  if (osf) {
    onOffErrorLedState = 1;
    clearOscillatorStopFlag();
  }

  /*float vbat = analogRead(FUELGAUGE_PIN);
  Serial.print("Vbat : ");
  Serial.println(vbat);

  vbat = vbat/65.4;
  Serial.print("Vbat en Volt: ");
  Serial.println(vbat);*/

  // Afficher les résultats
  Serial.print("Lever du soleil : ");
  Serial.print(sunriseHour);
  Serial.print(":");
  if (sunriseMinute < 10) Serial.print("0");
  Serial.println(sunriseMinute);

  Serial.print("Coucher du soleil : ");
  Serial.print(sunsetHour);
  Serial.print(":");
  if (sunsetMinute < 10) Serial.print("0");
  Serial.println(sunsetMinute);

  // Reset de l'arduino tous les premiers jours du mois à 5h55
  if(day == 1 && hour == 5 && minute == 55 && second < 3) {
    Serial.println("Premier jour du mois, redémmarage du système");
    resetFunc();
  }

  // Réarme le Watchdog Timer
  Serial.println("Watch dog reset");
  wdt_reset();
}

void timerFunction32ms() {
  playOnOffErrorLed();
  playOnOffWarningLed();
  multiBpManaging();
}
  
void loop () {
  manageDcMotor();
  if (bitRead (TIFR2, 0) == 1) {
    TCNT2 = 256 - 250;          // 250 x 64 µS = 16 ms
    bitSet (TIFR2, TOV2);     
    timerFunction32ms();
    if (varCompteur++ > 125) {  // 125 * 16 ms = 2000 ms (demi-période)
      varCompteur = 0; 
      timerFunction2s();        
    }
  }
}

/*----------------------------------------------
---------------  Setup functions ---------------
------------------------------------------------*/
void setup_pinMode() {
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(WARNING_LED_PIN, OUTPUT);
  pinMode(LIMIT_SW_DOWN_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SW_UP_PIN, INPUT_PULLUP);
  pinMode(MULTI_BP_PIN, INPUT_PULLUP);
  //pinMode(FUELGAUGE_PIN, INPUT);
  pinMode(BORNE_ENA, OUTPUT);
  pinMode(BORNE_IN1, OUTPUT);
  pinMode(BORNE_IN2, OUTPUT);
  pinMode(MOSFET_EXCITATEUR_PIN, OUTPUT);
}

void setup_timer() {
  bitClear (TCCR2A, WGM20);
  bitClear (TCCR2A, WGM21);
  TCCR2B = 0b00000111;      // Clock / 1024 soit 64 micro-s
  TIFR2 = 0b00000001;       // TOV2
  TCNT2 = 256 - 250;        // Chargement du timer à 6
}

/*----------------------------------------------
--------------  Door managing  ----------------
------------------------------------------------*/
void doorManaging() {
  // Condition d'ouverture
  if(hour == sunriseHour && minute == sunriseMinute && second < 3) {
    if(motorState == 'U' || motorState == 'E' || motorState == 'B') {
      motorState = 'O';
    }
    digitalWrite(MOSFET_EXCITATEUR_PIN, HIGH);
    // Calcul de l'heure de lever et coucher du soleil
    calculateSunriseSunset(year, month, day, sunriseHour, sunriseMinute, sunsetHour, sunsetMinute);
  }else if(hour == sunsetHour && minute == sunsetMinute && second < 3) {
    if(motorState == 'U' || motorState == 'E' || motorState == 'H') {
      motorState = 'F';
    }
    digitalWrite(MOSFET_EXCITATEUR_PIN, LOW);
  }
}

/*----------------------------------------------
--------------  Error managing  ----------------
------------------------------------------------*/
void playOnOffErrorLed() {
  if(onOffErrorLedState%4==1) {
    // Allumage de la LED
    digitalWrite(ERROR_LED_PIN, HIGH);

  } else if(onOffErrorLedState%4==3) {
    // Extinction de la LED
    digitalWrite(ERROR_LED_PIN, LOW);
  }
  if(onOffErrorLedState == 15) {
    onOffErrorLedState = 0;
  } else if(onOffErrorLedState != 0) {
    onOffErrorLedState++;
  }
}

void playOnOffWarningLed() {
  if(onOffWarningLedState%4==1) {
    // Allumage de la LED
    digitalWrite(WARNING_LED_PIN, HIGH);

  } else if(onOffWarningLedState%4==3) {
    // Extinction de la LED
    digitalWrite(WARNING_LED_PIN, LOW);
  }
  if(onOffWarningLedState == 15) {
    onOffWarningLedState = 0;
  } else if(onOffWarningLedState != 0) {
    onOffWarningLedState++;
  }
}

void errorManaging() {
// Si une erreur est active
// On 32ms, off 32ms, on 32ms, off 32ms, On 32ms, off 32ms, on 32ms, off 32ms, 
// Jouer la séquence à 2s d'interval
  if(motorState == 'E') {
    onOffErrorLedState = 1;
  } else if (motorState != 'H' and motorState != 'B') {
    onOffWarningLedState = 1;
  }
}

/*----------------------------------------------
--------------  Multi BP input  ----------------
------------------------------------------------*/
void multiBpManaging() {
  int multi_bp_pin_read_value = analogRead(MULTI_BP_PIN);
  if(multi_bp_pin_read_value < 50 && multi_bp_pin_read_value > 5) {
    Serial.println(multi_bp_pin_read_value);
    if(bp_first_read_ok == 0) {
      bp_first_read_ok = 1;
    } else if(bp_first_read_ok == 1) {
      if(motorState != 'M' && motorState != 'D') {
        if(previousMotorAction == 'M') {
          motorState = 'F';
        } else if(previousMotorAction == 'D' || previousMotorAction == 'U') {
          motorState = 'O';
        }
      }
    }
  } else {
    bp_first_read_ok = 0;
  }
}

/*----------------------------------------------
-------------  DC motor functions --------------
------------------------------------------------*/
void manageDcMotor(){
  if(motorState == 'O') { // Ouvrir
    if(digitalRead(LIMIT_SW_UP_PIN) != 0) {
      monter();
    } else {
      motorState = 'H';
      previousMotorAction = 'M';
    }
  } else if(motorState == 'F') { // Fermer
    if(digitalRead(LIMIT_SW_DOWN_PIN) != 0) {
      descendre();
    } else {
      motorState = 'B';
      previousMotorAction = 'D';
    }
  } else if(motorState == 'M') { // Monter
    if(digitalRead(LIMIT_SW_UP_PIN) == 0) {
      stopMotor();
      motorState = 'H';
    } else if(getCurrentTimeMS() > (startActionTime+TIMEOUT_MOVE_UP)) {
      stopMotor();
      motorState = 'E';
    }
  } else if(motorState == 'D') { // Descendre
    if(digitalRead(LIMIT_SW_DOWN_PIN) == 0) {
      stopMotor();
      motorState = 'B';
    } else if(getCurrentTimeMS() > (startActionTime+TIMEOUT_MOVE_DOWN)) {
      stopMotor();
      motorState = 'E';
    }
  }
}

void monter() {
  configurerSensDeRotationPontA('M');
  changeVitesseMoteurPontA(MOTOR_SPEED);
  saveActionStartTime();
  motorState = 'M';
  previousMotorAction = 'M';
}

void descendre() {
  configurerSensDeRotationPontA('D');
  changeVitesseMoteurPontA(MOTOR_SPEED);
  saveActionStartTime();
  motorState = 'D';
  previousMotorAction = 'D';
}

void stopMotor() {
  changeVitesseMoteurPontA(0);
  // Configuration du L298N en "freinage", pour le moteur câblé sur le pont A. Selon sa table de vérité, il faut que :
  digitalWrite(BORNE_IN1, HIGH);              // L'entrée IN1 doit être au niveau haut
  digitalWrite(BORNE_IN2, HIGH);              // L'entrée IN2 doit être au niveau haut
}

void configurerSensDeRotationPontA(char sensDeRotation) {
  if(sensDeRotation == 'D') {
    // Configuration du L298N en "marche avant", pour le moteur connecté au pont A. Selon sa table de vérité, il faut que :
    digitalWrite(BORNE_IN1, LOW);               // L'entrée IN1 doit être au niveau haut
    digitalWrite(BORNE_IN2, HIGH);              // L'entrée IN2 doit être au niveau bas    
  }else if(sensDeRotation == 'M') {
    // Configuration du L298N en "marche arrière", pour le moteur câblé sur le pont A. Selon sa table de vérité, il faut que :
    digitalWrite(BORNE_IN1, HIGH);              // L'entrée IN1 doit être au niveau bas
    digitalWrite(BORNE_IN2, LOW);               // L'entrée IN2 doit être au niveau haut
  }
}

void changeVitesseMoteurPontA(int nouvelleVitesse) {
  // Génère un signal PWM permanent, de rapport cyclique égal à "nouvelleVitesse" (valeur comprise entre 0 et 255)
  analogWrite(BORNE_ENA, nouvelleVitesse);
}

/*----------------------------------------------
----------------  RTC functions ----------------
------------------------------------------------*/
void init_RTC() {
  Serial.println(F("RTC initilization"));
 	Wire.begin();
  readRTC(false);
}

void saveActionStartTime() {
  startActionTime = Clock.getMinute()*60 + Clock.getSecond();
  Serial.print("Action started at ");
  Serial.print(startActionTime);
  Serial.println("s");
}

short getCurrentTimeMS() {
  return Clock.getMinute()*60 + Clock.getSecond();
}

void readRTC(bool log=false) {
  // Args: Log (True, False)
 	// Return Time, format: 
  year = Clock.getYear();
  month = Clock.getMonth(Century);
  day = Clock.getDate(); 
  hour = Clock.getHour(h12, PM);
  minute = Clock.getMinute();
  second = Clock.getSecond();

  if(log) {
    Serial.print(year, DEC);
    Serial.print("-");
    Serial.print(month, DEC);
    Serial.print("-");
    Serial.print(day, DEC);
    Serial.print(" ");
    Serial.print(hour, DEC); //24-hr
    Serial.print(":");
    Serial.print(minute, DEC);
    Serial.print(":");
    Serial.println(second, DEC);
  }
}

/*----------------------------------------------
-------------  Photo res functions -------------
------------------------------------------------*/
void getPhotoResVal() {
  // Lecture de l'entrée analogique (pour rappel, cela retourne une valeur sur 10 bits, comprise entre 0 et 1023)
  valeurDeLentreeAnalogique = analogRead(R_PHOTO_PIN);

  // Détermination de la tension présente sur l'entrée analogique
  tensionDeLentreeAnalogique = float(V_ALIM * valeurDeLentreeAnalogique) / 1023;

  // Détermination de la valeur de la LDR, en fonction de la valeur de la résistance fixe, et de la tension précédemment trouvée
      // Nota : je reprends ici la formule de calcul développée un peu plus haut dans ce tuto, à savoir :
      //   → LDR = R*(Vcc – Vs) / Vs
      //   → d'où valLDR = R * (tensionAlimArduino – tensionEntreeAnalogique) / tensionEntreeAnalogique
  valeurOhmiqueDeLaLDR = RES_DIV_PHOTO * (V_ALIM - tensionDeLentreeAnalogique) / tensionDeLentreeAnalogique;

  // Puis on affiche ces valeurs sur le moniteur série de l'interface arduino
  Serial.print(F("Tension au milieu du pont diviseur : "));
  Serial.print(tensionDeLentreeAnalogique);
  Serial.println(F(" V"));

  Serial.print(F("Estimation de la valeur de la LDR (par calcul) : "));
  Serial.print(valeurOhmiqueDeLaLDR);
  Serial.println(F(" ohms"));

  Serial.println("");
}

/*----------------------------------------------
-------------  Calcul sunSet/sunRise -------------
------------------------------------------------*/
double degToRad(double degree) {
  return degree * (M_PI / 180.0);
}

double radToDeg(double radian) {
  return radian * (180.0 / M_PI);
}

// Calcul du nombre de jours depuis le début de l'année
int dayOfYear(int year, int month, int day) {
  int N1 = floor(275 * month / 9);
  int N2 = floor((month + 9) / 12);
  int N3 = (1 + floor((year - 4 * floor(year / 4) + 2) / 3));
  return N1 - (N2 * N3) + day - 30;
}

void calculateSunriseSunset(int year, int month, int day, int &sunriseHour,int &sunriseMinute, int &sunsetHour, int &sunsetMinute) {
  int doy = dayOfYear(year, month, day);
  double sunrise;
  double sunset;

  Serial.print("Date : ");
  Serial.print(year);
  Serial.print("/");
  Serial.print(month);
  Serial.print("/");
  Serial.println(day);

  // Calcul de la longitude solaire moyenne
  double L = (280.460 + 0.9856474 * doy);
  L = fmod(L, 360);
  if (L < 0) L += 360;

  // Calcul de l'anomalie solaire
  double g = (357.528 + 0.9856003 * doy);
  g = fmod(g, 360);
  if (g < 0) g += 360;

  // Calcul de l'obliquité de l'écliptique
  double lambda = L + 1.915 * sin(degToRad(g)) + 0.020 * sin(degToRad(2 * g));
  double epsilon = 23.439 - 0.0000004 * doy;

  // Ascension droite du soleil
  double RA = radToDeg(atan2(cos(degToRad(epsilon)) * sin(degToRad(lambda)), cos(degToRad(lambda))));
  RA = fmod(RA, 360);
  if (RA < 0) RA += 360;

  // Déclinaison du soleil
  double declination = radToDeg(asin(sin(degToRad(epsilon)) * sin(degToRad(lambda))));

  // Temps solaire moyen
  double eqTime = 4 * (L - RA);

  // Heure locale en minutes
  double haSunrise = radToDeg(acos(cos(degToRad(90.833)) / (cos(degToRad(LATITUDE)) * cos(degToRad(declination))) - tan(degToRad(LATITUDE)) * tan(degToRad(declination))));

  // Calcul du lever du soleil
  sunrise = (720 - 4 * (LONGITUDE + haSunrise) - eqTime) / 60 + TIMEZONE;

  // Calcul du coucher du soleil
  sunset = (720 - 4 * (LONGITUDE - haSunrise) - eqTime) / 60 + TIMEZONE;

  // SunRise - 15 minutes
  sunrise = sunrise - 0.25;

  // SunSet + 20 minutes
  sunset = sunset + 0.33;

  sunriseHour = (int)sunrise;
  sunriseMinute = (int)((sunrise - sunriseHour) * 60);

  sunsetHour = (int)sunset;
  sunsetMinute = (int)((sunset - sunsetHour) * 60);
 
  Serial.print("sunsetHour : ");
  Serial.println(sunsetHour);

  Serial.print("sunsetMinute : ");
  Serial.println(sunsetMinute);
    
  Serial.print("sunriseHour : ");
  Serial.println(sunriseHour);

  Serial.print("sunriseMinute : ");
  Serial.println(sunriseMinute);
}
