/*
  HenHouse V1.1
  LE MELLEC Julien 22/11/2023 - révision 10/2026

  board:
    - RTC DS3231
    - DC motor 12V (L298N)
    - 2 limit switches
    - LED erreur + LED warning
    - Photo resistor
    - ADC input, fuel gauge
    - Multi BP (ouverture/fermeture porte)
    - MOSFET d'alimentation de l'excitateur de clôture

  Changements V1.1 :
    - Filtrage anti-IEM des fins de course (intégrateur logiciel)
    - Timeouts moteur basés sur millis() (l'ancien calcul minute*60+seconde
      ne détectait plus le timeout au passage d'une heure à la suivante)
    - Gestion jour/nuit par période (et non plus à la minute exacte) :
      la porte retrouve le bon état après un reset ou une coupure
    - Confirmation sur 2 lectures RTC avant de changer de période
    - Contrôle de cohérence des lectures RTC + timeout I2C
    - Clôture coupée pendant les mouvements de porte (moins d'IEM)
    - Reset mensuel protégé contre une boucle de redémarrage
    - Recalcul lever/coucher du soleil à chaque changement de jour
*/

#include <Wire.h>     // https://www.arduino.cc/en/reference/wire
#include <DS3231.h>   // https://github.com/NorthernWidget/DS3231
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

// Filtrage des fins de course :
// un échantillon toutes les LIMIT_SW_SAMPLE_MS ms, il faut LIMIT_SW_FILTER_COUNT
// échantillons "nets" dans le même sens pour changer d'état (~15 ms).
// Une impulsion de clôture (quelques centaines de µs) ne fausse qu'un échantillon.
#define LIMIT_SW_SAMPLE_MS     1
#define LIMIT_SW_FILTER_COUNT  15

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
#define MOTOR_SPEED         255     // between 0 & 255
#define TIMEOUT_MOVE_UP_MS   25000UL
#define TIMEOUT_MOVE_DOWN_MS 12000UL

// Déclaration de la sortie de commande du mosfet qui alimente l'excitateur
#define MOSFET_EXCITATEUR_PIN  17

// Délai minimum après démarrage avant d'autoriser le reset mensuel
#define MONTHLY_RESET_GUARD_MS 120000UL

// Déclare la fonction de réinitialisation à l'adresse 0
void(* resetFunc) (void) = 0;

/*----------------------------------------------
-----------  Entrées filtrées (IEM) ------------
------------------------------------------------*/
struct FilteredInput {
  uint8_t pin;
  uint8_t counter;            // intégrateur 0..LIMIT_SW_FILTER_COUNT
  bool active;                // état filtré : true = contact fermé (LOW, pull-up)
  unsigned long lastSampleMs;
};

FilteredInput limitSwUp   = {LIMIT_SW_UP_PIN,   0, false, 0};
FilteredInput limitSwDown = {LIMIT_SW_DOWN_PIN, 0, false, 0};

void initFilteredInput(FilteredInput &in);
void updateFilteredInput(FilteredInput &in);

// Variables DC motor
char motorState = 'U'; // M=Monter, D=Descendre, 'O'=Ouvrir, 'F'=Fermer 'H'=Haut, 'B'=Bas, U=Unknown, 'E'=Erreur
char previousMotorAction = 'U'; // M=Monter, D=Descendre, U=Unknown
unsigned long startActionTime = 0;
void manageDcMotor();

// RTC Variables
DS3231 Clock;
byte year;
byte month;
byte day;
byte hour;
byte minute;
byte second;
bool Century = false;
bool h12;
bool PM;
void init_RTC();
bool readRTC(bool log = false);

// Heure lever et coucher du soleil
int sunriseHour;
int sunriseMinute;
int sunsetHour;
int sunsetMinute;
byte lastSunCalcDay = 0;   // 0 = jamais calculé

// Gestion jour/nuit
int8_t dayPeriod = -1;           // -1 inconnu, 0 nuit, 1 jour
int8_t candidatePeriod = -1;
byte candidatePeriodCount = 0;
#define PERIOD_CONFIRM_COUNT 2   // nb de lectures RTC concordantes (2 x 2 s)

// Photo res var
int valeurDeLentreeAnalogique;
float tensionDeLentreeAnalogique;
long valeurOhmiqueDeLaLDR;
void getPhotoResVal();

// Timer var
byte varCompteur = 0;

// Error managment var
byte onOffErrorLedState = 0;
byte onOffWarningLedState = 0;

// Multi BP managing
byte bp_first_read_ok = 0; // 0 = relâché, 1 = 1ère lecture OK, 2 = action faite, attente relâchement
void multiBpManaging();

void setup_pinMode();
void setup_timer();
void stopMotor();
void manageFence();

void setup () {
  // Si le reset vient du watchdog, on le désarme tout de suite
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  setup_pinMode();

  // Moteur à l'arrêt et clôture coupée tant que l'état n'est pas connu
  stopMotor();
  digitalWrite(MOSFET_EXCITATEUR_PIN, LOW);

  init_RTC();
  setup_timer();

  // État initial fiable des fins de course avant toute action moteur
  initFilteredInput(limitSwUp);
  initFilteredInput(limitSwDown);

  // Remise à zéro du bit d'erreur de l'oscillateur du DS3231
  clearOscillatorStopFlag();

  Serial.println(F("Demarrage avec Watchdog Timer (4 secondes)."));
  wdt_enable(WDTO_4S);
}

void loop () {
  updateFilteredInput(limitSwUp);
  updateFilteredInput(limitSwDown);
  manageDcMotor();
  manageFence();

  if (bitRead(TIFR2, 0) == 1) {
    TCNT2 = 256 - 250;          // 250 x 64 µs = 16 ms
    bitSet(TIFR2, TOV2);
    timerFunction32ms();
    if (varCompteur++ > 125) {  // 126 * 16 ms ≈ 2 s
      varCompteur = 0;
      timerFunction2s();
    }
  }
}

void timerFunction32ms() {
  playOnOffErrorLed();
  playOnOffWarningLed();
  multiBpManaging();
}

void timerFunction2s() {
  bool rtcOk = readRTC(true);

  Serial.print(F("Machine d'etat : "));
  Serial.println(motorState);
  Serial.print(F("Fins de course H/B : "));
  Serial.print(limitSwUp.active);
  Serial.print("/");
  Serial.println(limitSwDown.active);

  errorManaging();

  if (rtcOk) {
    // Programmation de l'ouverture/fermeture de la porte
    doorManaging();

    // Reset de l'arduino tous les premiers jours du mois à 5h55
    // (garde millis() pour ne pas reboucler juste après le redémarrage)
    if (day == 1 && hour == 5 && minute == 55 && millis() > MONTHLY_RESET_GUARD_MS) {
      Serial.println(F("Premier jour du mois, redemarrage du systeme"));
      Serial.flush();
      resetFunc();
    }
  } else {
    onOffErrorLedState = 1;
  }

  bool osf = readOscillatorStopFlag();
  Serial.print(F("OSF : "));
  Serial.println(osf ? F("STOP") : F("OK"));
  if (osf) {
    onOffErrorLedState = 1;
    clearOscillatorStopFlag();
  }

  Serial.print(F("Lever du soleil : "));
  Serial.print(sunriseHour);
  Serial.print(":");
  if (sunriseMinute < 10) Serial.print("0");
  Serial.println(sunriseMinute);

  Serial.print(F("Coucher du soleil : "));
  Serial.print(sunsetHour);
  Serial.print(":");
  if (sunsetMinute < 10) Serial.print("0");
  Serial.println(sunsetMinute);

  // Réarme le Watchdog Timer
  wdt_reset();
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
  pinMode(BORNE_ENA, OUTPUT);
  pinMode(BORNE_IN1, OUTPUT);
  pinMode(BORNE_IN2, OUTPUT);
  pinMode(MOSFET_EXCITATEUR_PIN, OUTPUT);
}

void setup_timer() {
  bitClear(TCCR2A, WGM20);
  bitClear(TCCR2A, WGM21);
  TCCR2B = 0b00000111;      // Clock / 1024 soit 64 µs
  TIFR2 = 0b00000001;       // TOV2
  TCNT2 = 256 - 250;        // Chargement du timer à 6
}

/*----------------------------------------------
-----------  Entrées filtrées (IEM) ------------
------------------------------------------------*/
void initFilteredInput(FilteredInput &in) {
  // Vote majoritaire sur 32 lectures pour démarrer avec un état sûr
  uint8_t lowCount = 0;
  for (uint8_t i = 0; i < 32; i++) {
    if (digitalRead(in.pin) == LOW) lowCount++;
    delay(1);
  }
  in.active = (lowCount > 16);
  in.counter = in.active ? LIMIT_SW_FILTER_COUNT : 0;
  in.lastSampleMs = millis();
}

void updateFilteredInput(FilteredInput &in) {
  unsigned long now = millis();
  if (now - in.lastSampleMs < LIMIT_SW_SAMPLE_MS) return;
  in.lastSampleMs = now;

  if (digitalRead(in.pin) == LOW) {
    if (in.counter < LIMIT_SW_FILTER_COUNT) in.counter++;
  } else {
    if (in.counter > 0) in.counter--;
  }

  // Hystérésis : on ne bascule qu'aux extrémités de l'intégrateur
  if (in.counter >= LIMIT_SW_FILTER_COUNT) {
    in.active = true;
  } else if (in.counter == 0) {
    in.active = false;
  }
}

/*----------------------------------------------
--------------  Door managing  ----------------
------------------------------------------------*/
void doorManaging() {
  // Recalcul du lever/coucher à chaque nouveau jour
  if (day != lastSunCalcDay) {
    calculateSunriseSunset(2000 + year, month, day, sunriseHour, sunriseMinute, sunsetHour, sunsetMinute);
    lastSunCalcDay = day;
  }

  int nowMin  = hour * 60 + minute;
  int riseMin = sunriseHour * 60 + sunriseMinute;
  int setMin  = sunsetHour * 60 + sunsetMinute;
  int8_t newPeriod = (nowMin >= riseMin && nowMin < setMin) ? 1 : 0;

  // Confirmation sur plusieurs lectures pour ignorer une lecture RTC corrompue
  if (newPeriod != candidatePeriod) {
    candidatePeriod = newPeriod;
    candidatePeriodCount = 1;
    return;
  }
  if (candidatePeriodCount < PERIOD_CONFIRM_COUNT) {
    candidatePeriodCount++;
    if (candidatePeriodCount < PERIOD_CONFIRM_COUNT) return;
  }

  // On n'agit que sur un changement de période (ou au démarrage),
  // ce qui laisse le bouton manuel prioritaire le reste du temps.
  if (candidatePeriod != dayPeriod) {
    dayPeriod = candidatePeriod;
    if (dayPeriod == 1) {
      Serial.println(F("Periode jour : ouverture de la porte"));
      if (motorState == 'U' || motorState == 'E' || motorState == 'B') {
        motorState = 'O';
      }
    } else {
      Serial.println(F("Periode nuit : fermeture de la porte"));
      if (motorState == 'U' || motorState == 'E' || motorState == 'H') {
        motorState = 'F';
      }
    }
  }
}

/*----------------------------------------------
--------------  Clôture électrique -------------
------------------------------------------------*/
void manageFence() {
  // Clôture active le jour, coupée pendant les mouvements de porte
  // pour éviter les IEM au moment où les fins de course sont critiques.
  bool doorMoving = (motorState == 'M' || motorState == 'D' ||
                     motorState == 'O' || motorState == 'F');
  bool fenceOn = (dayPeriod == 1) && !doorMoving;
  digitalWrite(MOSFET_EXCITATEUR_PIN, fenceOn ? HIGH : LOW);
}

/*----------------------------------------------
--------------  Error managing  ----------------
------------------------------------------------*/
void playOnOffErrorLed() {
  if (onOffErrorLedState % 4 == 1) {
    digitalWrite(ERROR_LED_PIN, HIGH);
  } else if (onOffErrorLedState % 4 == 3) {
    digitalWrite(ERROR_LED_PIN, LOW);
  }
  if (onOffErrorLedState == 15) {
    onOffErrorLedState = 0;
  } else if (onOffErrorLedState != 0) {
    onOffErrorLedState++;
  }
}

void playOnOffWarningLed() {
  if (onOffWarningLedState % 4 == 1) {
    digitalWrite(WARNING_LED_PIN, HIGH);
  } else if (onOffWarningLedState % 4 == 3) {
    digitalWrite(WARNING_LED_PIN, LOW);
  }
  if (onOffWarningLedState == 15) {
    onOffWarningLedState = 0;
  } else if (onOffWarningLedState != 0) {
    onOffWarningLedState++;
  }
}

void errorManaging() {
  // Séquence de clignotement jouée toutes les 2 s
  if (motorState == 'E') {
    onOffErrorLedState = 1;
  } else if (motorState != 'H' && motorState != 'B') {
    onOffWarningLedState = 1;
  }
}

/*----------------------------------------------
--------------  Multi BP input  ----------------
------------------------------------------------*/
void multiBpManaging() {
  int multi_bp_pin_read_value = analogRead(MULTI_BP_PIN);
  if (multi_bp_pin_read_value < 50 && multi_bp_pin_read_value > 5) {
    if (bp_first_read_ok == 0) {
      bp_first_read_ok = 1;
    } else if (bp_first_read_ok == 1) {
      bp_first_read_ok = 2; // une seule action par appui
      Serial.println(multi_bp_pin_read_value);
      if (motorState != 'M' && motorState != 'D') {
        if (previousMotorAction == 'M') {
          motorState = 'F';
        } else if (previousMotorAction == 'D' || previousMotorAction == 'U') {
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
void manageDcMotor() {
  if (motorState == 'O') { // Ouvrir
    if (!limitSwUp.active) {
      monter();
    } else {
      motorState = 'H';
      previousMotorAction = 'M';
    }
  } else if (motorState == 'F') { // Fermer
    if (!limitSwDown.active) {
      descendre();
    } else {
      motorState = 'B';
      previousMotorAction = 'D';
    }
  } else if (motorState == 'M') { // Monter
    if (limitSwUp.active) {
      stopMotor();
      motorState = 'H';
    } else if (millis() - startActionTime > TIMEOUT_MOVE_UP_MS) {
      stopMotor();
      motorState = 'E';
      Serial.println(F("Timeout montee"));
    }
  } else if (motorState == 'D') { // Descendre
    if (limitSwDown.active) {
      stopMotor();
      motorState = 'B';
    } else if (millis() - startActionTime > TIMEOUT_MOVE_DOWN_MS) {
      stopMotor();
      motorState = 'E';
      Serial.println(F("Timeout descente"));
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
  digitalWrite(BORNE_IN1, HIGH);
  digitalWrite(BORNE_IN2, HIGH);
}

void configurerSensDeRotationPontA(char sensDeRotation) {
  if (sensDeRotation == 'D') {
    digitalWrite(BORNE_IN1, LOW);
    digitalWrite(BORNE_IN2, HIGH);
  } else if (sensDeRotation == 'M') {
    digitalWrite(BORNE_IN1, HIGH);
    digitalWrite(BORNE_IN2, LOW);
  }
}

void changeVitesseMoteurPontA(int nouvelleVitesse) {
  analogWrite(BORNE_ENA, nouvelleVitesse);
}

void saveActionStartTime() {
  startActionTime = millis();
  Serial.print(F("Action started at "));
  Serial.print(startActionTime);
  Serial.println(F(" ms"));
}

/*----------------------------------------------
----------------  RTC functions ----------------
------------------------------------------------*/
void init_RTC() {
  Serial.println(F("RTC initialization"));
  Wire.begin();
  // Évite un blocage du bus I2C en cas de parasite (core AVR >= 1.8.3)
  Wire.setWireTimeout(3000, true);
  readRTC(true);
}

bool readRTC(bool log) {
  // Lecture dans des variables temporaires, validées avant d'être utilisées
  byte y  = Clock.getYear();
  byte mo = Clock.getMonth(Century);
  byte d  = Clock.getDate();
  byte h  = Clock.getHour(h12, PM);
  byte mi = Clock.getMinute();
  byte s  = Clock.getSecond();

  if (mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || s > 59) {
    Serial.println(F("Lecture RTC incoherente, ignoree"));
    return false;
  }

  year = y; month = mo; day = d; hour = h; minute = mi; second = s;

  if (log) {
    Serial.print(year, DEC);
    Serial.print("-");
    Serial.print(month, DEC);
    Serial.print("-");
    Serial.print(day, DEC);
    Serial.print(" ");
    Serial.print(hour, DEC);
    Serial.print(":");
    Serial.print(minute, DEC);
    Serial.print(":");
    Serial.println(second, DEC);
  }
  return true;
}

bool readOscillatorStopFlag() {
  Wire.beginTransmission(0x68);   // Adresse I2C du DS3231
  Wire.write(0x0F);               // Registre Status
  Wire.endTransmission();

  Wire.requestFrom(0x68, 1);
  if (Wire.available()) {
    byte status = Wire.read();
    return bitRead(status, 7);    // Bit 7 = OSF
  }
  return false;
}

void clearOscillatorStopFlag() {
  Wire.beginTransmission(0x68);
  Wire.write(0x0F);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 1);
  if (Wire.available()) {
    byte status = Wire.read();
    status &= ~(1 << 7);          // Efface le bit OSF

    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.write(status);
    Wire.endTransmission();
  }
}

/*----------------------------------------------
-------------  Photo res functions -------------
------------------------------------------------*/
void getPhotoResVal() {
  valeurDeLentreeAnalogique = analogRead(R_PHOTO_PIN);
  tensionDeLentreeAnalogique = float(V_ALIM * valeurDeLentreeAnalogique) / 1023;
  // LDR = R*(Vcc – Vs) / Vs
  valeurOhmiqueDeLaLDR = RES_DIV_PHOTO * (V_ALIM - tensionDeLentreeAnalogique) / tensionDeLentreeAnalogique;

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

void calculateSunriseSunset(int year, int month, int day, int &sunriseHour, int &sunriseMinute, int &sunsetHour, int &sunsetMinute) {
  int doy = dayOfYear(year, month, day);
  double sunrise;
  double sunset;

  Serial.print(F("Date : "));
  Serial.print(year);
  Serial.print("/");
  Serial.print(month);
  Serial.print("/");
  Serial.println(day);

  // Longitude solaire moyenne
  double L = (280.460 + 0.9856474 * doy);
  L = fmod(L, 360);
  if (L < 0) L += 360;

  // Anomalie solaire
  double g = (357.528 + 0.9856003 * doy);
  g = fmod(g, 360);
  if (g < 0) g += 360;

  // Longitude écliptique et obliquité
  double lambda = L + 1.915 * sin(degToRad(g)) + 0.020 * sin(degToRad(2 * g));
  double epsilon = 23.439 - 0.0000004 * doy;

  // Ascension droite du soleil
  double RA = radToDeg(atan2(cos(degToRad(epsilon)) * sin(degToRad(lambda)), cos(degToRad(lambda))));
  RA = fmod(RA, 360);
  if (RA < 0) RA += 360;

  // Déclinaison du soleil
  double declination = radToDeg(asin(sin(degToRad(epsilon)) * sin(degToRad(lambda))));

  // Équation du temps
  double eqTime = 4 * (L - RA);

  // Angle horaire
  double haSunrise = radToDeg(acos(cos(degToRad(90.833)) / (cos(degToRad(LATITUDE)) * cos(degToRad(declination))) - tan(degToRad(LATITUDE)) * tan(degToRad(declination))));

  sunrise = (720 - 4 * (LONGITUDE + haSunrise) - eqTime) / 60 + TIMEZONE;
  sunset  = (720 - 4 * (LONGITUDE - haSunrise) - eqTime) / 60 + TIMEZONE;

  // SunRise - 15 minutes
  sunrise = sunrise - 0.25;
  // SunSet + 20 minutes
  sunset = sunset + 0.33;

  sunriseHour = (int)sunrise;
  sunriseMinute = (int)((sunrise - sunriseHour) * 60);
  sunsetHour = (int)sunset;
  sunsetMinute = (int)((sunset - sunsetHour) * 60);
}
