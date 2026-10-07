#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>

// Configuration Écran OLED (Bus I2C A4/A5)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Module RTC DS3231 / DS1307
RTC_DS3231 rtc;

// Pins selon relevé matériel sur votre carte Arduino Nano
const int PIN_ENC_CLK  = 2; // Signal A Encodeur (Interrupt D2)
const int PIN_ENC_DT   = 3; // Signal B Encodeur (D3)
const int PIN_ENC_SW   = 4; // Bouton pouss. Encodeur (D4) -> Entrée Menu / Valider
const int PIN_BTN_SAVE = 5; // Bouton d'enregistrement (D5) -> Moyenne 10 mesures

// Adresses EEPROM (ATmega328P)
const int ADDR_LANG      = 0; // 0 = FR, 1 = EN
const int ADDR_COUNT     = 1; // Nb d'enregistrements (uint8_t)
const int ADDR_DATA_HEAD = 2; // Début des structures de données
const int MAX_RECORDS    = 150;

struct Record {
  uint8_t  bpm;
  uint32_t timestamp; // Epoch Unix (Date + Heure)
};

// États du système
enum State { STATE_GRAPH, STATE_MENU, STATE_CONSULT };
State currentState = STATE_GRAPH;

// Logic Encodeur
volatile int encoderPos = 0;
int lastEncoderPos = 0;
int menuIndex = 0;
const int MENU_ITEMS_COUNT = 4; // 1. Consulter, 2. Effacer, 3. Langue, 4. Quitter

// Anti-rebond boutons
bool lastSwState = HIGH;
bool lastSaveBtnState = HIGH;

// Multilingue
uint8_t currentLang = 0; // 0 = FR, 1 = EN
const char* menuTxtFR[] = {"1. Consulter EEPROM", "2. Effacer EEPROM", "3. Langue: FR", "4. Quitter"};
const char* menuTxtEN[] = {"1. View EEPROM",      "2. Clear EEPROM", "3. Lang: EN",     "4. Exit"};

// Variable temporaire de travail pour la fréquence instantanée
uint8_t bpmInstantane = 75; 

void setup() {
  Serial.begin(9600);

  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_BTN_SAVE, INPUT_PULLUP);

  // Interruption dédiée à l'encodeur rotatif
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), isrEncoder, CHANGE);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;); // Arrêt si OLED non détecté
  }
  display.clearDisplay();

  rtc.begin();

  // Lecture de la langue enregistrée en EEPROM
  currentLang = EEPROM.read(ADDR_LANG);
  if (currentLang > 1) {
    currentLang = 0; // FR par défaut
    EEPROM.update(ADDR_LANG, currentLang);
  }

  afficherEcranCourbe();
}

void loop() {
  // 1. Déclenchement de l'enregistrement (Moyenne de 10 mesures - ET6.2)
  bool saveBtnState = digitalRead(PIN_BTN_SAVE);
  if (lastSaveBtnState == HIGH && saveBtnState == LOW) {
    effectuerEnregistrementMoyen();
    delay(200);
  }
  lastSaveBtnState = saveBtnState;

  // 2. Gestion du bouton de l'encodeur (D4) : Ouverture Menu / Validation
  bool swState = digitalRead(PIN_ENC_SW);
  if (lastSwState == HIGH && swState == LOW) {
    gererAppuiEncodeur();
    delay(200);
  }
  lastSwState = swState;

  // 3. Navigation dans le Menu via rotation d'encodeur
  if (encoderPos != lastEncoderPos) {
    if (currentState == STATE_MENU) {
      if (encoderPos > lastEncoderPos) {
        menuIndex = (menuIndex + 1) % MENU_ITEMS_COUNT;
      } else {
        menuIndex = (menuIndex - 1 + MENU_ITEMS_COUNT) % MENU_ITEMS_COUNT;
      }
      afficherMenu();
    }
    lastEncoderPos = encoderPos;
  }

  // 4. Affichage dynamique de la courbe quand le menu n'est pas actif
  if (currentState == STATE_GRAPH) {
    rafraichirCourbePPG();
    delay(40);
  }
}

// --- INTERRUPT ENCODEUR ---
void isrEncoder() {
  if (digitalRead(PIN_ENC_CLK) != digitalRead(PIN_ENC_DT)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

// --- LOGIQUE BOUTON ENCODEUR ---
void gererAppuiEncodeur() {
  if (currentState == STATE_GRAPH) {
    currentState = STATE_MENU;
    menuIndex = 0;
    afficherMenu();
  } 
  else if (currentState == STATE_MENU) {
    switch (menuIndex) {
      case 0: // Consulter
        currentState = STATE_CONSULT;
        afficherConsultationEEPROM(0);
        break;

      case 1: // Effacer
        EEPROM.update(ADDR_COUNT, 0);
        display.clearDisplay();
        display.setCursor(0, 20);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.println(currentLang == 0 ? F("EEPROM effacee !") : F("EEPROM cleared!"));
        display.display();
        delay(1000);
        afficherMenu();
        break;

      case 2: // Basculer Langue FR/EN
        currentLang = (currentLang == 0) ? 1 : 0;
        EEPROM.update(ADDR_LANG, currentLang);
        afficherMenu();
        break;

      case 3: // OPTION QUITTER -> RETOUR COURBE PPG
        currentState = STATE_GRAPH;
        afficherEcranCourbe();
        break;
    }
  } 
  else if (currentState == STATE_CONSULT) {
    currentState = STATE_MENU;
    afficherMenu();
  }
}

// --- ALGORITHME MOYENNAGE 10 MESURES (ET6.2 & ET6.3) ---
void effectuerEnregistrementMoyen() {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(currentLang == 0 ? F("Calcul Moyenne...") : F("Averaging..."));
  display.println(currentLang == 0 ? F("10 mesures en cours") : F("10 samples active"));
  display.display();

  uint16_t sommeBpm = 0;
  for (int i = 0; i < 10; i++) {
    // Remplacer "bpmInstantane" par votre fonction d'acquisition brute du signal FS1
    sommeBpm += (bpmInstantane + random(-2, 3)); 
    delay(150);
  }
  uint8_t bpmMoyen = sommeBpm / 10;

  DateTime now = rtc.now();
  Record newRecord = { bpmMoyen, now.unixtime() };

  uint8_t count = EEPROM.read(ADDR_COUNT);
  if (count < MAX_RECORDS) {
    int addr = ADDR_DATA_HEAD + (count * sizeof(Record));
    EEPROM.put(addr, newRecord);
    EEPROM.update(ADDR_COUNT, count + 1);

    display.clearDisplay();
    display.setCursor(0, 15);
    display.println(currentLang == 0 ? F("Enregistre !") : F("Saved !"));
    display.print(F("BPM Moyen: ")); 
    display.println(bpmMoyen);
    display.display();
    delay(1200);
  }
  afficherEcranCourbe();
}

// --- FONCTIONS D'AFFICHAGE OLED ---
void afficherEcranCourbe() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Signal PPG - Direct"));
  display.display();
}

void rafraichirCourbePPG() {
  static int x = 0;
  static int lastY = 32;
  int y = 32 + (sin(x * 0.1) * 15); // Tracé simulé du signal PPG

  display.drawLine(x, lastY, x + 1, y, SSD1306_WHITE);
  lastY = y;
  x++;
  if (x >= SCREEN_WIDTH) {
    x = 0;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Signal PPG - Direct"));
  }
  display.display();
}

void afficherMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("=== MENU FS6 ==="));

  const char** txt = (currentLang == 0) ? menuTxtFR : menuTxtEN;

  for (int i = 0; i < MENU_ITEMS_COUNT; i++) {
    display.setCursor(5, 16 + (i * 12));
    if (i == menuIndex) display.print(F("> "));
    else display.print(F("  "));
    display.println(txt[i]);
  }
  display.display();
}

void afficherConsultationEEPROM(uint8_t index) {
  uint8_t count = EEPROM.read(ADDR_COUNT);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (count == 0) {
    display.println(currentLang == 0 ? F("Aucune donnee") : F("No records"));
  } else {
    Record rec;
    int addr = ADDR_DATA_HEAD + (index * sizeof(Record));
    EEPROM.get(addr, rec);
    DateTime dt(rec.timestamp);

    display.print(currentLang == 0 ? F("Mesure ") : F("Record "));
    display.print(index + 1); display.print(F("/")); display.println(count);
    display.print(F("BPM: ")); display.println(rec.bpm);
    display.print(F("Date: ")); display.print(dt.day()); display.print(F("/")); display.print(dt.month()); display.print(F("/")); display.println(dt.year());
    display.print(F("Heure: ")); display.print(dt.hour()); display.print(F(":")); display.println(dt.minute());
  }
  display.display();
}