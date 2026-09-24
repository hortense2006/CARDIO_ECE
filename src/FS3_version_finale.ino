//FS3
//CONFIGURATION DES BROCHES
const uint8_t PIN_LED_ROUGE = 2;
const uint8_t PIN_LED_VERTE = 3;
const uint8_t PIN_LED_JAUNE = 4;

//SEUILS CARDIAQUES
const int SEUIL_BRADY = 60;
const int SEUIL_TACHY = 100;
const int BPM_MIN_VALIDE = 30;
const int BPM_MAX_VALIDE = 220;

void setupFS3() {
  pinMode(PIN_LED_ROUGE, OUTPUT);
  pinMode(PIN_LED_VERTE, OUTPUT);
  pinMode(PIN_LED_JAUNE, OUTPUT);
  //ETEINT LES LEDS AU DEMARAGE
  digitalWrite(PIN_LED_ROUGE, LOW);
  digitalWrite(PIN_LED_VERTE, LOW);
  digitalWrite(PIN_LED_JAUNE, LOW);
}

void afficherEtatSante(int bpm) {
//DETECTION VALEURS ABERRANTES
  if (bpm < BPM_MIN_VALIDE || bpm > BPM_MAX_VALIDE) {
    digitalWrite(PIN_LED_JAUNE, LOW);
    digitalWrite(PIN_LED_VERTE, LOW);
    digitalWrite(PIN_LED_ROUGE, LOW);
    return;
  }

  if (bpm < SEUIL_BRADY) {
    digitalWrite(PIN_LED_JAUNE, HIGH);
    digitalWrite(PIN_LED_VERTE, LOW);
    digitalWrite(PIN_LED_ROUGE, LOW);
  } 
  else if (bpm <= SEUIL_TACHY) {
    digitalWrite(PIN_LED_JAUNE, LOW);
    digitalWrite(PIN_LED_VERTE, HIGH);
    digitalWrite(PIN_LED_ROUGE, LOW);
  } 
  else {
    digitalWrite(PIN_LED_JAUNE, LOW);
    digitalWrite(PIN_LED_VERTE, LOW);
    digitalWrite(PIN_LED_ROUGE, HIGH);
  }
}
