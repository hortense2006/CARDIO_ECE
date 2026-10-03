// ===================== FS4 : BEEPER LE RYTHME CARDIAQUE =====================

// PINS
const int BUZZER_PIN = 9;   // PB1 - sortie PWM
const int BTN_BEEP    = 11; // PB3 - interruption/reprise du beepage

// SEUILS (cohérents avec FS3)
const int BPM_MIN_VALIDE = 30;
const int BPM_MAX_VALIDE = 220;
const int SEUIL_BRADY    = 60;
const int SEUIL_TACHY    = 100;

// FREQUENCES DES TROIS SONS (ET4.2)
const int FREQ_AIGU   = 1500; // fréquence élevée
const int FREQ_MODERE = 800;  // fréquence normale
const int FREQ_GRAVE  = 400;  // fréquence faible

// DUREE D'UN BEEP (ms) - le son ne dure qu'une fraction de l'intervalle entre deux battements
const int DUREE_BEEP = 100;

// GESTION DU BOUTON (anti-rebond logiciel)
bool beepActif = true;
bool dernierEtatBouton = HIGH;
unsigned long dernierDebounce = 0;
const unsigned long DELAI_DEBOUNCE = 50;

// GESTION DU RYTHME DES BEEPS
unsigned long dernierBeep = 0;
bool buzzerEnCours = false;

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BTN_BEEP, INPUT_PULLUP); // pull-up interne : bouton relâché = HIGH, pressé = LOW
  noTone(BUZZER_PIN);
}

void loop() {
  int bpm = 100;
  gererBouton();     // ET4.3 : vérifié à chaque itération, sans delay() bloquant
  gererBeepage(bpm); // bpm = valeur mesurée par FS1, à remplacer par votre variable réelle
}

// ================= ET4.3 : INTERRUPTION / REPRISE DU BEEPAGE =================
void gererBouton() {
  bool etatBouton = digitalRead(BTN_BEEP);

  // Anti-rebond : on ne valide un changement d'état que s'il est stable depuis DELAI_DEBOUNCE
  if (etatBouton != dernierEtatBouton) {
    dernierDebounce = millis();
  }

  if ((millis() - dernierDebounce) > DELAI_DEBOUNCE) {
    // Détection d'un appui franc (front descendant : HIGH -> LOW)
    if (etatBouton == LOW && dernierEtatBouton == HIGH) {
      beepActif = !beepActif;
      if (!beepActif) {
        noTone(BUZZER_PIN); // coupe immédiatement le son en cours si on désactive
      }
    }
  }

  dernierEtatBouton = etatBouton;
}

// ================= ET4.1 et ET4.2 : BEEPAGE SELON LE BPM =================
void gererBeepage(int bpmActuel) {
  if (!beepActif) {
    return; // système interrompu (ET4.3), on ne fait rien
  }

  if (bpmActuel < BPM_MIN_VALIDE || bpmActuel > BPM_MAX_VALIDE) {
    noTone(BUZZER_PIN); // valeur aberrante : pas de son (cohérent avec ET3.3 pour les LEDs)
    return;
  }

  // Intervalle entre deux beeps, calculé à partir du BPM (ET4.1 : rythme = fréquence cardiaque)
  unsigned long intervalle = 60000UL / bpmActuel; // en millisecondes

  unsigned long maintenant = millis();

  if (!buzzerEnCours && (maintenant - dernierBeep >= intervalle)) {
    // Démarrage d'un nouveau beep
    int frequence = choisirFrequence(bpmActuel); // ET4.2 : trois sons différents
    tone(BUZZER_PIN, frequence);
    dernierBeep = maintenant;
    buzzerEnCours = true;
  }

  if (buzzerEnCours && (maintenant - dernierBeep >= DUREE_BEEP)) {
    // Fin du beep en cours (le son ne dure qu'une fraction de l'intervalle)
    noTone(BUZZER_PIN);
    buzzerEnCours = false;
  }
}

int choisirFrequence(int bpmActuel) {
  if (bpmActuel < SEUIL_BRADY) {
    return FREQ_GRAVE;   // fréquence cardiaque faible
  } else if (bpmActuel <= SEUIL_TACHY) {
    return FREQ_MODERE;  // fréquence cardiaque normale
  } else {
    return FREQ_AIGU;    // fréquence cardiaque élevée
  }
}

