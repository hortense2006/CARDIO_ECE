#include <EEPROM.h>
#include <Adafruit_GFX.h>

#define SSD1306_BLACK        0         // couleurs utilisées avec Adafruit_GFX
#define SSD1306_WHITE        1

// =====================================================================================
//                                    RÉGLAGES
// =====================================================================================

// ---------- Mesure (FS1) ----------
#define PIN_CAPTEUR          A0
#define PERIODE_ECH_MS       10        // 100 Hz
#define INVERSER_SIGNAL      0         // 1 si les battements apparaissent vers le bas
#define ALPHA_PASSE_BAS      0.30f     // plus petit = plus lisse
#define ALPHA_BASE           0.01f     // ligne de base (passe-haut) : ~1 s
#define NB_BLOCS             10        // min et max cherchés sur 10 blocs...
#define TAILLE_BLOC          20        // ...de 20 mesures (0,2 s) = fenêtre de 2 s
#define AMPLITUDE_MIN        4.0f      // écart max - min en pas ADC : en dessous, pas de doigt
#define ALPHA_BRUIT          0.005f    // estimation du bruit haute fréquence : ~2 s
#define SIGNAL_SUR_BRUIT     5.0f      // l'écart max - min doit valoir au moins 5 fois le bruit

// ---------- Détection des battements (FS1) ----------
#define SEUIL_HAUT           0.60f     // battement quand le signal normalisé (0..1) passe au-dessus...
#define SEUIL_BAS            0.30f     // ...puis il doit redescendre sous ce seuil (hystérésis)
#define INTERVALLE_MIN_MS    300       // 200 bpm max
#define REFRACTAIRE_RELATIF  0.6f      // pas de battement avant 60 % de l'intervalle précédent (onde dicrote)
#define INTERVALLE_MAX_MS    2000      // 30 bpm min ; plus de battement pendant 2 s -> perdu
#define ECART_MAX            0.35f     // deux intervalles successifs ne doivent pas différer de plus de 35 %
#define NB_INTERVALLES_OK    2         // nb d'intervalles réguliers d'affilée avant d'afficher
#define NB_MOYENNE           4         // BPM = moyenne des 4 derniers intervalles réguliers

// ---------- ET5.1 : valeur de BPM conforme ----------
#define BPM_CONFORME_MIN     30        // en dehors de [30 ; 200] bpm : tiret à la place du nombre
#define BPM_CONFORME_MAX     200

// ---------- Voyants d'état de santé (FS3) ----------
#define PIN_VOYANT_JAUNE     3         // fréquence faible
#define PIN_VOYANT_VERT      4         // fréquence normale
#define PIN_VOYANT_ROUGE     5         // fréquence élevée
// ET3.2 : plages de la fréquence cardiaque au repos chez l'adulte (AHA / ESC)
#define BPM_NORMAL_MIN       60        // en dessous : bradycardie -> JAUNE
#define BPM_NORMAL_MAX       100       // au-dessus  : tachycardie -> ROUGE
// ET3.3 : valeur aberrante = hors de [BPM_CONFORME_MIN ; BPM_CONFORME_MAX] = [30 ; 200] -> aucun voyant
#define HYSTERESIS_BPM       2         // évite le clignotement quand le BPM oscille autour de 60 ou 100
#define TEST_VOYANTS         1         // 1 = au démarrage, chaque voyant s'allume 0,5 s, un par un (câblage)
#define SIMULER_BPM          0         // 1 = mode test : taper un BPM dans le moniteur série (115200,
                                       //     "Nouvelle ligne") ; les voyants réagissent à cette valeur ;
                                       //     "-1" = retour au vrai capteur. Remettre 0 pour l'utilisation.

// Voyant allumé (déclaré ici, avant toute fonction, pour l'IDE Arduino)
enum Voyant : uint8_t { AUCUN, JAUNE, VERT, ROUGE };

// ---------- Beeper (FS4) ----------
#define PIN_BUZZER           2         // buzzer PASSIF
#define PIN_BOUTON_BEEP      15        // 15 = A1 (possible aussi : 16 = A2, 17 = A3 ; écrire le NUMÉRO)
#define BOUTON_BEEP_ACTIF_HAUT  BOUTONS_ACTIFS_HAUT   // même montage que MENU / ENREG (voir FS6) ;
                                       // mettre 0 si le bouton va directement à GND (pull-up interne)
// ET4.2 : trois sons différents (mêmes fréquences que la première version de FS4)
#define FREQ_GRAVE           400       // Hz : fréquence cardiaque faible  (voyant jaune)
#define FREQ_MODERE          800       // Hz : fréquence cardiaque normale (voyant vert)
#define FREQ_AIGU            1500      // Hz : fréquence cardiaque élevée  (voyant rouge)
#define DUREE_BEEP_MS        100       // durée d'un beep (< 300 ms = intervalle à 200 bpm)
#define ANTI_REBOND_BEEP_MS  30        // les rebonds du bouton durent quelques ms
#define TEST_BEEPER          1         // 1 = au démarrage, joue les 3 sons (grave, modéré, aigu)
#define TRACE_BEEPS          0         // 1 = écrit chaque beep dans le moniteur série (preuve de ET4.1 /
                                       //     ET4.2 pour le rapport) ; le traceur série est alors coupé
// ET4.3 : interruption "pin change" du port où se trouve le bouton
#if PIN_BOUTON_BEEP >= 15 && PIN_BOUTON_BEEP <= 17
  #define BOUTON_BEEP_VECT   PCINT1_vect   // broches A0 à A5 (port C)
#else
  #error "PIN_BOUTON_BEEP : choisir 15 (A1), 16 (A2) ou 17 (A3) ; les autres broches sont deja prises"
#endif

// ---------- Module RTC DS1302 ----------
#define RTC_RST              6
#define RTC_DAT              12
#define RTC_CLK              13
#define REGLER_HEURE         0         // 1 = remet l'heure à celle de la compilation au démarrage
                                       // (téléverser une fois avec 1, puis remettre 0 et retéléverser)
#define CLIGNOTER_2PTS       1         // 1 = les ":" de l'heure clignotent chaque seconde

// Date et heure lues sur le module (déclarée ici, avant toute fonction, pour l'IDE Arduino)
struct Heure {
  uint8_t sec, min, hr, date, mon, jour;   // jour de la semaine : 1 = dimanche ... 7 = samedi
  uint16_t yr;
};

// ---------- Écrans (FS5) ----------
#define ADRESSE_ECRAN1       0x3C      // écran 1 : heure + BPM (ET5.1)
#define ADRESSE_ECRAN2       0x3D      // écran 2 : graphique du PPG (ET5.2)
#define DUREE_COEUR_MS       150       // le cœur grossit pendant 150 ms à chaque battement
#define PERIODE_GRAPHE_MS    100       // graphique redessiné 10 fois par seconde

// Disposition du graphique (écran 2)
#define GRAPHE_L             100       // largeur de la courbe en pixels
#define GRAPHE_Y_HAUT        11
#define GRAPHE_H             40
#define GRAPHE_Y_BAS         (GRAPHE_Y_HAUT + GRAPHE_H - 1)
#define AXE_X                (GRAPHE_L + 1)      // axe vertical (amplitude) à droite
#define AXE_Y                (GRAPHE_Y_BAS + 2)  // axe horizontal (temps)
#define PX_PAR_GRAD          25                  // ET5.3 : une graduation de temps tous les 25 pixels

// ---------- Encodeur (ET5.4) ----------
// Les broches sont fixées par l'interruption du port B : A = D9 (PB1), B = D10 (PB2), bouton = D11 (PB3)
#define PIN_ENC_A            9
#define PIN_ENC_B            10
#define PIN_ENC_BOUTON       11        // clic de l'encodeur (facultatif) : même rôle que le bouton MENU/OK
#define TRANSITIONS_CRAN     4         // 4 sur un KY-040 (mettre 2 si un cran saute 2 calibres)
#define SENS_ENCODEUR        1         // -1 pour inverser le sens

// ---------- Enregistrement (FS6) ----------
#define PIN_BOUTON_ENREG     8         // ET6.2 : enregistrer (depuis la courbe) / retour (dans le menu)
#define PIN_BOUTON_MENU      7         // ET6.4 : ouvrir le menu / valider
#define BOUTONS_ACTIFS_HAUT  1         // 1 = bouton vers 5V + résistance vers GND (appuyé = HIGH)
                                       // 0 = bouton directement vers GND, pull-up interne (appuyé = LOW)
#define NIVEAU_APPUI         (BOUTONS_ACTIFS_HAUT ? HIGH : LOW)
#define ANTI_REBOND_MS       30        // un appui doit rester stable 30 ms pour compter
#define NB_MESURES_MOYENNE   10        // ET6.2 : moyenne de 10 mesures consécutives
#define DELAI_ABANDON_MS     8000      // plus de mesure valide pendant 8 s -> enregistrement annulé
#define ECART_MESURE         0.35f     // une mesure à plus de 35 % de la moyenne en cours est écartée
#define DUREE_MESSAGE_MS     1500      // durée d'affichage d'un message ("Enregistre !", ...)

// Organisation de l'EEPROM (1024 octets sur l'ATmega328P)
//   octet 0     : langue (0 = français, 1 = anglais ; 0xFF = jamais écrit -> français)
//   octet 1     : nombre d'enregistrements
//   octets 2... : les enregistrements, 6 octets chacun -> 170 au maximum
#define ADDR_LANGUE          0
#define ADDR_NB              1
#define ADDR_DONNEES         2
#define LANGUE_FR            0
#define LANGUE_EN            1

// Un enregistrement (ET6.3) : BPM + date + heure, 6 octets
struct Enregistrement {
  uint8_t bpm;
  uint8_t jour, mois, annee;         // annee = 2 derniers chiffres (26 = 2026) ; 0 = heure inconnue
  uint8_t heure, minute;
};
#define NB_ENREG_MAX         ((E2END + 1 - ADDR_DONNEES) / sizeof(Enregistrement))   // 170

// Ce que montre l'écran 2
enum Mode : uint8_t {
  MODE_GRAPHE,                       // courbe du PPG (FS5)
  MODE_MENU,                         // menu FS6
  MODE_CONSULTER,                    // une mesure enregistrée par page
  MODE_CONFIRMER,                    // "Effacer les mesures ?" Non / Oui
  MODE_ENREG,                        // enregistrement en cours (n / 10 mesures)
  MODE_MESSAGE                       // message temporaire
};
enum Message : uint8_t { MSG_ENREGISTRE, MSG_ANNULE, MSG_PLEINE, MSG_EFFACE, MSG_PAS_DOIGT, MSG_AUCUNE };
#define NB_LIGNES_MENU       4       // Consulter, Effacer, Langue, Retour

// Bouton avec anti-rebond (lu à chaque tour de boucle)
struct Bouton {
  uint8_t pin;
  bool stable;                       // état validé (au départ : niveau relâché)
  bool lu;                           // dernier état lu
  unsigned long t_changement;
  bool niveau_appui;                 // niveau lu quand le bouton est appuyé (HIGH ou LOW)
};

// ---------- Traceur série ----------
#define AFFICHER_1_SUR       2         // 1 point sur 2 envoyé (50 points/s)

// =====================================================================================
//                                 VARIABLES GLOBALES
// =====================================================================================

// Un seul objet pour les deux écrans : une image de 128 x 64 pixels (1 bit par pixel = 1024 octets)
// sur laquelle Adafruit_GFX dessine, puis envoyée à l'écran choisi (voir "PILOTE DES ÉCRANS").
// Organisation de l'image comme dans le SSD1306 : 8 "pages" de 8 lignes, un octet = 8 pixels
// verticaux, bit 0 en haut.
class EcranOLED : public Adafruit_GFX {
public:
  EcranOLED() : Adafruit_GFX(128, 64) {}
  void drawPixel(int16_t x, int16_t y, uint16_t couleur) override {
    if ((uint16_t)x >= 128 || (uint16_t)y >= 64) return;    // hors de l'écran
    ecrire_pixel(x, y, couleur);
  }
  inline void ecrire_pixel(uint8_t x, uint8_t y, uint16_t couleur) {   // sans vérification
    uint8_t *octet = &image[x + (y >> 3) * 128];
    uint8_t masque = 1 << (y & 7);
    if (couleur == SSD1306_WHITE) *octet |= masque;
    else if (couleur == SSD1306_BLACK) *octet &= ~masque;
    else *octet ^= masque;                                   // couleur 2 : inverse
  }
  // Lignes horizontales / verticales écrites directement dans l'image (bien plus rapide que
  // pixel par pixel) : Adafruit_GFX s'en sert aussi pour les rectangles pleins et les gros textes
  void drawFastHLine(int16_t x, int16_t y, int16_t l, uint16_t couleur) override {
    if (y < 0 || y >= 64) return;
    if (x < 0) { l += x; x = 0; }
    if (x + l > 128) l = 128 - x;
    for (int16_t i = 0; i < l; i++) ecrire_pixel(x + i, y, couleur);
  }
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t couleur) override {
    if (x < 0 || x >= 128) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > 64) h = 64 - y;
    for (int16_t i = 0; i < h; i++) ecrire_pixel(x, y + i, couleur);
  }
  void clearDisplay() { memset(image, 0, sizeof(image)); }
  bool begin(uint8_t adresse);         // initialise un écran (définies plus bas)
  bool envoyer(uint8_t adresse);       // envoie l'image à un écran
private:
  uint8_t image[128 * 64 / 8];
};
EcranOLED ecran;
bool ecran1_present = false;
bool ecran2_present = false;

// --- Filtres ---
float passe_bas = -1, base = 0, bruit = 0, signal_norm = 0;

// --- Normalisation : min et max par blocs de 0,2 s sur les 2 dernières secondes ---
int16_t bloc_min[NB_BLOCS], bloc_max[NB_BLOCS];   // gardés x8 en entier pour économiser la RAM
uint8_t bloc_index = 0, bloc_nb = 0, ech_dans_bloc = 0;
float min_courant = 1e9, max_courant = -1e9;

// --- Détection ---
unsigned long n_echantillons = 0;    // temps = n_echantillons * 10 ms
unsigned long t_dernier_battement = 0;
unsigned long intervalle_prec = 0;
uint8_t nb_reguliers = 0;
bool arme = false;
bool battements_ok = false;

// --- BPM ---
unsigned int intervalles[NB_MOYENNE];
uint8_t index_iv = 0, nb_iv = 0;
int bpm = 0;                         // 0 = pas de valeur valide
unsigned long t_coeur = 0;           // millis() du dernier battement (animation du cœur)

// --- FS3 ---
Voyant voyant = AUCUN;               // voyant actuellement allumé
int bpm_simule = -1;                 // mode SIMULER_BPM : -1 = pas de simulation (vrai capteur)

// --- FS4 ---
uint8_t nb_battements = 0;           // +1 à chaque battement détecté par FS1 (seule la différence compte)
uint8_t nb_battements_beepes = 0;    // battements déjà "beepés"
unsigned long t_dernier_beep = 0;    // millis() du dernier beep (trace + cadence du mode SIMULER_BPM)
volatile bool beep_actif = true;     // ET4.3 : basculé par le bouton, dans l'interruption
volatile bool bouton_beep_appuye = false;    // état du bouton après anti-rebond
volatile unsigned long t_front_beep = 0;     // millis() du dernier front vu sur la broche du bouton
volatile bool bouton_beep_evenement = false; // un appui a eu lieu (message à écrire par loop)
volatile uint16_t duree_isr_us = 0;  // temps entre l'entrée dans l'interruption et le son coupé

// --- Heure lue sur le module RTC ---
Heure heure = {0, 0, 0, 1, 1, 7, 2000};
bool heure_ok = false;               // false si le module ne répond pas (heure impossible)
bool heure_jamais_lue = true;
unsigned long t_lecture_rtc = 0;

// --- Ce qui est affiché sur l'écran 1 (on ne le redessine que si quelque chose change) ---
int bpm_affiche = -1;
bool coeur_affiche = false;
uint8_t seconde_affichee = 255;

// --- File d'échantillons remplie par l'interruption du Timer1 ---
#define FILE_LEN 16                  // 160 ms de mesures en attente au maximum (un écran prend ~30 ms)
volatile int file_ech[FILE_LEN];
volatile uint8_t file_tete = 0, file_queue = 0;

// --- Graphique (ET5.3 / ET5.4) ---
// Calibres : nombre d'échantillons moyennés par pixel
//   x 10 ms  = 10 / 20 / 40 / 80 / 200 ms par pixel
//   x 25 px  = 0,25 / 0,5 / 1 / 2 / 5 s par graduation
const uint8_t CALIBRES[] = {1, 2, 4, 8, 20};
#define NB_CALIBRES (sizeof(CALIBRES) / sizeof(CALIBRES[0]))
uint8_t calibre = 2;                 // départ : 40 ms/px = 1 s pour 25 px
uint8_t graphe[GRAPHE_L];            // ordonnée y de chaque pixel
uint8_t graphe_index = 0, graphe_nb = 0;
float cumul_pixel = 0;
uint8_t nb_cumul = 0;
unsigned long t_graphe = 0;
uint8_t compteur_serie = 0;

// --- Encodeur ---
volatile int8_t enc_transitions = 0;
volatile uint8_t enc_etat_prec = 0;
const int8_t TABLE_QUADRATURE[16] PROGMEM = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// --- FS6 ---
uint8_t langue = LANGUE_FR;          // ET6.5
uint8_t nb_enreg = 0;                // nombre d'enregistrements en EEPROM
Mode mode = MODE_GRAPHE;
uint8_t ligne_menu = 0;
uint8_t index_consult = 0;
bool confirmer_oui = false;
Message message = MSG_ENREGISTRE;
Mode mode_apres_message = MODE_GRAPHE;
unsigned long t_fin_message = 0;
uint8_t bpm_enregistre = 0;          // valeur affichée dans le message "Enregistre !"
Bouton bouton_enc   = {PIN_ENC_BOUTON, HIGH, HIGH, 0, LOW};   // clic encodeur : vers GND, pull-up
Bouton bouton_enreg = {PIN_BOUTON_ENREG, !NIVEAU_APPUI, !NIVEAU_APPUI, 0, NIVEAU_APPUI};
Bouton bouton_menu  = {PIN_BOUTON_MENU,  !NIVEAU_APPUI, !NIVEAU_APPUI, 0, NIVEAU_APPUI};
// Mesures pour la moyenne (ET6.2)
uint8_t nb_intervalles_total = 0;         // +1 à chaque battement régulier (rempli par FS1)
unsigned int dernier_intervalle = 0;      // intervalle (ms) du dernier battement
uint8_t nb_intervalles_vus = 0;           // (compteurs 8 bits : seule la différence compte)
uint8_t nb_mesures = 0;
uint16_t somme_mesures = 0;
uint8_t derniere_mesure = 0;
uint8_t nb_ecartees = 0;             // mesures écartées d'affilée (battement mal détecté)
unsigned long t_derniere_mesure = 0;

// Texte en français ou en anglais selon la langue choisie (ET6.5) ; les textes restent en Flash
#define TXT(fr, en) (langue == LANGUE_FR ? F(fr) : F(en))

// =====================================================================================
//                                   INTERRUPTIONS
// =====================================================================================
// Échantillonnage à 100 Hz, régulier même pendant l'envoi d'une image à un écran (~25 ms)
ISR(TIMER1_COMPA_vect) {
  uint8_t suivant = (file_tete + 1) & (FILE_LEN - 1);
  int v = analogRead(PIN_CAPTEUR);
  if (suivant != file_queue) {
    file_ech[file_tete] = v;
    file_tete = suivant;
  }
}

// Encodeur sur les broches 9 et 10 (port B) : interruption "pin change", aucun cran perdu
// même pendant le dessin d'un écran (D11, D12 et D13 sont aussi sur le port B mais leur
// interruption n'est pas activée)
ISR(PCINT0_vect) {
  uint8_t etat = (PINB >> 1) & 0x03;   // bit0 = broche 9 (A), bit1 = broche 10 (B)
  enc_transitions += (int8_t)pgm_read_byte(&TABLE_QUADRATURE[(enc_etat_prec << 2) | etat]);
  enc_etat_prec = etat;
}

void demarrer_timer1() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);   // mode CTC, prédiviseur 64
  TCNT1 = 0;
  OCR1A = 16000000UL / 64 / (1000 / PERIODE_ECH_MS) - 1; // 2499 -> 100 Hz
  TIMSK1 = (1 << OCIE1A);
  interrupts();
}

// D9 et D10 sont aussi les sorties PWM du Timer1 ; comme TCCR1A = 0 (aucune sortie PWM),
// ce sont de simples entrées : pas de conflit avec l'échantillonnage.
void demarrer_encodeur() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_BOUTON, INPUT_PULLUP);           // évite une entrée "en l'air"
  enc_etat_prec = (PINB >> 1) & 0x03;
  PCMSK0 |= (1 << PCINT1) | (1 << PCINT2);         // PCINT1 = D9, PCINT2 = D10
  PCICR |= (1 << PCIE0);
}

bool lire_echantillon(int &v) {
  if (file_queue == file_tete) return false;
  v = file_ech[file_queue];
  file_queue = (file_queue + 1) & (FILE_LEN - 1);
  return true;
}

int lire_encodeur() {                // nombre de crans tournés depuis le dernier appel
  noInterrupts();
  int8_t t = enc_transitions;
  int crans = t / TRANSITIONS_CRAN;
  enc_transitions = t - crans * TRANSITIONS_CRAN;
  interrupts();
  return crans * SENS_ENCODEUR;
}

// =====================================================================================
//                         FS1 : CALCUL DU BPM
// =====================================================================================
void ajouter_intervalle(unsigned int dt) {
  intervalles[index_iv] = dt;
  index_iv = (index_iv + 1) % NB_MOYENNE;
  if (nb_iv < NB_MOYENNE) nb_iv++;
}

int calculer_bpm() {                 // 60000 / moyenne des intervalles, arrondi
  if (nb_iv == 0) return 0;
  unsigned long somme = 0;
  for (uint8_t i = 0; i < nb_iv; i++) somme += intervalles[i];
  return (60000UL * nb_iv + somme / 2) / somme;
}

void vider_intervalles() {
  nb_iv = 0;
  index_iv = 0;
}

// =====================================================================================
//                   FS1 : TRAITEMENT D'UN ÉCHANTILLON
// =====================================================================================
void graphe_ajouter(float v);        // définie dans la partie "ÉCRAN 2"

void traiter_echantillon(int brut) {
  n_echantillons++;
  unsigned long t = n_echantillons * PERIODE_ECH_MS;
  if (INVERSER_SIGNAL) brut = 1023 - brut;

  if (passe_bas < 0) {               // premier échantillon : initialise les filtres
    passe_bas = brut;
    base = brut;
  }

  // 1) Passe-bas
  passe_bas += ALPHA_PASSE_BAS * (brut - passe_bas);
  // 2) Passe-haut : on retire la ligne de base
  base += ALPHA_BASE * (passe_bas - base);
  float alternatif = passe_bas - base;

  // 3) Normalisation entre 0 et 1 : min et max sur les 2 dernières secondes,
  //    puis signal_norm = (signal - min) / (max - min) : 0 = creux, 1 = sommet du battement
  if (alternatif < min_courant) min_courant = alternatif;
  if (alternatif > max_courant) max_courant = alternatif;
  if (++ech_dans_bloc >= TAILLE_BLOC) {        // bloc de 0,2 s terminé : on le garde
    bloc_min[bloc_index] = (int16_t)(min_courant * 8);
    bloc_max[bloc_index] = (int16_t)(max_courant * 8);
    bloc_index = (bloc_index + 1) % NB_BLOCS;  // remplace le bloc le plus ancien
    if (bloc_nb < NB_BLOCS) bloc_nb++;
    ech_dans_bloc = 0;
    min_courant = 1e9;
    max_courant = -1e9;
  }
  float mini = min_courant, maxi = max_courant;
  for (uint8_t i = 0; i < bloc_nb; i++) {
    if (bloc_min[i] / 8.0f < mini) mini = bloc_min[i] / 8.0f;
    if (bloc_max[i] / 8.0f > maxi) maxi = bloc_max[i] / 8.0f;
  }
  float amplitude = maxi - mini;

  //    Doigt présent si l'écart max - min est assez grand et nettement plus grand que le bruit
  bruit += ALPHA_BRUIT * (fabs(brut - passe_bas) - bruit);   // ce que le passe-bas a retiré
  bool amplitude_ok = (bloc_nb >= 3) && (amplitude > AMPLITUDE_MIN) && (amplitude > SIGNAL_SUR_BRUIT * bruit);
  signal_norm = amplitude_ok ? (alternatif - mini) / amplitude : 0;

  // 4) Détection des battements
  if (arme && signal_norm > SEUIL_HAUT) {
    arme = false;
    unsigned long dt = t - t_dernier_battement;
    unsigned long dt_min = INTERVALLE_MIN_MS;
    if (nb_reguliers > 0 && REFRACTAIRE_RELATIF * intervalle_prec > dt_min) {
      dt_min = REFRACTAIRE_RELATIF * intervalle_prec;
    }
    if (dt >= dt_min) {              // sinon : rebond / onde dicrote, ignoré
      t_coeur = millis();            // fait battre le cœur de l'écran 1
      nb_battements++;               // FS4 : un beep à jouer
      if (t_dernier_battement != 0 && dt <= INTERVALLE_MAX_MS) {
        // intervalle plausible : est-il régulier par rapport au précédent ?
        if (nb_reguliers > 0 && fabs((float)dt - intervalle_prec) > ECART_MAX * intervalle_prec) {
          nb_reguliers = 1;          // irrégulier : on recommence à compter et à moyenner
          vider_intervalles();
        } else if (nb_reguliers < 255) {
          nb_reguliers++;
        }
        ajouter_intervalle(dt);
        intervalle_prec = dt;
        dernier_intervalle = dt;     // FS6 : une nouvelle mesure (BPM instantané = 60000 / dt)
        nb_intervalles_total++;
      } else {
        nb_reguliers = 0;
        vider_intervalles();
      }
      t_dernier_battement = t;
    }
  } else if (!arme && signal_norm < SEUIL_BAS) {
    arme = true;
  }
  if (!amplitude_ok || t - t_dernier_battement > INTERVALLE_MAX_MS) {
    nb_reguliers = 0;
    vider_intervalles();
  }
  battements_ok = amplitude_ok && nb_reguliers >= NB_INTERVALLES_OK;
  bpm = battements_ok ? calculer_bpm() : 0;

  // 5) Graphique (toujours mémorisé, affiché seulement si battements_ok)
  graphe_ajouter(signal_norm);

  // 6) Traceur série (coupé en mode SIMULER_BPM ou TRACE_BEEPS pour garder lisibles les messages)
  if (!SIMULER_BPM && !TRACE_BEEPS && ++compteur_serie >= AFFICHER_1_SUR) {   // coupé en mode test
    compteur_serie = 0;
    Serial.print(F("signal:"));
    Serial.print(signal_norm, 3);
    Serial.print(F(",battements_ok:"));
    Serial.print(battements_ok ? 1.1 : -0.1);
    Serial.print(F(",bpm_div100:"));        // BPM / 100 (0.72 = 72 bpm) pour rester sur la même échelle
    Serial.print(bpm / 100.0, 2);
    Serial.println(F(",haut:1.2,bas:-0.2"));
  }
}

// =====================================================================================
//                 FS3 : VOYANTS D'ÉTAT DE SANTÉ (ET3.1 à ET3.3)
// =====================================================================================
bool bpm_conforme(int b);             // définie dans la partie "ÉCRAN 1" (ET5.1)

// ET3.1 : une seule fonction pilote les trois sorties. Elle éteint d'abord les trois voyants,
// puis allume au plus UN seul : deux voyants ne peuvent jamais être allumés en même temps,
// même pendant une fraction de seconde.
void allumer_voyant(Voyant v) {
  digitalWrite(PIN_VOYANT_JAUNE, LOW);
  digitalWrite(PIN_VOYANT_VERT, LOW);
  digitalWrite(PIN_VOYANT_ROUGE, LOW);
  if (v == JAUNE) digitalWrite(PIN_VOYANT_JAUNE, HIGH);
  if (v == VERT)  digitalWrite(PIN_VOYANT_VERT, HIGH);
  if (v == ROUGE) digitalWrite(PIN_VOYANT_ROUGE, HIGH);
  voyant = v;
}

// ET3.2 + ET3.3 : choisit le voyant à partir du BPM
//   b hors de [30 ; 200] (0 = pas de battements réguliers) -> AUCUN (valeur aberrante)
//   sinon jaune / vert / rouge, avec une hystérésis de HYSTERESIS_BPM autour de 60 et 100 :
//   la frontière s'éloigne du voyant actuel, il faut la dépasser nettement pour changer
//   (ex. vert -> rouge à 103 bpm, rouge -> vert à 97 bpm)
Voyant classer_bpm(int b, Voyant actuel) {
  if (b < BPM_CONFORME_MIN || b > BPM_CONFORME_MAX) return AUCUN;
  int lim_bas = BPM_NORMAL_MIN;      // en dessous : jaune
  int lim_haut = BPM_NORMAL_MAX;     // au-dessus : rouge
  if (actuel == JAUNE) lim_bas += HYSTERESIS_BPM;
  if (actuel == VERT)  { lim_bas -= HYSTERESIS_BPM; lim_haut += HYSTERESIS_BPM; }
  if (actuel == ROUGE) lim_haut -= HYSTERESIS_BPM;
  if (b < lim_bas)  return JAUNE;
  if (b > lim_haut) return ROUGE;
  return VERT;
}

void serie_nom_voyant(Voyant v) {    // textes en Flash (F) : la RAM est comptée
  if (v == JAUNE)      Serial.print(F("JAUNE (faible)"));
  else if (v == VERT)  Serial.print(F("VERT (normal)"));
  else if (v == ROUGE) Serial.print(F("ROUGE (eleve)"));
  else                 Serial.print(F("AUCUN (aberrant / pas de mesure)"));
}

// Relit l'état réel des 3 broches : preuve de ET3.1 pendant les tests (doit valoir 0 ou 1)
uint8_t nb_voyants_allumes() {
  return digitalRead(PIN_VOYANT_JAUNE) + digitalRead(PIN_VOYANT_VERT) + digitalRead(PIN_VOYANT_ROUGE);
}

void demarrer_voyants() {
  pinMode(PIN_VOYANT_JAUNE, OUTPUT);
  pinMode(PIN_VOYANT_VERT, OUTPUT);
  pinMode(PIN_VOYANT_ROUGE, OUTPUT);
  allumer_voyant(AUCUN);
  if (TEST_VOYANTS) {                // test du câblage : jaune, vert, rouge, un par un
    allumer_voyant(JAUNE); delay(500);
    allumer_voyant(VERT);  delay(500);
    allumer_voyant(ROUGE); delay(500);
    allumer_voyant(AUCUN);
  }
}

// Mode test : lit un nombre tapé dans le moniteur série (sans bloquer la boucle)
void lire_bpm_simule() {
  static int valeur = 0;
  static bool negatif = false, chiffres = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '-') negatif = true;
    else if (c >= '0' && c <= '9') {
      valeur = valeur * 10 + (c - '0');
      chiffres = true;
      if (valeur > 9999) valeur = 9999;
    } else if (c == '\n' || c == '\r') {
      if (chiffres) {
        bpm_simule = negatif ? -1 : valeur;
        Serial.print(F("[FS3] BPM simule : "));
        if (bpm_simule < 0) Serial.println(F("arret, retour au capteur"));
        else Serial.println(bpm_simule);
      }
      valeur = 0; negatif = false; chiffres = false;
    }
  }
}

// Appelée à chaque tour de boucle : ne touche aux sorties que si le voyant doit changer.
// Chaque changement est tracé dans le moniteur série (preuve pour le rapport).
void mettre_a_jour_voyants() {
  int b = bpm_conforme(bpm) ? bpm : 0;   // même règle que l'écran 1 (tiret = aucun voyant)
  if (SIMULER_BPM) {
    lire_bpm_simule();
    if (bpm_simule >= 0) b = bpm_simule;
  }
  Voyant v = classer_bpm(b, voyant);
  if (v == voyant) return;
  allumer_voyant(v);
  Serial.print(F("[FS3] t="));
  Serial.print(millis());
  Serial.print(F(" ms  bpm="));
  Serial.print(b);
  Serial.print(F("  voyant="));
  serie_nom_voyant(v);
  Serial.print(F("  nb_allumes="));
  Serial.println(nb_voyants_allumes());
}

// =====================================================================================
//                 FS4 : BEEPER LE RYTHME CARDIAQUE (ET4.1 à ET4.3)
// =====================================================================================
#define NIVEAU_APPUI_BEEP    (BOUTON_BEEP_ACTIF_HAUT ? HIGH : LOW)

// ET4.2 : le son dépend du même classement que les voyants (mêmes seuils 60 / 100 bpm, même
// hystérésis) : le beeper et les voyants donnent toujours la même information
unsigned int frequence_beep(Voyant v) {
  if (v == JAUNE) return FREQ_GRAVE;
  if (v == ROUGE) return FREQ_AIGU;
  return FREQ_MODERE;
}

void serie_nom_son(Voyant v) {
  if (v == JAUNE)      Serial.print(F("GRAVE"));
  else if (v == ROUGE) Serial.print(F("AIGU"));
  else                 Serial.print(F("MODERE"));
}

// ET4.3 : interruption à chaque changement de niveau de la broche du bouton.
// Anti-rebond : seul le PREMIER front après ANTI_REBOND_BEEP_MS de calme compte (les rebonds qui
// suivent sont ignorés) : l'appui agit dès le premier contact, sans attendre la fin des rebonds.
// Temps de réaction : quelques µs ; au pire ~0,1 ms si l'appui tombe pendant l'interruption du
// Timer1 (analogRead du capteur). Le programme principal (dessin d'un écran, ~25 ms) n'est
// jamais attendu.
ISR(BOUTON_BEEP_VECT) {
  uint16_t t0 = micros();
  unsigned long t = millis();
  if (t - t_front_beep >= ANTI_REBOND_BEEP_MS) {
    bouton_beep_appuye = !bouton_beep_appuye;
    if (bouton_beep_appuye) {                    // appui (le relâchement ne fait rien)
      beep_actif = !beep_actif;
      if (!beep_actif) {                         // coupure immédiate du son en cours :
        TIMSK2 &= ~_BV(OCIE2A);                  //   arrête l'interruption de tone() (Timer2)...
        digitalWrite(PIN_BUZZER, LOW);           //   ...et met le buzzer au repos
      }
      duree_isr_us = (uint16_t)micros() - t0;    // mesure pour le rapport
      bouton_beep_evenement = true;
    }
  }
  t_front_beep = t;
}

void demarrer_beeper() {
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BOUTON_BEEP, BOUTON_BEEP_ACTIF_HAUT ? INPUT : INPUT_PULLUP);
  if (TEST_BEEPER) {                 // test du câblage : chaque son avec le voyant correspondant
    allumer_voyant(JAUNE); tone(PIN_BUZZER, FREQ_GRAVE);  delay(300);
    allumer_voyant(VERT);  tone(PIN_BUZZER, FREQ_MODERE); delay(300);
    allumer_voyant(ROUGE); tone(PIN_BUZZER, FREQ_AIGU);   delay(300);
    noTone(PIN_BUZZER);
    allumer_voyant(AUCUN);
  }
  bouton_beep_appuye = (digitalRead(PIN_BOUTON_BEEP) == NIVEAU_APPUI_BEEP);
  *digitalPinToPCMSK(PIN_BOUTON_BEEP) |= _BV(digitalPinToPCMSKbit(PIN_BOUTON_BEEP));
  PCIFR = _BV(digitalPinToPCICRbit(PIN_BOUTON_BEEP));   // efface une demande déjà en attente
  PCICR |= _BV(digitalPinToPCICRbit(PIN_BOUTON_BEEP));
}

// Sécurité : si un front a été manqué, on recale l'état du bouton quand il est stable (sans
// basculer le beeper). Écrit chaque appui dans le moniteur série (preuve de ET4.3).
// Renvoie true si l'état a changé (l'icône de l'écran 2 doit être redessinée).
bool surveiller_bouton_beep() {
  noInterrupts();
  if (millis() - t_front_beep > 2 * ANTI_REBOND_BEEP_MS) {
    bouton_beep_appuye = (digitalRead(PIN_BOUTON_BEEP) == NIVEAU_APPUI_BEEP);
  }
  bool evenement = bouton_beep_evenement;
  bouton_beep_evenement = false;
  bool actif = beep_actif;
  uint16_t d = duree_isr_us;
  interrupts();
  if (!evenement) return false;
  Serial.print(F("[FS4] t="));
  Serial.print(millis());
  if (actif) {
    Serial.println(F(" ms  bouton : beeps REMIS EN ROUTE"));
  } else {
    Serial.print(F(" ms  bouton : beeps INTERROMPUS (son coupe "));
    Serial.print(d);
    Serial.println(F(" us apres l'entree dans l'interruption)"));
  }
  return true;
}

// ET4.1 : un beep à chaque battement détecté par FS1 (nb_battements augmente à chaque battement) :
//   les beeps suivent le vrai rythme du cœur, pas une minuterie qui pourrait se décaler.
// En mode SIMULER_BPM (pas de vrai battement), les beeps sont cadencés à 60000 / BPM ms.
// Pas de beep si la valeur est aberrante ou sans doigt (voyant AUCUN), comme pour ET3.3.
// tone(broche, fréquence, durée) ne bloque pas : le Timer2 arrête le son tout seul après 100 ms.
void mettre_a_jour_beeper() {
  unsigned long maintenant = millis();
  bool battement;
  if (SIMULER_BPM && bpm_simule >= 0) {
    nb_battements_beepes = nb_battements;        // les vrais battements sont ignorés
    battement = bpm_simule > 0 && (maintenant - t_dernier_beep >= 60000UL / bpm_simule);
  } else {
    battement = (nb_battements != nb_battements_beepes);
    nb_battements_beepes = nb_battements;
  }
  if (!battement || voyant == AUCUN) return;

  unsigned long intervalle = maintenant - t_dernier_beep;
  t_dernier_beep = maintenant;
  unsigned int f = frequence_beep(voyant);
  noInterrupts();                    // le bouton ne peut pas couper le son entre le test et tone()
  bool actif = beep_actif;
  if (actif) tone(PIN_BUZZER, f, DUREE_BEEP_MS);
  interrupts();

  if (TRACE_BEEPS) {                 // preuve pour le rapport : intervalle entre 2 beeps = rythme
    Serial.print(F("[FS4] t="));
    Serial.print(maintenant);
    Serial.print(F(" ms  beep "));
    serie_nom_son(voyant);
    Serial.print(' ');
    Serial.print(f);
    Serial.print(F(" Hz  intervalle="));
    Serial.print(intervalle);
    Serial.print(F(" ms"));
    if (intervalle > 0 && intervalle < 5000) {
      Serial.print(F(" (="));
      Serial.print((60000UL + intervalle / 2) / intervalle);
      Serial.print(F(" bpm)"));
    }
    Serial.print(F("  bpm affiche="));
    Serial.print(SIMULER_BPM && bpm_simule >= 0 ? bpm_simule : bpm);
    Serial.print(F("  voyant="));
    serie_nom_voyant(voyant);
    Serial.println(actif ? F("") : F("  [beeps coupes : silence]"));
  }
}

// Petite icône de haut-parleur (11 x 8 px) : barrée quand les beeps sont coupés
void dessiner_haut_parleur(int x, int y, bool actif) {
  ecran.fillRect(x, y + 2, 3, 4, SSD1306_WHITE);
  ecran.fillTriangle(x + 2, y + 3, x + 5, y, x + 5, y + 7, SSD1306_WHITE);
  ecran.fillTriangle(x + 2, y + 4, x + 5, y, x + 5, y + 7, SSD1306_WHITE);
  if (actif) {                       // ondes sonores
    ecran.drawFastVLine(x + 7, y + 2, 4, SSD1306_WHITE);
    ecran.drawFastVLine(x + 9, y + 1, 6, SSD1306_WHITE);
  } else {                           // croix
    ecran.drawLine(x + 7, y + 1, x + 10, y + 6, SSD1306_WHITE);
    ecran.drawLine(x + 7, y + 6, x + 10, y + 1, SSD1306_WHITE);
  }
}

// =====================================================================================
//                 MODULE RTC DS1302 : COMMUNICATION (SANS BIBLIOTHÈQUE)
// =====================================================================================
// Le DS1302 communique sur 3 fils : RST (active la communication), CLK (horloge) et DAT (données,
// dans les deux sens). Les octets passent bit par bit, bit de poids faible en premier. Les valeurs
// sont codées en BCD : 0x29 veut dire 29 (un chiffre par groupe de 4 bits).

uint8_t bcd_vers_dec(uint8_t b) { return 10 * (b >> 4) + (b & 0x0F); }
uint8_t dec_vers_bcd(uint8_t d) { return ((d / 10) << 4) | (d % 10); }

void rtc_debut() {                   // début d'un échange
  digitalWrite(RTC_CLK, LOW);
  digitalWrite(RTC_RST, HIGH);
  delayMicroseconds(4);
}

void rtc_fin() {                     // fin d'un échange
  digitalWrite(RTC_RST, LOW);
  delayMicroseconds(4);
}

// Envoie un octet ; si lecture_apres, DAT repasse en entrée pour que le module réponde
void rtc_envoyer(uint8_t v, bool lecture_apres) {
  pinMode(RTC_DAT, OUTPUT);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(RTC_DAT, (v >> i) & 1);
    delayMicroseconds(1);
    digitalWrite(RTC_CLK, HIGH);
    delayMicroseconds(1);
    if (lecture_apres && i == 7) {
      pinMode(RTC_DAT, INPUT);
    } else {
      digitalWrite(RTC_CLK, LOW);
      delayMicroseconds(1);
    }
  }
}

// Reçoit un octet (le module envoie chaque bit sur le front descendant de CLK)
uint8_t rtc_recevoir() {
  uint8_t v = 0;
  pinMode(RTC_DAT, INPUT);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(RTC_CLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(RTC_CLK, LOW);
    delayMicroseconds(1);
    v |= digitalRead(RTC_DAT) << i;
  }
  return v;
}

void rtc_ecrire_registre(uint8_t reg, uint8_t valeur) {
  rtc_debut();
  rtc_envoyer(0x80 | (reg << 1), false);
  rtc_envoyer(valeur, false);
  rtc_fin();
}

uint8_t rtc_lire_registre(uint8_t reg) {
  rtc_debut();
  rtc_envoyer(0x81 | (reg << 1), true);
  uint8_t v = rtc_recevoir();
  rtc_fin();
  return v;
}

// Lit les 7 registres de l'heure d'un seul coup (mode "burst", commande 0xBF)
void rtc_lire(Heure &h) {
  uint8_t r[7];
  rtc_debut();
  rtc_envoyer(0xBF, true);
  for (uint8_t i = 0; i < 7; i++) r[i] = rtc_recevoir();
  rtc_fin();
  h.sec  = bcd_vers_dec(r[0] & 0x7F);              // bit 7 = horloge arrêtée
  h.min  = bcd_vers_dec(r[1] & 0x7F);
  h.hr   = bcd_vers_dec(r[2] & 0x3F);              // mode 24 h
  h.date = bcd_vers_dec(r[3] & 0x3F);
  h.mon  = bcd_vers_dec(r[4] & 0x1F);
  h.jour = r[5] & 0x07;
  h.yr   = 2000 + bcd_vers_dec(r[6]);
}

// Écrit l'heure d'un seul coup (mode "burst", commande 0xBE) ; démarre aussi l'horloge (bit 7 à 0)
void rtc_ecrire(const Heure &h) {
  rtc_debut();
  rtc_envoyer(0xBE, false);
  rtc_envoyer(dec_vers_bcd(h.sec), false);
  rtc_envoyer(dec_vers_bcd(h.min), false);
  rtc_envoyer(dec_vers_bcd(h.hr), false);
  rtc_envoyer(dec_vers_bcd(h.date), false);
  rtc_envoyer(dec_vers_bcd(h.mon), false);
  rtc_envoyer(h.jour, false);
  rtc_envoyer(dec_vers_bcd(h.yr - 2000), false);
  rtc_envoyer(0x00, false);                        // registre de protection : écriture autorisée
  rtc_fin();
}

// Jour de la semaine (0 = dimanche) d'une date, méthode de Sakamoto
uint8_t jour_semaine(int a, uint8_t m, uint8_t j) {
  static const uint8_t t[] PROGMEM = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3) a -= 1;
  return (a + a / 4 - a / 100 + a / 400 + pgm_read_byte(&t[m - 1]) + j) % 7;
}

// Règle le module sur la date et l'heure de la compilation du programme (__DATE__ et __TIME__)
void regler_heure_compilation() {
  char d[12], h[9];                              // textes gardés en Flash, copiés seulement ici
  strcpy_P(d, PSTR(__DATE__));                   // ex. "Oct  9 2026"
  strcpy_P(h, PSTR(__TIME__));                   // ex. "18:29:17"
  const char *mois = PSTR("JanFebMarAprMayJunJulAugSepOctNovDec");
  uint8_t m = 1;
  for (uint8_t i = 0; i < 12; i++) {
    if (d[0] == pgm_read_byte(&mois[3 * i]) && d[1] == pgm_read_byte(&mois[3 * i + 1])
        && d[2] == pgm_read_byte(&mois[3 * i + 2])) m = i + 1;
  }
  Heure t;
  t.date = atoi(d + 4);
  t.yr   = atoi(d + 7);
  t.mon  = m;
  t.hr   = atoi(h);
  t.min  = atoi(h + 3);
  t.sec  = atoi(h + 6);
  t.jour = jour_semaine(t.yr, t.mon, t.date) + 1;
  rtc_ecrire(t);
}

void demarrer_rtc() {
  digitalWrite(RTC_RST, LOW);
  pinMode(RTC_RST, OUTPUT);
  digitalWrite(RTC_CLK, LOW);
  pinMode(RTC_CLK, OUTPUT);
  pinMode(RTC_DAT, INPUT);

  rtc_ecrire_registre(7, 0x00);                  // enlève la protection en écriture
  Heure t;
  rtc_lire(t);
  // On ne règle l'heure que si elle n'a jamais été réglée : grâce à la pile du module,
  // elle n'est pas perdue quand l'Arduino est débranché
  if (REGLER_HEURE || t.yr < 2020 || t.yr >= 2100 || t.mon < 1 || t.mon > 12) {
    regler_heure_compilation();
  } else {
    uint8_t sec = rtc_lire_registre(0);
    if (sec & 0x80) rtc_ecrire_registre(0, sec & 0x7F);   // horloge arrêtée : on la redémarre
  }
  rtc_ecrire_registre(7, 0x80);                  // protège l'heure contre une écriture accidentelle
}

// Lit l'heure sur le module une fois par seconde (une lecture dure moins d'1 ms)
void lire_heure() {
  if (!heure_jamais_lue && millis() - t_lecture_rtc < 1000) return;
  t_lecture_rtc = millis();
  heure_jamais_lue = false;
  rtc_lire(heure);
  // Un module absent ou mal branché renvoie des valeurs impossibles
  heure_ok = heure.yr >= 2020 && heure.yr < 2100 && heure.mon >= 1 && heure.mon <= 12
             && heure.hr < 24 && heure.min < 60 && heure.sec < 60;
}

// =====================================================================================
//                  FS5 — ÉCRAN 1 (0x3C) : HEURE + CŒUR + BPM (ET5.1)
// =====================================================================================
// Disposition (128 x 64), comme sur la maquette du sujet :
//   lignes  0..27 : heure HH:MM en police x4 (24 x 32 px par caractère)
//   ligne  33     : trait de séparation
//   lignes 40..63 : cœur à gauche, BPM en police x3, "bpm" en police x2

// ET5.1 : la valeur est affichée seulement si des battements réguliers sont détectés (doigt posé)
// et si elle est dans la plage conforme ; sinon on affiche un tiret
bool bpm_conforme(int b) {
  return battements_ok && b >= BPM_CONFORME_MIN && b <= BPM_CONFORME_MAX;
}

void dessiner_coeur(int x, int y, int r) {     // cœur = deux disques + un triangle
  ecran.fillCircle(x - r, y, r, SSD1306_WHITE);
  ecran.fillCircle(x + r, y, r, SSD1306_WHITE);
  ecran.fillTriangle(x - 2 * r, y + 1, x + 2 * r, y + 1, x, y + 2 * r + 2, SSD1306_WHITE);
}

void print2(uint8_t n) {             // nombre sur 2 chiffres (ex. 7 -> "07")
  if (n < 10) ecran.print('0');
  ecran.print(n);
}

void dessiner_ecran1(bool coeur_plein) {
  ecran.clearDisplay();
  ecran.setTextColor(SSD1306_WHITE);

  // ----- Heure en grand, centrée : "HH:MM" = 5 x 24 px - 4 px d'espace final = 116 px -----
  ecran.setTextSize(4);
  ecran.setCursor(6, 0);
  if (heure_ok) {
    print2(heure.hr);
    // Les ":" clignotent (visibles les secondes paires) : montre que l'horloge tourne
    ecran.print((!CLIGNOTER_2PTS || heure.sec % 2 == 0) ? ':' : ' ');
    print2(heure.min);
  } else {
    ecran.print(F("--:--"));                     // module absent ou mal branché
  }

  // ----- Trait de séparation -----
  ecran.drawFastHLine(8, 33, 112, SSD1306_WHITE);

  // ----- Cœur + BPM -----
  dessiner_coeur(18, 46, coeur_plein ? 6 : 4);   // le cœur grossit à chaque battement

  ecran.setTextSize(3);                          // 18 x 24 px par chiffre
  if (bpm_conforme(bpm)) {
    ecran.setCursor((bpm >= 100) ? 40 : 58, 40); // aligné à droite, juste avant "bpm"
    ecran.print(bpm);
  } else {
    ecran.setCursor(76, 40);
    ecran.print('-');                            // ET5.1 : tiret
  }
  ecran.setTextSize(2);                          // 12 x 16 px par caractère
  ecran.setCursor(94, 47);                       // même ligne de base que les chiffres du BPM
  ecran.print(F("bpm"));

  ecran.envoyer(ADRESSE_ECRAN1);                 // envoi des 1024 octets (~25 ms)
}

// Ne redessine l'écran 1 que si la seconde, le BPM affiché ou l'état du cœur a changé
// (renvoie true si l'écran a été redessiné)
bool mettre_a_jour_ecran1() {
  if (!ecran1_present) return false;
  lire_heure();
  int b = bpm_conforme(bpm) ? bpm : 0;
  bool plein = battements_ok && (millis() - t_coeur < DUREE_COEUR_MS);
  if (b == bpm_affiche && plein == coeur_affiche && heure.sec == seconde_affichee) return false;
  dessiner_ecran1(plein);
  bpm_affiche = b;
  coeur_affiche = plein;
  seconde_affichee = heure.sec;
  return true;
}

// =====================================================================================
//                FS5 — ÉCRAN 2 (0x3D) : GRAPHIQUE DU PPG (ET5.2 à ET5.4)
// =====================================================================================
// Disposition (128 x 64), police 6 x 8 px :
//   ligne du haut  : "PPG", base de temps en ms par pixel, icône du beeper (FS4), "Amp"
//   courbe         : 100 px de large (x = 0..99), 40 px de haut ; la plus récente à droite
//   axe vertical   : à droite, graduations 0.0 / 0.5 / 1.0 (amplitude normalisée, sans unité)
//   axe horizontal : flèche vers la droite, une graduation tous les 25 pixels
//   ligne du bas   : durée d'une graduation ("1s/25px"), nom de l'axe "t (s)"

uint8_t y_ecran(float v) {           // valeur normalisée (0..1) -> ligne de pixel
  if (v > 1) v = 1;
  if (v < 0) v = 0;
  return (uint8_t)(GRAPHE_Y_BAS - v * (GRAPHE_H - 1) + 0.5f);   // 0 en bas, 1 en haut
}

void graphe_ajouter(float v) {       // un pixel = moyenne de CALIBRES[calibre] échantillons
  cumul_pixel += v;
  if (++nb_cumul < CALIBRES[calibre]) return;
  graphe[graphe_index] = y_ecran(cumul_pixel / nb_cumul);
  graphe_index = (graphe_index + 1) % GRAPHE_L;
  if (graphe_nb < GRAPHE_L) graphe_nb++;
  cumul_pixel = 0;
  nb_cumul = 0;
}

// ET5.4 : l'encodeur change le calibre (la base de temps), comme sur un oscilloscope
void changer_echelle(int delta) {
  int c = (int)calibre + delta;
  if (c < 0) c = 0;
  if (c > (int)NB_CALIBRES - 1) c = NB_CALIBRES - 1;
  if (c == calibre) return;
  calibre = c;
  graphe_nb = 0;                     // l'ancienne courbe n'est plus à la bonne échelle
  graphe_index = 0;
  cumul_pixel = 0;
  nb_cumul = 0;
  Serial.print(F("Base de temps : "));             // trace pour le rapport (ET5.4)
  Serial.print(CALIBRES[calibre] * PERIODE_ECH_MS);
  Serial.println(F(" ms/px"));
}

// Écrit une durée en ms sous la forme "0.25s", "0.5s", "1s", "2s", "5s"
void print_secondes(unsigned int ms) {
  ecran.print(ms / 1000);
  unsigned int reste = ms % 1000;
  if (reste != 0) {
    ecran.print('.');
    if (reste % 100 == 0) {
      ecran.print(reste / 100);
    } else {
      if (reste < 100) ecran.print('0');
      ecran.print(reste / 10);
    }
  }
  ecran.print('s');
}

void dessiner_graphe() {
  unsigned int ms_px = CALIBRES[calibre] * PERIODE_ECH_MS;   // ET5.3 : ms par pixel
  unsigned int ms_grad = ms_px * PX_PAR_GRAD;                // ET5.3 : durée d'une graduation

  // (l'écran est déjà effacé par dessiner_ecran2)

  // ----- Ligne du haut : titre, base de temps par pixel, beeper, nom de l'axe vertical -----
  ecran.setCursor(0, 0);
  ecran.print(F("PPG"));
  ecran.setCursor(30, 0);
  ecran.print(ms_px);
  ecran.print(F("ms/px"));
  dessiner_haut_parleur(84, 0, beep_actif);      // FS4 (ET4.3) : beeps actifs ou coupés
  ecran.setCursor(AXE_X + 5, 0);
  ecran.print(F("Amp"));

  // ----- Axe vertical à droite : flèche vers le haut + graduations 0.0 / 0.5 / 1.0 -----
  ecran.drawFastVLine(AXE_X, GRAPHE_Y_HAUT - 2, AXE_Y - GRAPHE_Y_HAUT + 3, SSD1306_WHITE);
  ecran.drawLine(AXE_X - 2, GRAPHE_Y_HAUT + 1, AXE_X, GRAPHE_Y_HAUT - 2, SSD1306_WHITE);
  ecran.drawLine(AXE_X + 2, GRAPHE_Y_HAUT + 1, AXE_X, GRAPHE_Y_HAUT - 2, SSD1306_WHITE);
  for (uint8_t g = 0; g <= 2; g++) {
    uint8_t y = y_ecran(g * 0.5f);
    ecran.drawFastHLine(AXE_X, y, 3, SSD1306_WHITE);
    ecran.setCursor(AXE_X + 5, y - 3);
    ecran.print(g == 0 ? F("0.0") : (g == 1 ? F("0.5") : F("1.0")));
  }

  // ----- Axe horizontal (temps) : flèche vers la droite + une graduation tous les 25 px -----
  ecran.drawFastHLine(0, AXE_Y, GRAPHE_L, SSD1306_WHITE);
  ecran.drawLine(GRAPHE_L - 3, AXE_Y - 2, GRAPHE_L, AXE_Y, SSD1306_WHITE);
  ecran.drawLine(GRAPHE_L - 3, AXE_Y + 2, GRAPHE_L, AXE_Y, SSD1306_WHITE);
  for (int x = GRAPHE_L - PX_PAR_GRAD; x >= 0; x -= PX_PAR_GRAD) {
    ecran.drawFastVLine(x, AXE_Y, 3, SSD1306_WHITE);
  }

  // ----- Ligne du bas : durée d'une graduation, nom de l'axe -----
  ecran.setCursor(0, 56);
  print_secondes(ms_grad);
  ecran.print('/');
  ecran.print(PX_PAR_GRAD);
  ecran.print(F("px"));
  ecran.setCursor(GRAPHE_L - 30, 56);
  ecran.print(F("t (s)"));

  // ----- Courbe (ET5.2) -----
  if (battements_ok) {
    // La plus récente à droite, elle défile vers la gauche
    int x0 = GRAPHE_L - graphe_nb;
    for (int k = 0; k < graphe_nb; k++) {
      uint8_t y = graphe[(graphe_index + GRAPHE_L - graphe_nb + k) % GRAPHE_L];
      if (k == 0) {
        ecran.drawPixel(x0, y, SSD1306_WHITE);
      } else {
        uint8_t y_prec = graphe[(graphe_index + GRAPHE_L - graphe_nb + k - 1) % GRAPHE_L];
        ecran.drawLine(x0 + k - 1, y_prec, x0 + k, y, SSD1306_WHITE);
      }
    }
  } else {
    // Pas de battement régulier : pas de courbe, on guide l'utilisateur
    ecran.setCursor(5, 22);
    ecran.print(TXT("Aucun battement", "No heartbeat"));
    ecran.setCursor(8, 34);
    ecran.print(TXT("Posez le doigt", "Place your finger"));
  }
}

// =====================================================================================
//            FS6 — EEPROM : LIRE / ÉCRIRE / EFFACER LES ENREGISTREMENTS (ET6.1)
// =====================================================================================
// L'EEPROM garde son contenu sans alimentation. EEPROM.update() n'écrit que si la valeur change
// (l'EEPROM supporte ~100 000 écritures par octet).
void charger_eeprom() {
  langue = EEPROM.read(ADDR_LANGUE);
  if (langue > LANGUE_EN) {                      // EEPROM neuve (0xFF) : français par défaut
    langue = LANGUE_FR;
    EEPROM.update(ADDR_LANGUE, langue);
  }
  nb_enreg = EEPROM.read(ADDR_NB);
  if (nb_enreg > NB_ENREG_MAX) {                 // EEPROM neuve ou incohérente : aucune mesure
    nb_enreg = 0;
    EEPROM.update(ADDR_NB, nb_enreg);
  }
}

void lire_enreg(uint8_t i, Enregistrement &e) {
  EEPROM.get(ADDR_DONNEES + i * sizeof(Enregistrement), e);
}

bool ajouter_enreg(const Enregistrement &e) {     // ~25 ms (3,3 ms par octet écrit)
  if (nb_enreg >= NB_ENREG_MAX) return false;
  EEPROM.put(ADDR_DONNEES + nb_enreg * sizeof(Enregistrement), e);
  nb_enreg++;
  EEPROM.update(ADDR_NB, nb_enreg);              // écrit APRÈS les données : une coupure pendant
  return true;                                   // l'écriture ne laisse pas d'enregistrement à moitié
}

void effacer_enreg() {                           // remet le compteur à 0 (les mesures sont oubliées)
  nb_enreg = 0;
  EEPROM.update(ADDR_NB, nb_enreg);
}

// --- Affichage d'un enregistrement dans le moniteur série (preuve pour le rapport) ---
void serie2(uint8_t n) {
  if (n < 10) Serial.print('0');
  Serial.print(n);
}

void serie_enreg(uint8_t i, const Enregistrement &e) {
  Serial.print(F("  n")); Serial.print(i + 1); Serial.print(F(" : "));
  Serial.print(e.bpm); Serial.print(F(" bpm  "));
  if (e.annee == 0) { Serial.println(F("date inconnue")); return; }
  serie2(e.jour); Serial.print('/'); serie2(e.mois); Serial.print(F("/20")); serie2(e.annee);
  Serial.print(' '); serie2(e.heure); Serial.print(':'); serie2(e.minute);
  Serial.println();
}

void lister_enreg_serie() {
  Serial.print(F("[FS6] EEPROM : ")); Serial.print(nb_enreg);
  Serial.print(F(" mesure(s) enregistree(s), langue = "));
  Serial.println(langue == LANGUE_FR ? F("FR") : F("EN"));
  Enregistrement e;
  for (uint8_t i = 0; i < nb_enreg; i++) { lire_enreg(i, e); serie_enreg(i, e); }
}

// =====================================================================================
//          FS6 — ENREGISTREMENT : MOYENNE DE 10 MESURES CONSÉCUTIVES (ET6.2, ET6.3)
// =====================================================================================
// Sans bloquer le programme (pas de delay) : la mesure, les écrans et les boutons continuent.
// Chaque battement détecté par FS1 donne une mesure = 60000 / intervalle (BPM instantané).
// Une mesure est gardée si elle est entre 30 et 200 bpm et à moins de 35 % de la moyenne des
// mesures déjà gardées (sinon c'est un battement mal détecté : elle est écartée). Après 3 mesures
// écartées d'affilée, on recommence à zéro (les 10 mesures doivent être consécutives).
// Si plus aucune mesure n'est gardée pendant 8 s (doigt retiré) ou si on appuie sur un bouton,
// l'enregistrement est annulé.

void afficher_message(Message m, Mode ensuite) {
  message = m;
  mode_apres_message = ensuite;
  t_fin_message = millis() + DUREE_MESSAGE_MS;
  mode = MODE_MESSAGE;
}

void demarrer_enregistrement() {
  if (nb_enreg >= NB_ENREG_MAX) { afficher_message(MSG_PLEINE, MODE_GRAPHE); return; }
  nb_mesures = 0;                                // (pas besoin d'un BPM affiché à l'instant
  somme_mesures = 0;                             //  de l'appui : on attend les battements)
  derniere_mesure = 0;
  nb_ecartees = 0;
  nb_intervalles_vus = nb_intervalles_total;     // on ne compte que les battements à venir
  t_derniere_mesure = millis();
  mode = MODE_ENREG;
  Serial.println(F("[FS6] Enregistrement : 10 mesures..."));
}

void terminer_enregistrement() {
  Enregistrement e;
  e.bpm = (somme_mesures + NB_MESURES_MOYENNE / 2) / NB_MESURES_MOYENNE;   // moyenne arrondie
  Heure h;
  rtc_lire(h);                                   // ET6.3 : date et heure du moment
  bool h_ok = h.yr >= 2020 && h.yr < 2100 && h.mon >= 1 && h.mon <= 12 && h.hr < 24 && h.min < 60;
  e.jour   = h_ok ? h.date : 0;
  e.mois   = h_ok ? h.mon : 0;
  e.annee  = h_ok ? h.yr - 2000 : 0;             // 0 = date inconnue (module RTC absent)
  e.heure  = h_ok ? h.hr : 0;
  e.minute = h_ok ? h.min : 0;
  if (!ajouter_enreg(e)) { afficher_message(MSG_PLEINE, MODE_GRAPHE); return; }
  bpm_enregistre = e.bpm;
  Serial.print(F("[FS6] Enregistre (moyenne de 10 mesures) :"));
  serie_enreg(nb_enreg - 1, e);
  afficher_message(MSG_ENREGISTRE, MODE_GRAPHE);
}

// Appelée à chaque tour de boucle pendant l'enregistrement ; renvoie true si l'écran doit changer
bool suivre_enregistrement() {
  bool change = false;
  if (nb_intervalles_total != nb_intervalles_vus) {          // un nouveau battement
    nb_intervalles_vus = nb_intervalles_total;
    unsigned int b = (60000UL + dernier_intervalle / 2) / dernier_intervalle;
    bool garder = (b >= BPM_CONFORME_MIN && b <= BPM_CONFORME_MAX);
    if (garder && nb_mesures > 0) {              // cohérente avec les mesures déjà gardées ?
      float moyenne = (float)somme_mesures / nb_mesures;
      garder = fabs(b - moyenne) <= ECART_MESURE * moyenne;
    }
    if (garder) {
      somme_mesures += b;
      nb_mesures++;
      derniere_mesure = b;
      nb_ecartees = 0;
      t_derniere_mesure = millis();
      Serial.print(F("[FS6]   mesure ")); Serial.print(nb_mesures);
      Serial.print(F(" : ")); Serial.print(b); Serial.println(F(" bpm"));
    } else {
      Serial.print(F("[FS6]   ecartee : ")); Serial.print(b); Serial.println(F(" bpm"));
      if (++nb_ecartees >= 3) {                  // battements trop irréguliers : on recommence
        nb_mesures = 0;
        somme_mesures = 0;
        derniere_mesure = 0;
        nb_ecartees = 0;
        Serial.println(F("[FS6]   trop irregulier : on recommence"));
      }
    }
    change = true;
  }
  if (nb_mesures >= NB_MESURES_MOYENNE) {
    terminer_enregistrement();
    change = true;
  } else if (millis() - t_derniere_mesure > DELAI_ABANDON_MS) {
    Serial.println(F("[FS6] Annule : plus de mesure valide"));
    afficher_message(MSG_ANNULE, MODE_GRAPHE);
    change = true;
  }
  return change;
}

// =====================================================================================
//                FS6 — BOUTONS ET MENU À L'ENCODEUR (ET6.4, ET6.5)
// =====================================================================================
// Renvoie true une seule fois par appui (passage au niveau "appuyé", stable pendant ANTI_REBOND_MS)
bool bouton_appui(Bouton &b) {
  bool l = digitalRead(b.pin);
  if (l != b.lu) { b.lu = l; b.t_changement = millis(); }
  if (l != b.stable && millis() - b.t_changement >= ANTI_REBOND_MS) {
    b.stable = l;
    return l == b.niveau_appui;
  }
  return false;
}

// Encodeur et boutons : ce qu'ils font dépend de l'écran affiché.
//   ok    = bouton MENU/OK (D7) ou clic de l'encodeur (D11)
//   enreg = bouton ENREG/RETOUR (D8) : enregistrer sur la courbe, retour dans le menu
// Renvoie true si l'écran 2 doit être redessiné tout de suite.
bool gerer_entrees() {
  int crans = lire_encodeur();
  bool ok_menu = bouton_appui(bouton_menu);    // les deux sont lus à chaque tour
  bool ok_enc  = bouton_appui(bouton_enc);     // (pour que leur anti-rebond reste à jour)
  bool clic  = ok_menu || ok_enc;
  bool enreg = bouton_appui(bouton_enreg);
  if (crans == 0 && !clic && !enreg) return false;

  switch (mode) {
    case MODE_GRAPHE:
      if (crans != 0) changer_echelle(crans);   // ET5.4 : base de temps
      if (clic) { mode = MODE_MENU; ligne_menu = 0; }
      if (enreg) demarrer_enregistrement();     // ET6.2
      break;

    case MODE_MENU:
      if (crans != 0) {                          // la sélection tourne en boucle
        int l = ((int)ligne_menu + crans) % NB_LIGNES_MENU;
        if (l < 0) l += NB_LIGNES_MENU;
        ligne_menu = l;
      }
      if (clic) {
        switch (ligne_menu) {
          case 0:                                // Consulter
            if (nb_enreg == 0) afficher_message(MSG_AUCUNE, MODE_MENU);
            else { index_consult = nb_enreg - 1; mode = MODE_CONSULTER; }   // la plus récente
            break;
          case 1:                                // Effacer (après confirmation)
            confirmer_oui = false;
            mode = MODE_CONFIRMER;
            break;
          case 2:                                // Langue FR <-> EN (ET6.5), gardée en EEPROM
            langue = (langue == LANGUE_FR) ? LANGUE_EN : LANGUE_FR;
            EEPROM.update(ADDR_LANGUE, langue);
            break;
          default:                               // Retour à la courbe
            mode = MODE_GRAPHE;
            break;
        }
      } else if (enreg) {
        mode = MODE_GRAPHE;                      // RETOUR : on quitte le menu
      }
      break;

    case MODE_CONSULTER:
      if (crans != 0) {                          // rotation : mesure précédente / suivante
        int i = (int)index_consult + crans;
        if (i < 0) i = 0;
        if (i > nb_enreg - 1) i = nb_enreg - 1;
        index_consult = i;
      }
      if (clic || enreg) mode = MODE_MENU;       // OK ou RETOUR : retour au menu
      break;

    case MODE_CONFIRMER:
      if (crans != 0) confirmer_oui = (crans > 0);
      if (clic) {
        if (confirmer_oui) {
          effacer_enreg();
          Serial.println(F("[FS6] Mesures effacees"));
          afficher_message(MSG_EFFACE, MODE_MENU);
        } else {
          mode = MODE_MENU;
        }
      } else if (enreg) {
        mode = MODE_MENU;                        // RETOUR = Non
      }
      break;

    case MODE_ENREG:
      if (clic || enreg) {                       // un appui annule l'enregistrement
        Serial.println(F("[FS6] Annule par l'utilisateur"));
        afficher_message(MSG_ANNULE, MODE_GRAPHE);
      }
      break;

    case MODE_MESSAGE:
      if (clic || enreg) mode = mode_apres_message;   // un appui ferme le message
      break;
  }
  return true;
}

// =====================================================================================
//                  FS6 — PAGES DE L'ÉCRAN 2 (menu, consultation, ...)
// =====================================================================================
void barre_titre() {                             // bandeau blanc en haut, texte en noir
  ecran.fillRect(0, 0, 128, 10, SSD1306_WHITE);
  ecran.setTextColor(SSD1306_BLACK);
  ecran.setCursor(2, 1);
}

void fin_titre() {
  ecran.setTextColor(SSD1306_WHITE);
}

void dessiner_menu() {
  barre_titre();
  ecran.print(F("MENU"));
  ecran.setCursor(80, 1);
  ecran.print(nb_enreg);
  ecran.print(TXT(" mes.", " rec."));
  fin_titre();
  for (uint8_t i = 0; i < NB_LIGNES_MENU; i++) {
    uint8_t y = 14 + i * 12;
    if (i == ligne_menu) {                       // ligne sélectionnée : en vidéo inverse
      ecran.fillRect(0, y - 2, 128, 11, SSD1306_WHITE);
      ecran.setTextColor(SSD1306_BLACK);
    }
    ecran.setCursor(4, y);
    if (i == 0)      ecran.print(TXT("Consulter mesures", "View records"));
    else if (i == 1) ecran.print(TXT("Effacer mesures", "Clear records"));
    else if (i == 2) ecran.print(TXT("Langue : Francais", "Language: English"));
    else             ecran.print(TXT("Retour courbe", "Back to graph"));
    ecran.setTextColor(SSD1306_WHITE);
  }
}

void dessiner_consultation() {
  Enregistrement e;
  lire_enreg(index_consult, e);
  barre_titre();
  ecran.print(TXT("Mesure ", "Record "));
  ecran.print(index_consult + 1);
  ecran.print('/');
  ecran.print(nb_enreg);
  fin_titre();
  // BPM en grand
  ecran.setTextSize(3);
  ecran.setCursor(e.bpm >= 100 ? 20 : 38, 15);
  ecran.print(e.bpm);
  ecran.setTextSize(2);
  ecran.setCursor(76, 22);
  ecran.print(F("bpm"));
  ecran.setTextSize(1);
  // Date et heure (ET6.3)
  ecran.setCursor(4, 44);
  if (e.annee == 0) {
    ecran.print(TXT("date inconnue", "unknown date"));
  } else {
    print2(e.jour); ecran.print('/'); print2(e.mois); ecran.print(F("/20")); print2(e.annee);
    ecran.print(F("  "));
    print2(e.heure); ecran.print(':'); print2(e.minute);
  }
  ecran.setCursor(0, 56);
  ecran.print(TXT("<> defiler  OK:retour", "<> scroll   OK:back"));
}

void dessiner_confirmation() {
  barre_titre();
  ecran.print(TXT("EFFACER", "CLEAR"));
  fin_titre();
  ecran.setCursor(4, 16);
  ecran.print(TXT("Effacer les ", "Clear all "));
  ecran.print(nb_enreg);
  ecran.setCursor(4, 26);
  ecran.print(TXT("mesures ?", "records?"));
  // Choix Non / Oui : celui sélectionné est en vidéo inverse
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t x = (k == 0) ? 14 : 74;
    bool choisi = (k == 1) == confirmer_oui;
    if (choisi) {
      ecran.fillRect(x - 4, 42, 40, 13, SSD1306_WHITE);
      ecran.setTextColor(SSD1306_BLACK);
    }
    ecran.setCursor(x + 4, 45);
    if (k == 0) ecran.print(TXT("Non", "No"));
    else        ecran.print(TXT("Oui", "Yes"));
    ecran.setTextColor(SSD1306_WHITE);
  }
}

void dessiner_enregistrement() {
  barre_titre();
  ecran.print(TXT("ENREGISTREMENT", "RECORDING"));
  fin_titre();
  ecran.setCursor(4, 14);
  ecran.print(TXT("Mesure ", "Sample "));
  ecran.print(nb_mesures);
  ecran.print(F("/"));
  ecran.print(NB_MESURES_MOYENNE);
  // Dernière mesure en grand
  ecran.setTextSize(2);
  ecran.setCursor(40, 25);
  if (nb_mesures > 0) {
    ecran.print(derniere_mesure);
    ecran.setTextSize(1);
    ecran.print(F(" bpm"));
  } else {
    ecran.setTextSize(1);
    ecran.setCursor(4, 28);
    ecran.print(TXT("Gardez le doigt pose", "Keep your finger on"));
  }
  // Barre de progression
  ecran.drawRect(4, 44, 120, 7, SSD1306_WHITE);
  ecran.fillRect(4, 44, 120 * nb_mesures / NB_MESURES_MOYENNE, 7, SSD1306_WHITE);
  ecran.setCursor(4, 56);
  ecran.print(TXT("Bouton = annuler", "Button = cancel"));
}

void dessiner_message() {
  ecran.setCursor(4, 8);
  switch (message) {
    case MSG_ENREGISTRE:
      ecran.print(TXT("Enregistre !", "Saved!"));
      ecran.setTextSize(3);
      ecran.setCursor(14, 24);
      ecran.print(bpm_enregistre);
      ecran.setTextSize(2);
      ecran.print(F(" bpm"));
      ecran.setTextSize(1);
      ecran.setCursor(4, 54);
      ecran.print(TXT("Memoire : ", "Memory: "));
      ecran.print(nb_enreg); ecran.print('/'); ecran.print(NB_ENREG_MAX);
      break;
    case MSG_ANNULE:
      ecran.print(TXT("Enregistrement", "Recording"));
      ecran.setCursor(4, 20);
      ecran.print(TXT("annule", "cancelled"));
      break;
    case MSG_PLEINE:
      ecran.print(TXT("Memoire pleine", "Memory full"));
      ecran.setCursor(4, 20);
      ecran.print(TXT("Effacez des mesures", "Clear some records"));
      break;
    case MSG_EFFACE:
      ecran.print(TXT("Mesures effacees", "Records cleared"));
      break;
    case MSG_PAS_DOIGT:
      ecran.print(TXT("Posez le doigt et", "Place your finger,"));
      ecran.setCursor(4, 20);
      ecran.print(TXT("attendez le BPM", "wait for the BPM"));
      ecran.setCursor(4, 32);
      ecran.print(TXT("puis reessayez", "then try again"));
      break;
    case MSG_AUCUNE:
      ecran.print(TXT("Aucune mesure", "No records"));
      ecran.setCursor(4, 20);
      ecran.print(TXT("enregistree", "saved yet"));
      break;
  }
}

// Écran 2 : la page dépend du mode
void dessiner_ecran2() {
  ecran.clearDisplay();
  ecran.setTextSize(1);
  ecran.setTextColor(SSD1306_WHITE);
  switch (mode) {
    case MODE_GRAPHE:    dessiner_graphe();         break;
    case MODE_MENU:      dessiner_menu();           break;
    case MODE_CONSULTER: dessiner_consultation();   break;
    case MODE_CONFIRMER: dessiner_confirmation();   break;
    case MODE_ENREG:     dessiner_enregistrement(); break;
    case MODE_MESSAGE:   dessiner_message();        break;
  }
  ecran.envoyer(ADRESSE_ECRAN2);                 // envoi des 1024 octets (~25 ms)
}

// =====================================================================================
//                 PILOTE DES ÉCRANS : I2C (TWI) + SSD1306, SANS BIBLIOTHÈQUE
// =====================================================================================
// Le bus I2C est géré directement par le module TWI de l'ATmega328P (SDA = A4, SCL = A5), à
// 400 kHz. On attend la fin de chaque octet (pas de tampon, pas d'interruption) : envoyer une
// image de 1024 octets prend ~25 ms, comme avec la bibliothèque Wire, et l'échantillonnage
// continue pendant ce temps (interruption du Timer1).
//   Codes d'état TWI (TWSR) : 0x08 / 0x10 = START envoyé, 0x18 = adresse acquittée,
//                             0x28 = octet acquitté
#define TWI_ATTENTE_MAX   20000      // garde-fou : on abandonne si le bus reste bloqué (~10 ms)

bool i2c_attendre() {                // attend la fin de l'opération en cours
  uint16_t n = 0;
  while (!(TWCR & _BV(TWINT))) {
    if (++n > TWI_ATTENTE_MAX) return false;
  }
  return true;
}

void i2c_stop() {
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
  uint16_t n = 0;
  while ((TWCR & _BV(TWSTO)) && ++n < TWI_ATTENTE_MAX) {}
}

bool i2c_debut(uint8_t adresse) {    // START + adresse en écriture ; true si l'écran répond
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
  if (!i2c_attendre()) return false;
  uint8_t etat = TWSR & 0xF8;
  if (etat != 0x08 && etat != 0x10) return false;
  TWDR = adresse << 1;
  TWCR = _BV(TWINT) | _BV(TWEN);
  if (!i2c_attendre()) return false;
  return (TWSR & 0xF8) == 0x18;
}

bool i2c_ecrire(uint8_t v) {
  TWDR = v;
  TWCR = _BV(TWINT) | _BV(TWEN);
  if (!i2c_attendre()) return false;
  return (TWSR & 0xF8) == 0x28;
}

void i2c_demarrer() {
  pinMode(A4, INPUT_PULLUP);         // résistances de tirage internes en plus de celles des modules
  pinMode(A5, INPUT_PULLUP);
  TWSR = 0;                          // prédiviseur 1
  TWBR = 12;                         // 16 MHz / (16 + 2 x 12) = 400 kHz
  TWCR = _BV(TWEN);
}

bool ecran_repond(uint8_t adresse) { // un appareil répond-il à cette adresse I2C ?
  bool ok = i2c_debut(adresse);
  i2c_stop();
  return ok;
}

// Envoie une suite de commandes au SSD1306 (octet de contrôle 0x00 = "commandes")
bool ssd1306_commandes(uint8_t adresse, const uint8_t *liste, uint8_t n) {
  bool ok = i2c_debut(adresse) && i2c_ecrire(0x00);
  for (uint8_t i = 0; ok && i < n; i++) ok = i2c_ecrire(pgm_read_byte(&liste[i]));
  i2c_stop();
  return ok;
}

// Initialisation d'un écran 128 x 64 (mêmes réglages que la bibliothèque Adafruit_SSD1306)
const uint8_t SSD1306_INIT[] PROGMEM = {
  0xAE,             // écran éteint pendant le réglage
  0xD5, 0x80,       // fréquence d'horloge interne
  0xA8, 0x3F,       // 64 lignes
  0xD3, 0x00,       // pas de décalage vertical
  0x40,             // première ligne affichée = 0
  0x8D, 0x14,       // pompe de charge interne activée (alimentation 3,3 / 5 V)
  0x20, 0x00,       // adressage horizontal : les octets remplissent l'écran page par page
  0xA1,             // colonnes de gauche à droite
  0xC8,             // lignes de haut en bas
  0xDA, 0x12,       // câblage des lignes pour 128 x 64
  0x81, 0xCF,       // contraste
  0xD9, 0xF1,       // précharge
  0xDB, 0x40,       // tension VCOMH
  0xA4,             // affiche le contenu de la mémoire
  0xA6,             // normal (pas inversé)
  0x2E,             // pas de défilement
  0xAF              // écran allumé
};

const uint8_t SSD1306_ZONE[] PROGMEM = {
  0x21, 0, 127,     // colonnes 0 à 127
  0x22, 0, 7        // pages 0 à 7
};

bool EcranOLED::begin(uint8_t adresse) {
  return ssd1306_commandes(adresse, SSD1306_INIT, sizeof(SSD1306_INIT));
}

// Envoie l'image (1024 octets) à l'écran : octet de contrôle 0x40 = "données"
bool EcranOLED::envoyer(uint8_t adresse) {
  if (!ssd1306_commandes(adresse, SSD1306_ZONE, sizeof(SSD1306_ZONE))) return false;
  bool ok = i2c_debut(adresse) && i2c_ecrire(0x40);
  for (uint16_t i = 0; ok && i < sizeof(image); i++) ok = i2c_ecrire(image[i]);
  i2c_stop();
  return ok;
}

// =====================================================================================
//                                  SETUP / LOOP
// =====================================================================================
// RAM restante pour la pile (l'image de l'écran est déjà comptée dans les variables globales)
extern char *__brkval;
extern char __heap_start;
int ram_libre() {
  char v;
  return __brkval ? &v - __brkval : &v - &__heap_start;
}


void setup() {
  Serial.begin(115200);
  pinMode(PIN_CAPTEUR, INPUT);
  demarrer_voyants();                            // FS3 : D3/D4/D5 éteintes (+ test au démarrage)
  demarrer_beeper();                             // FS4 : buzzer D2, bouton A1 (+ test des 3 sons)

  demarrer_rtc();

  // ----- Écrans : on vérifie que chacun répond, puis on l'initialise et on l'efface -----
  i2c_demarrer();
  delay(100);                                    // les écrans démarrent en ~100 ms
  ecran1_present = ecran_repond(ADRESSE_ECRAN1);
  ecran2_present = ecran_repond(ADRESSE_ECRAN2);
  ecran.clearDisplay();
  if (ecran1_present) { ecran.begin(ADRESSE_ECRAN1); ecran.envoyer(ADRESSE_ECRAN1); }
  if (ecran2_present) { ecran.begin(ADRESSE_ECRAN2); ecran.envoyer(ADRESSE_ECRAN2); }

  // ----- Bilan dans le moniteur série (utile pour le rapport) -----
  Serial.print(F("Ecran 1 (heure + BPM, 0x3C) : "));
  Serial.println(ecran1_present ? F("OK") : F("ABSENT"));
  Serial.print(F("Ecran 2 (PPG, 0x3D)         : "));
  Serial.println(ecran2_present ? F("OK") : F("ABSENT (resistance d'adresse au dos ?)"));
  Serial.print(F("RAM libre : "));
  Serial.println(ram_libre());
  lire_heure();
  Serial.print(F("Heure RTC : "));
  if (heure_ok) {
    Serial.print(heure.hr); Serial.print(':');
    if (heure.min < 10) Serial.print('0');
    Serial.println(heure.min);
  } else {
    Serial.println(F("module absent ou mal branche (RST 6, DAT 12, CLK 13)"));
    // Ce qui a été lu, pour le diagnostic : 0/0/2000 0:0:0 = le module ne répond pas
    // (fil RST ou alimentation) ; 165/... ou 85/... = DAT ou CLK mal branché
    Serial.print(F("  lu : "));
    Serial.print(heure.date); Serial.print('/'); Serial.print(heure.mon); Serial.print('/');
    Serial.print(heure.yr); Serial.print(' '); Serial.print(heure.hr); Serial.print(':');
    Serial.print(heure.min); Serial.print(':'); Serial.println(heure.sec);
  }

  // ----- FS6 : relit l'EEPROM (langue + mesures gardées malgré la coupure, ET6.1) -----
  charger_eeprom();
  lister_enreg_serie();
  // Boutons : résistance externe vers GND -> simple entrée ; sinon pull-up interne
  pinMode(PIN_BOUTON_ENREG, BOUTONS_ACTIFS_HAUT ? INPUT : INPUT_PULLUP);
  pinMode(PIN_BOUTON_MENU,  BOUTONS_ACTIFS_HAUT ? INPUT : INPUT_PULLUP);

  demarrer_encodeur();
  demarrer_timer1();
}

void loop() {
  // 1) Traite tous les échantillons arrivés depuis le dernier tour
  int v;
  while (lire_echantillon(v)) traiter_echantillon(v);

  // 2) Voyants (FS3), beeper (FS4), encodeur + boutons : échelle du graphique (ET5.4), menu et enregistrement (FS6)
  bool redessiner = gerer_entrees();
  mettre_a_jour_voyants();                       // FS3 : jaune / vert / rouge / aucun
  if (surveiller_bouton_beep()) redessiner = true;   // FS4 : le bouton agit déjà dans son interruption
  mettre_a_jour_beeper();                        // FS4 : un beep par battement (après les voyants)
  if (mode == MODE_ENREG && suivre_enregistrement()) redessiner = true;
  if (mode == MODE_MESSAGE && (long)(millis() - t_fin_message) >= 0) {
    mode = mode_apres_message;                   // fin du message temporaire
    redessiner = true;
  }

  // 3) Un seul écran redessiné par tour de boucle (~25 ms chacun), pour ne pas laisser
  //    la file d'échantillons déborder
  if (ecran2_present && (redessiner || millis() - t_graphe >= PERIODE_GRAPHE_MS)) {
    t_graphe = millis();
    dessiner_ecran2();
  } else {
    mettre_a_jour_ecran1();          // ne fait rien si l'heure, le BPM et le cœur n'ont pas changé
  }
}
