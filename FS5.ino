// =====================================================================================
//  Cardio ECE — FS1 (mesure PPG + BPM) + FS5 (affichage sur deux écrans OLED 128x64)
// =====================================================================================
//
//  FS5 : Afficher les informations
//    ET5.1  Écran 1 (adresse 0x3C) : heure (module RTC DS1302) + BPM, ou un tiret si la valeur
//           n'est pas conforme ou si le doigt n'est pas posé
//    ET5.2  Écran 2 (adresse 0x3D) : graphique du PPG
//    ET5.3  Graphique gradué : base de temps affichée en ms par pixel (en haut) et en secondes
//           pour 25 pixels (en bas, une graduation tous les 25 pixels)
//    ET5.4  Encodeur rotatif : change la base de temps, comme sur un oscilloscope
//           (10 / 20 / 40 / 80 / 200 ms par pixel)
//
//  Câblage
//    capteur WPSE340       : signal -> A0
//    2 écrans OLED (I2C)   : SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND (les deux en parallèle)
//                            l'écran 2 a sa résistance d'adresse déplacée au dos : 0x3C -> 0x3D
//    encodeur rotatif      : côté 3 broches : A -> D9, broche du MILIEU -> GND, B -> D10
//                            côté 2 broches : bouton poussoir, une broche -> D11, l'autre -> GND
//                            (pas de résistance : on utilise les pull-up internes de l'Arduino)
//    module RTC DS1302     : RST -> 3, DAT -> 12, CLK -> 13, VCC -> 5V, GND -> GND
//  Bibliothèques : "Adafruit GFX" et "Adafruit SSD1306" (le DS1302 est piloté sans bibliothèque)
//
//  Traitement du signal (FS1), à chaque échantillon (100 Hz, cadencé par le Timer1) :
//    1) passe-bas      : enlève le bruit
//    2) passe-haut     : enlève la composante continue et la dérive (luminosité, pression du doigt)
//    3) normalisation  : (signal - min) / (max - min) sur les 2 dernières secondes -> entre 0 et 1
//    4) détection      : un battement = passage au-dessus de SEUIL_HAUT (avec hystérésis)
//    5) BPM            : 60000 / moyenne des 4 derniers intervalles réguliers
//    -> la courbe et le BPM ne sont affichés que si des battements RÉGULIERS sont détectés.
//
//  Mémoire : la bibliothèque Adafruit garde une image de 1024 octets par écran. Deux écrans =
//  2048 octets = toute la RAM de l'Arduino Uno. On utilise donc UN SEUL objet écran (une seule
//  image en mémoire) et on change d'adresse I2C avant d'envoyer l'image à l'un ou à l'autre.
// =====================================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

// ---------- Module RTC DS1302 ----------
#define RTC_RST              3
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
#define PIN_ENC_BOUTON       11        // bouton poussoir de l'encodeur : pas utilisé par FS5
                                       // (réservé pour FS4 : marche / arrêt du beeper)
#define TRANSITIONS_CRAN     4         // 4 sur un KY-040 (mettre 2 si un cran saute 2 calibres)
#define SENS_ENCODEUR        1         // -1 pour inverser le sens

// ---------- Traceur série ----------
#define AFFICHER_1_SUR       2         // 1 point sur 2 envoyé (50 points/s)

// =====================================================================================
//                                 VARIABLES GLOBALES
// =====================================================================================

// Un seul objet pour les deux écrans : on change simplement l'adresse I2C avant chaque envoi.
// i2caddr est une variable interne de la bibliothèque, accessible depuis une classe dérivée.
class EcranPartage : public Adafruit_SSD1306 {
public:
  EcranPartage() : Adafruit_SSD1306(128, 64, &Wire, -1) {}
  void choisir(uint8_t adresse) { i2caddr = adresse; }
};
EcranPartage ecran;
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
#define FILE_LEN 8                   // 80 ms de mesures en attente au maximum (un écran prend ~25 ms)
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

  // 6) Traceur série
  if (++compteur_serie >= AFFICHER_1_SUR) {
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

  ecran.choisir(ADRESSE_ECRAN1);
  ecran.display();                               // envoi des 1024 octets (~25 ms)
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
//   ligne du haut  : "PPG", base de temps en ms par pixel, "Amp"
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

void dessiner_ecran2() {
  unsigned int ms_px = CALIBRES[calibre] * PERIODE_ECH_MS;   // ET5.3 : ms par pixel
  unsigned int ms_grad = ms_px * PX_PAR_GRAD;                // ET5.3 : durée d'une graduation

  ecran.clearDisplay();
  ecran.setTextSize(1);
  ecran.setTextColor(SSD1306_WHITE);

  // ----- Ligne du haut : titre, base de temps par pixel, nom de l'axe vertical -----
  ecran.setCursor(0, 0);
  ecran.print(F("PPG"));
  ecran.setCursor(30, 0);
  ecran.print(ms_px);
  ecran.print(F("ms/px"));
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
    ecran.print(g * 0.5f, 1);
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
    ecran.print(F("Aucun battement"));
    ecran.setCursor(8, 34);
    ecran.print(F("Posez le doigt"));
  }

  ecran.choisir(ADRESSE_ECRAN2);
  ecran.display();                               // envoi des 1024 octets (~25 ms)
}

// =====================================================================================
//                                  SETUP / LOOP
// =====================================================================================
// RAM restante pour la pile après la réservation de l'image de l'écran (doit rester > ~150)
extern char *__brkval;
extern char __heap_start;
int ram_libre() {
  char v;
  return __brkval ? &v - __brkval : &v - &__heap_start;
}

bool ecran_repond(uint8_t adresse) {   // un appareil répond-il à cette adresse I2C ?
  Wire.beginTransmission(adresse);
  return Wire.endTransmission() == 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CAPTEUR, INPUT);

  demarrer_rtc();

  // ----- Écrans : on vérifie que chacun répond, puis on l'initialise -----
  // begin() réserve l'image de 1024 octets au premier appel seulement, puis envoie la séquence
  // d'initialisation à l'adresse donnée : on l'appelle une fois par écran.
  Wire.begin();
  ecran1_present = ecran_repond(ADRESSE_ECRAN1);
  ecran2_present = ecran_repond(ADRESSE_ECRAN2);
  if (!ecran.begin(SSD1306_SWITCHCAPVCC, ADRESSE_ECRAN1)) {
    Serial.println(F("Erreur : pas assez de RAM pour l'image de l'ecran"));
    while (true) {}
  }
  if (ecran2_present) ecran.begin(SSD1306_SWITCHCAPVCC, ADRESSE_ECRAN2);
  ecran.clearDisplay();
  if (ecran1_present) { ecran.choisir(ADRESSE_ECRAN1); ecran.display(); }
  if (ecran2_present) { ecran.choisir(ADRESSE_ECRAN2); ecran.display(); }

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
    Serial.println(F("module absent ou mal branche (RST 3, DAT 12, CLK 13)"));
  }

  demarrer_encodeur();
  demarrer_timer1();
}

void loop() {
  // 1) Traite tous les échantillons arrivés depuis le dernier tour
  int v;
  while (lire_echantillon(v)) traiter_echantillon(v);

  // 2) Encodeur (ET5.4) : changement d'échelle -> graphique redessiné tout de suite
  bool redessiner = false;
  int crans = lire_encodeur();
  if (crans != 0) {
    changer_echelle(crans);
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