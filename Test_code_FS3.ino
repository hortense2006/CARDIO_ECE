/*
  Projet Cardio ECE - Module FS3
  Test d'affichage de l'état de santé via LEDs
*/

// Attribution des broches
const int PIN_LED_ROUGE = 2; // Fréquence élevée (Tachycardie)
const int PIN_LED_VERTE = 3; // Fréquence normale
const int PIN_LED_JAUNE = 4; // Fréquence faible (Bradycardie)

// Définition des seuils de BPM (valeurs médicales usuelles de repos)
const int SEUIL_BAS = 60;   // En dessous : Fréquence faible (Jaune)
const int SEUIL_HAUT = 100; // Au dessus : Fréquence élevée (Rouge)

// Bornes pour éliminer les valeurs aberrantes (ET3.3)
const int BPM_MIN_VALIDE = 30;
const int BPM_MAX_VALIDE = 220;

void setup() {
  // Configuration des broches en sortie
  pinMode(PIN_LED_ROUGE, OUTPUT);
  pinMode(PIN_LED_VERTE, OUTPUT);
  pinMode(PIN_LED_JAUNE, OUTPUT);

  // Initialisation : Éteindre tous les voyants
  eteindreToutesLEDs();

  // Moniteur série pour simulation/test
  Serial.begin(9600);
  Serial.println("--- Test du module FS3 ---");
}

void loop() {
  // SIMULATION : Vous pouvez entrer un BPM dans le moniteur série
  if (Serial.available() > 0) {
    int bpmSaisi = Serial.parseInt();
    
    // Si la saisie est valide
    if (bpmSaisi > 0) {
      Serial.print("BPM reçu : ");
      Serial.println(bpmSaisi);
      
      // Appel de la fonction de traitement
      afficherEtatSante(bpmSaisi);
    }
  }
}

/**
 * Fonction traitant l'affichage lumineux en fonction du BPM
 * @param bpm Valeur du rythme cardiaque en Battements Par Minute
 */
void afficherEtatSante(int bpm) {
  // ET3.3 : Vérification si la valeur est réaliste / non aberrante
  if (bpm < BPM_MIN_VALIDE || bpm > BPM_MAX_VALIDE) {
    // Valeur non conforme -> Extinction de tous les voyants
    eteindreToutesLEDs();
    Serial.println("Etat : Valeur aberrante/invalide -> LEDs éteintes");
    return;
  }

  // ET3.1 & ET3.2 : Mise à jour des voyants (Un seul allumé à la fois)
  if (bpm < SEUIL_BAS) {
    // Rythme faible (Bradycardie) -> LED Jaune
    digitalWrite(PIN_LED_JAUNE, HIGH);
    digitalWrite(PIN_LED_VERTE, LOW);
    digitalWrite(PIN_LED_ROUGE, LOW);
    Serial.println("Etat : Fréquence faible -> LED Jaune");
  } 
  else if (bpm <= SEUIL_HAUT) {
    // Rythme normal -> LED Verte
    digitalWrite(PIN_LED_JAUNE, LOW);
    digitalWrite(PIN_LED_VERTE, HIGH);
    digitalWrite(PIN_LED_ROUGE, LOW);
    Serial.println("Etat : Fréquence normale -> LED Verte");
  } 
  else {
    // Rythme élevé (Tachycardie) -> LED Rouge
    digitalWrite(PIN_LED_JAUNE, LOW);
    digitalWrite(PIN_LED_VERTE, LOW);
    digitalWrite(PIN_LED_ROUGE, HIGH);
    Serial.println("Etat : Fréquence élevée -> LED Rouge");
  }
}

/**
 * Éteint l'ensemble des LEDs
 */
void eteindreToutesLEDs() {
  digitalWrite(PIN_LED_ROUGE, LOW);
  digitalWrite(PIN_LED_VERTE, LOW);
  digitalWrite(PIN_LED_JAUNE, LOW);
}