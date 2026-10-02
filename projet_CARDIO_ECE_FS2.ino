#include <Wire.h>
#include <DS1302.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// INITIALISATION DU DS1302
const int CE_PIN = 5;
const int IO_PIN = 6;
const int SCLK_PIN = 7;

DS1302 rtc(CE_PIN,IO_PIN,SCLK_PIN);
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void setup() {
  // INITIALISATION DE L'ECRAN OLED
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  rtc.halt(false);
  rtc.writeProtect(false);

  // INITIALISATION DE LA DATE DE DEBUT
  Time t = rtc.time();
  if (t.yr < 2020) {  // heure jamais réglée
    Time initial(2026,10,2, 18, 29,17, Time::kFriday);
    rtc.time(initial);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Time t = rtc.time();
  // EFFACEMENT DE L'ANCIEN AFFICHAGE
  display.clearDisplay();

  // Affichage de l'heure
  display.setCursor(0, 0);
  display.print("Heure ");
  display.print(t.hr);
  display.print(":");
  if (t.min < 10) display.print("0");
  display.print(t.min);
  display.print(":");
  if (t.sec < 10) display.print("0");
  display.print(t.sec);
  
  // Affichage de la date
  display.setCursor(0, 16);
  display.print("Date ");
  if (t.date < 10) display.print("0");
  display.print(t.date);
  display.print("/");
  if (t.mon < 10) display.print("0");
  display.print(t.mon);
  display.print("/");
  display.print(t.yr);
  
  // MISE A JOUR
  display.display();
  delay(1000); // Mise à jour chaque seconde

}
