#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>

const int pinLDR = A0;
const int pinJoystickX = A1;
const int pinJoystickY = A2;
const int pinJoystickButton = 2;
const int pinLED = 8; 

LiquidCrystal_I2C lcd(0x27, 16, 2);

int currentScreen = 0;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

bool screenChanged = true;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100;
const String ID = "6291623";
const String name = "Jacob";

bool pharesOn = false;

unsigned long lastLowLightTime = 0;
unsigned long lastHighLightTime = 0;
bool belowThreshold = false;
bool aboveThreshold = false;

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  pinMode(pinJoystickButton, INPUT_PULLUP);
  pinMode(pinLED, OUTPUT);

  lcd.clear();

  createCustomChar();
  displayStartup();
}

void loop() {
  handleScreenButton();
  readJoystick();
  handleLDR();
  updateLCD();
  serialData();

  if (digitalRead(pinJoystickButton) == LOW) {
    currentScreen = (currentScreen == 1) ? 2 : 1;
    delay(200);
  }

  delay(1000);
}

//Écran LCD
void displayStartup() {
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(name);
  lcd.setCursor(0, 1);
  lcd.write(byte(0));
  lcd.setCursor(14, 1);
  lcd.print("23");
  delay(3000);
}

// Caractère personnalisé
void createCustomChar() {
  byte customChar[8] = {
    B11100,
    B10010,
    B10010,
    B11100,
    B10010,
    B10010,
    B11100,
    B00000
  };
  lcd.createChar(0, customChar);
}

//Lecture joystick
int speedValue = 0;
int directionAngle = 0;

void readJoystick() {
  int yValue = analogRead(pinJoystickY);  
  int xValue = analogRead(pinJoystickX);

  if (yValue < 512) {
    speedValue = map(yValue, 512, 1023, 25, 120);
  } else {
 
    directionAngle = map(xValue, 0, 1023, -90, 90);  
  }
}

//LDR
void handleLDR() {
  int sensorValue = analogRead(pinLDR);
  int luminosityPct = map(sensorValue, 0, 1023, 100, 0);

  unsigned long currentTime = millis();

  if (luminosityPct < 50) {
    if (!belowThreshold) {
      belowThreshold = true;
      lastLowLightTime = currentTime;
    }
    if (currentTime - lastLowLightTime >= 5000) {
      pharesOn = true;
    }
  } else {
    belowThreshold = false;
  }

  if (luminosityPct >= 50) {
    if (!aboveThreshold) {
      aboveThreshold = true;
      lastHighLightTime = currentTime;
    }
    if (currentTime - lastHighLightTime >= 5000) {
      pharesOn = false;
    }
  } else {
    aboveThreshold = false;
  }

  // Contrôle LED
  digitalWrite(pinLED, pharesOn ? HIGH : LOW);
}

//Gestion bouton écran
void handleScreenButton() {
  bool reading = digitalRead(pinJoystickButton);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      currentScreen++;
      if (currentScreen > 1) currentScreen = 0;
      screenChanged = true;
      Serial.print(currentScreen);
    }
  }

  lastButtonState = reading;
}

//Affichage LCD
void updateLCD() {
  lcd.clear();
//Page 1
  if (currentScreen == 1) {
    int sensorValue = analogRead(pinLDR);
    lcd.setCursor(0, 0);
    lcd.print("Pct Lum : ");
    lcd.print(round(sensorValue/10.2));
    lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("Phares : ");
    lcd.print(digitalRead(pinLED) ? "ON" : "OFF");
//Page 2
  } else {
    lcd.setCursor(0, 0);
    int speed = map(analogRead(pinJoystickY), 0, 1023, -25, 120);
    if(speed<0){
      lcd.print("Recul : ");
      lcd.print(speed);
      lcd.print("km/h");
    }else {
      lcd.print("Avance : ");
      lcd.print(speed);
      lcd.print("km/h");
    }
    lcd.setCursor(0 ,1);
    int direction = map(analogRead(pinJoystickX), 0, 1023, -90, 90);
    lcd.print("Direct : ");
    lcd.print(direction);
    lcd.print(direction < 0 ? " G" : " D");
  }
}

void serialData() {
  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();

    int xValue = analogRead(pinJoystickX);
    int yValue = analogRead(pinJoystickY);

    String dataFrame = "etd:" + ID + ",";
    dataFrame += "x:" + String(xValue) + ",";
    dataFrame += "y:" + String(yValue) + ",";
    dataFrame += "sys:" + String(pharesOn ? 1 : 0);

    Serial.println(dataFrame);
  }
}