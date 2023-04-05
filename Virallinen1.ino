// Harjoitustyö
// Koodissa vältelty tietoisesti looppeja ja delay funktion käyttöä, jotta eri järjestelmiä pystytään ohjaamaan samanaikaisesti.
// Looppia selkeytetty tekemällä jokaisesta ominaisuudesta oma funktio, jolloin ohjauksen tekeminen ja ymmärtäminen äärimmäisen helppoa.
// Funktiot kutsutaan looppiin haluttujen ehtojen täyttyessä.

#include <Servo.h>
#include <EEPROM.h>
Servo servo_9;

// PINS

const int JOYSTICK_PIN_X = A0;
const int JOYSTICK_PIN_Y = A1;
const int SYSTEM_STATE_BTN_PIN = 2;
const int AUTOMATI_DRIVE_BTN_PIN = 3;
const int SYSTEM_STATE_LED_PIN = 4;
const int MOTOR_DIRECTION_RELAY_PIN = 5;
const int AUTO_DRIVE_INDICATOR_LED_PIN = 6;
const int PULSE_METER_PIN = 7;
const int SERVO_PIN = 10;
const int FET_PIN = 11;

//  VARIABLES

//  Moottorin nopeuden inkrementointi
const int motorSpeedIncrement = 2;  // 0 min 255 max
const int motorIncrementDelay = 50; // Aika millisekunteina jokaisen inkrementin välissä
const int motorStopIncrementDelay = 25;
const int motorStopSpeedIncrement = 5;

bool servoDirection = 0;
bool pulseReceived = 0;
bool motorDirection = 0;
bool manualDriveRushStopper = 0;
bool receivingPulse = 0;

int lastServoPos, joystickXVal, joystickYVal, joystickServoAngle, pulseMotorSpeedControl, motorTargetSpeed, beatsPerMinute;
int motorSpeed = 0;
int servoPos = 90;
int heartBeatCount = -1;
int memorySlot = 0;

volatile byte automatic = 0;
volatile byte systemState = 0;

unsigned long elapsedTime, currentTime, startTime, endTime, timeOFF, totalStandbyTime, lastMotorIncrement, lastMotorIncrementTime, beatTimeForAverage, lastAverageBPMTime, beatTime;
unsigned long lastInterruptTime = 0;
unsigned long lastServoIncrementTime = 0;

void setup()
{
  //  PINS
  pinMode(JOYSTICK_PIN_X, INPUT);
  pinMode(JOYSTICK_PIN_Y, INPUT);
  pinMode(SYSTEM_STATE_BTN_PIN, INPUT_PULLUP);
  pinMode(AUTOMATI_DRIVE_BTN_PIN, INPUT_PULLUP);
  pinMode(SYSTEM_STATE_LED_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_RELAY_PIN, OUTPUT);
  pinMode(AUTO_DRIVE_INDICATOR_LED_PIN, OUTPUT);
  pinMode(PULSE_METER_PIN, INPUT_PULLUP);         //MUISTA VAIHTAA INPUT_PULLUP!!!!!!!
  pinMode(FET_PIN, OUTPUT);

  Serial.begin(9600);
  servo_9.attach(SERVO_PIN, 500, 2500);
  attachInterrupt(digitalPinToInterrupt(SYSTEM_STATE_BTN_PIN), systemStateSwitch, FALLING);
  attachInterrupt(digitalPinToInterrupt(AUTOMATI_DRIVE_BTN_PIN), automaticStateSwitch, CHANGE);
  joystickXVal = analogRead(JOYSTICK_PIN_X);
  totalStandbyTime = EEPROM.get(memorySlot, elapsedTime);
}

void loop()
{
  currentTime = millis();
  joystickXVal = analogRead(A0);      // MUISTA TARKISTAA KÄYTETTÄVÄN JOYSTICKIN KALIBROINTI!!!!!!
  joystickYVal = analogRead(A1);

  if (systemState)
  {
    timer();

    if (automatic)
    {
      automaticServoDrive();
      motorPulseDrive();
      //      Serial.print("Motor target speed: ");
      //      Serial.print(motorTargetSpeed);
      //      Serial.print("\t Motor Speed: ");
      //      Serial.println(abs(motorSpeed));
      //      Serial.print("\t");
    }
    else
    {
      manualServoDrive();
      manualMotorDrive();
      Serial.print("Motor target speed: ");
      Serial.print(motorTargetSpeed);
      Serial.print("\t Motor Speed: ");
      Serial.print(abs(motorSpeed));
      Serial.print("\t motorDirection: ");
      Serial.print(motorDirection);
      Serial.print("\t RelayPin: ");
      Serial.println(digitalRead(MOTOR_DIRECTION_RELAY_PIN));
    }
  }
  else
  {
    automatic = 0;
    motorStop();
  }
}

// OHJAUSFUNKTIOT

//  DC moottorin manuaaliohjaus. Nopeus säädettävissä joystickin Y akselilla.
//  ylöspäin joystickiä liikuttamalla moottori pyörii myötäpäivään, alaspäin vastapäivään.
//  Sekä kiihdytys että hidastus tapahtuvat hallitusti ja portaittain.
//  Suunnanvaihto tapahtuu relettä ohjaamalla kun moottorin nopeus on lähellä nollaa ja
//  joystickin Y-akselin poikkeaman suunta määrää moottorin suunnan

void manualMotorDrive()
{
  bool joystickCentered = joystickXVal > 500 && joystickXVal < 530 && joystickYVal > 500 && joystickYVal < 530;
  motorTargetSpeed = map(joystickYVal, 0, 1023, -255, 255);
  bool delayOK = currentTime - lastMotorIncrementTime > motorIncrementDelay;

  if (delayOK)
  {
    if (abs(abs(motorSpeed) - motorTargetSpeed) > 3)
    {
      motorSpeedControl();
    }
    if (abs(motorSpeed) < 10)
    {
      if (motorTargetSpeed > 10)
      {
        motorDirection = 1;
      }
      else if (motorTargetSpeed < 0)
      {
        motorDirection = 0;
      }
      else
      {
        motorSpeed = 0;
      }
    }
    digitalWrite(MOTOR_DIRECTION_RELAY_PIN, motorDirection);

    if (motorSpeed < 0 && !motorDirection)
    {
      analogWrite(FET_PIN, abs(motorSpeed));
      lastMotorIncrementTime = currentTime;
    }
    else
    {
      analogWrite(FET_PIN, motorSpeed);
      lastMotorIncrementTime = currentTime;
    }
  }
}

// Moottorin nopeuden säätö pulssianturilla. Laskee viidestä sydämenlyönnistä beatsPerMinute arvon ja skaalaa sen
// moottorin nopeudenohjaukseksi FETille. Moottorin ohjauksessa nopeuden inkrementointi. Pyörimissuunta ainoastaan myötäpäivään.
// Mikäli edellisestä lyönnistä yli 5sec, resettaa countterin ja pysäyttää moottorin.

void motorPulseDrive()                                      // MUISTA INPUT_PULLUP!!!!!!
{
  bool delayOK = currentTime - lastMotorIncrementTime > motorIncrementDelay;
  int beatsForAverage = 5; // Sydämenlyöntien määrä keskiarvoistetussa sykkeessä.
  Serial.println(receivingPulse);

  if (!pulseReceived && !digitalRead(PULSE_METER_PIN))
  {
    pulseReceived = 1;
    heartBeatCount += 1;
    receivingPulse = 1;
    beatTime = currentTime;
  }

  else if (digitalRead(PULSE_METER_PIN))
  {
    pulseReceived = 0;
  }
  if (currentTime - beatTime > 5000)
  {
    heartBeatCount = -1;
    lastAverageBPMTime = currentTime;
    beatTimeForAverage = 0;
    motorTargetSpeed = 0;
    receivingPulse = 0;

  }
  if (heartBeatCount == beatsForAverage)
  {
    beatsPerMinute = 60000 / (currentTime - lastAverageBPMTime) * beatsForAverage;
    Serial.println(beatsPerMinute);
    motorTargetSpeed = map(beatsPerMinute, 40, 180, 0, 255);
    heartBeatCount = 0;
    lastAverageBPMTime = currentTime;
  }
  if (delayOK && receivingPulse)
  {
    motorSpeedControl();
    lastMotorIncrementTime = currentTime;
    analogWrite(FET_PIN, motorSpeed);
  }
  else if (!receivingPulse)
  {
    motorStop();
  }
}

//  Moottorin nopeuden inkrementointi motorTargetSpeediä kohden, joka on asetettu joko manuaaliajossa joystickillä tai
//  sykemittariajossa sykkeen perusteella

void motorSpeedControl()
{
  if (motorSpeed < motorTargetSpeed)
  {
    motorSpeed += motorSpeedIncrement;
  }
  else if (motorSpeed > motorTargetSpeed)
  {
    motorSpeed -= motorSpeedIncrement;
  }
}
// Servon manuaaliohjaus. Lisätty ryntäyksenesto automaattiajolta manuaalille vaihdon jälkeen.

void manualServoDrive()
{
  joystickServoAngle = map(joystickXVal, 0, 1023, 0, 180);

  if (manualDriveRushStopper)
  {
    if (servoPos > joystickServoAngle && currentTime - lastServoIncrementTime > 25)
    {
      servoPos -= 3;
      lastServoIncrementTime = millis();
      servo_9.write(servoPos);
    }
    else if (currentTime - lastServoIncrementTime > 25)
    {
      servoPos += 3;
      lastServoIncrementTime = millis();
      servo_9.write(servoPos);
    }
    if (abs(servoPos - joystickServoAngle) < 4)
      manualDriveRushStopper = 0;
  }
  else
  {
    servoPos = joystickServoAngle;
    servo_9.write(servoPos);
  }
}

//  Automaattinen servon ohjaus. pyörittää servoa laidasta toiseen 10 asteen
//  inkrementeissä joystickin Y-akselin poikkeaman asettamalla viiveellä. Muistaa edellisen pyörimissuunnan.

void automaticServoDrive()
{
  int servoIncrementDelay = map(joystickYVal, 0, 1023, 100, 500);
  bool delayOK = currentTime - lastServoIncrementTime > servoIncrementDelay;
  manualDriveRushStopper = 1;

  if (delayOK)
  {
    if (!servoDirection)
    {
      servoPos += 10;

      if (servoPos >= 180)
      {
        servoDirection = 1;
        servoPos = 180;
      }
    }
    else
    {
      servoPos -= 10;
      if (servoPos <= 0)
      {
        servoDirection = 0;
        servoPos = 0;
      }
    }
    servo_9.write(servoPos);
    lastServoIncrementTime = millis();
    lastServoPos = servoPos;
  }
}

//  Pysäytysfunktio moottorille. Ajetaan, kun systeemi menee pois standby tilasta moottorin pyöriessä.
//  Pysäyttää moottorin inkrementoiden.

void motorStop()
{
  bool delayOK = currentTime - lastMotorIncrementTime > motorStopIncrementDelay;
  motorTargetSpeed = 0;
  if (abs(motorSpeed) < 3)
  {
    motorSpeed = 0;
    analogWrite(FET_PIN, motorSpeed);
  }
  if (abs(motorSpeed) > 0)
  {
    Serial.println(lastMotorIncrementTime);
    if (delayOK)
    {
      if (motorSpeed < motorTargetSpeed)
      {
        motorSpeed += motorStopSpeedIncrement;
        lastMotorIncrementTime = currentTime;
      }
      else if (motorSpeed > motorTargetSpeed)
      {
        motorSpeed -= motorStopSpeedIncrement;
        lastMotorIncrementTime = currentTime;
      }
    }
    analogWrite(FET_PIN, abs(motorSpeed));
  }
}
//  Ajastinfunktio. Tulostaa sekunnin välein monitoriin käynnissäoloaikaa, joka on tallennettuna EEPROMiin, jolloin käynnissäoloaika ei resettaa kun
//  Arduino resetoidaan tai virta katkaistaan. Käynnissäoloajan tallennus EEPROMiin tapahtuu 60 sekunnin välein, sillä kirjoitus kestää n. 3,3ms joten muistiin
//  kirjoitusta ei kannata jokaisella ohjelmakierroksella tehdä. Mustiin tallennus on myös rajallista per muistipaikka (n. 100 000 ylikirjoitusta).
//  Nykyisellä tallennustaajuudella yksi muistipaikka kestää n. 70 vuorokautta 24/7 standby tilassa.

void timer()
{
  elapsedTime = (currentTime - timeOFF) / 1000 + totalStandbyTime;

  if (elapsedTime % 60 == 0)
  {
    EEPROM.put(memorySlot, elapsedTime);
  }
  int seconds = elapsedTime % 60;
  int minutes = elapsedTime / 60 % 60;
  int hours = elapsedTime / 3600;
  if (currentTime % 1000 == 0)
  {
    Serial.print(hours);
    Serial.print("h ");
    Serial.print(minutes);
    Serial.print("m ");
    Serial.print(seconds);
    Serial.println("s");
  }
}

//  INTERRUPTIFUNKTIOT

//  Interruptifunktio valmiustilalle. Ei salli valmiustilaa, jollei joystick ole keskiasennossa. Tunnistaa laskevan reunan,
//  lisätty debounce toiminnallisuus. Tallentaa myös standby tilan käynnistys- ja sammutusajan ja laskee ajan kiinnioloajan käynnissäoloajan laskemista varten.

void systemStateSwitch()
{
  bool joystickCentered = joystickXVal > 500 && joystickXVal < 530 && joystickYVal > 500 && joystickYVal < 530;
  bool delayOK = currentTime - lastInterruptTime > 50;

  if (joystickCentered && delayOK && !systemState)
  {
    systemState = 1;
    startTime = currentTime;
    timeOFF += startTime - endTime;
    digitalWrite(SYSTEM_STATE_LED_PIN, systemState);
    lastInterruptTime = currentTime;
  }
  else if (delayOK)
  {
    systemState = 0;
    endTime = currentTime;
    digitalWrite(SYSTEM_STATE_LED_PIN, systemState);
    digitalWrite(AUTO_DRIVE_INDICATOR_LED_PIN, 0);
    lastInterruptTime = currentTime;
  }
}

//  Interruptifunktio automaattiajolle. Tunnistaa napin tilanmuutoksen, eli automaattiajo toimii ainoastaan nappia pohjaan painettaessa.

void automaticStateSwitch()
{
  bool delayOK = currentTime - lastInterruptTime > 50;

  if (delayOK && systemState)
  {
    automatic = !automatic;
    digitalWrite(AUTO_DRIVE_INDICATOR_LED_PIN, automatic);
    lastInterruptTime = currentTime;
  }
}
