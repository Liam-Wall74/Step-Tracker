#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>

// =====================================================
// LCD configuration
// =====================================================

// Change 0x27 to 0x3F if the LCD backlight turns on
// but no text appears.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Custom upward arrow character for the LCD
byte upArrow[8] = {
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00100,
  B00000
};

// =====================================================
// PCB pin connections
// =====================================================

// Filtered accelerometer outputs
const byte X_PIN = A0;        // FILT_OUT1
const byte Y_PIN = A1;        // FILT_OUT2
const byte Z_PIN = A2;        // FILT_OUT3

// ADXL335 self-test control
const byte ST_CTRL_PIN = 2;

// Indicator LEDs
const byte GREEN_LED  = 3;
const byte YELLOW_LED = 5;
const byte RED_LED    = 6;

// Pushbuttons
const byte NEXT_BUTTON = 7;   // B1: save each position
const byte CAL_BUTTON  = 8;   // B2: start calibration

// =====================================================
// Calibration settings
// =====================================================

// Change this to your measured Arduino/Nano 5 V rail
// for more accurate voltage display.
const float ADC_REF_VOLTAGE = 5.00;

// Number of samples used during calibration
const int CAL_SAMPLES = 250;

// Number of samples used for the live LCD display
const int LIVE_SAMPLES = 40;

// Time allowed for the box to settle after pressing B1
const unsigned long SETTLE_TIME_MS = 2000;

// Reject readings too close to the Arduino ADC limits
const float ADC_LOW_LIMIT  = 10.0;
const float ADC_HIGH_LIMIT = 1010.0;

// EEPROM storage
const int EEPROM_ADDR = 0;
const uint16_t CAL_MAGIC = 0xA335;

struct CalibrationData {
  uint16_t magic;
  float offset[3];   // X, Y, Z zero-g centre values
  float scale[3];    // X, Y, Z ADC counts per g
};

CalibrationData cal;

const byte axisPins[3] = {X_PIN, Y_PIN, Z_PIN};

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(9600);

  analogReference(DEFAULT);

  pinMode(ST_CTRL_PIN, OUTPUT);

  // Your self-test circuit is active-low:
  // D2 HIGH keeps the actual ADXL335 self-test signal LOW/OFF.
  digitalWrite(ST_CTRL_PIN, HIGH);

  pinMode(NEXT_BUTTON, INPUT);
  pinMode(CAL_BUTTON, INPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  allLEDsOff();

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, upArrow);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("ADXL335 System"));
  lcd.setCursor(0, 1);
  lcd.print(F("Starting..."));

  Serial.println(F("ADXL335 LCD Calibration Test"));
  Serial.println(F("----------------------------"));
  Serial.println(F("Use external PCB power during calibration."));
  Serial.println();

  delay(1500);

  if (loadCalibration()) {
    digitalWrite(GREEN_LED, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Cal data loaded"));
    lcd.setCursor(0, 1);
    lcd.print(F("B2=recalibrate"));

    Serial.println(F("Stored calibration loaded."));
    printCalibrationData();
  } else {
    digitalWrite(YELLOW_LED, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("No calibration"));
    lcd.setCursor(0, 1);
    lcd.print(F("Press B2 start"));

    Serial.println(F("No stored calibration found."));
    Serial.println(F("Press B2 to begin calibration."));
  }

  delay(2000);
}

// =====================================================
// Main loop
// =====================================================

void loop() {
  if (buttonPressed(CAL_BUTTON)) {
    runCalibration();
  }

  if (cal.magic == CAL_MAGIC) {
    displayLiveAcceleration();
  } else {
    displayWaitingScreen();
  }

  delay(300);
}

// =====================================================
// Calibration routine
// =====================================================

void runCalibration() {
  digitalWrite(ST_CTRL_PIN, HIGH);   // Self-test OFF

  // Save the currently active successful calibration before attempting a new one.
  CalibrationData previousCal = cal;
  bool hadPreviousCalibration = (cal.magic == CAL_MAGIC);

  allLEDsOff();
  digitalWrite(YELLOW_LED, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Calibration"));
  lcd.setCursor(0, 1);
  lcd.print(F("Starting..."));

  Serial.println();
  Serial.println(F("Starting calibration..."));
  Serial.println(F("Keep the box still for each reading."));
  Serial.println();

  delay(1500);

  float plusRaw[3]  = {0.0, 0.0, 0.0};
  float minusRaw[3] = {0.0, 0.0, 0.0};

  const byte targetAxis[6] = {
    0, 0,   // +X, -X
    1, 1,   // +Y, -Y
    2, 2    // +Z, -Z
  };

  const bool positiveReading[6] = {
    true, false,
    true, false,
    true, false
  };

  bool readingClipped = false;

  for (byte position = 0; position < 6; position++) {
    displayOrientationPrompt(position);
    printSerialOrientationPrompt(position);

    waitForButton(NEXT_BUTTON);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Hold still..."));
    lcd.setCursor(0, 1);
    lcd.print(F("Measuring"));

    Serial.println(F("Hold still..."));

    delay(SETTLE_TIME_MS);

    float raw[3];
    readXYZAverage(raw, CAL_SAMPLES);

    byte axis = targetAxis[position];

    if (positiveReading[position]) {
      plusRaw[axis] = raw[axis];
    } else {
      minusRaw[axis] = raw[axis];
    }

    if (nearADCRail(raw[axis])) {
      readingClipped = true;
    }

    Serial.print(F("Saved "));
    printOrientationNameSerial(position);
    Serial.println();
    printRawVoltages(raw);
    Serial.println();

    displaySavedReading(position, axis, raw[axis]);

    delay(1500);
  }

  // Store the new calculations in a temporary structure.
  // The active calibration is not changed unless this attempt passes.
  CalibrationData newCal;
  newCal.magic = CAL_MAGIC;

  bool calibrationOK = !readingClipped;

  for (byte axis = 0; axis < 3; axis++) {
    newCal.offset[axis] = (plusRaw[axis] + minusRaw[axis]) / 2.0;
    newCal.scale[axis]  = (plusRaw[axis] - minusRaw[axis]) / 2.0;

    // Reject if the +1g and -1g readings were too similar.
    if (fabs(newCal.scale[axis]) < 20.0) {
      calibrationOK = false;
    }

    // Reject if either end of an axis clipped near the ADC rails.
    if (nearADCRail(plusRaw[axis]) ||
        nearADCRail(minusRaw[axis])) {
      calibrationOK = false;
    }
  }

  if (calibrationOK) {
    // Only now replace the active and stored calibration.
    cal = newCal;
    EEPROM.put(EEPROM_ADDR, cal);

    allLEDsOff();
    digitalWrite(GREEN_LED, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Calibration"));
    lcd.setCursor(0, 1);
    lcd.print(F("SUCCESS"));

    Serial.println(F("Calibration successful."));
    Serial.println(F("New calibration has been saved."));
    printCalibrationData();

    delay(3000);
  } else {
    allLEDsOff();
    digitalWrite(RED_LED, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Cal attempt FAIL"));

    Serial.println(F("Calibration failed."));
    Serial.println(F("Check for:"));
    Serial.println(F("- Incorrect orientation"));
    Serial.println(F("- Movement during reading"));
    Serial.println(F("- ADC clipping near 0 or 1023"));
    Serial.println(F("- Signal/filter hardware fault"));

    delay(2000);

    if (hadPreviousCalibration) {
      // Restore the previously successful active calibration.
      // EEPROM was never overwritten, so the saved calibration is also unchanged.
      cal = previousCal;

      allLEDsOff();
      digitalWrite(GREEN_LED, HIGH);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Using saved cal"));
      lcd.setCursor(0, 1);
      lcd.print(F("B2=try again"));

      Serial.println();
      Serial.println(F("Previous successful calibration restored."));
      Serial.println(F("Failed readings were not saved."));
      printCalibrationData();

      delay(3000);
    } else {
      // No successful calibration has ever been completed.
      cal.magic = 0;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("No valid cal"));
      lcd.setCursor(0, 1);
      lcd.print(F("B2=try again"));

      Serial.println();
      Serial.println(F("No previous successful calibration available."));

      delay(3000);
    }
  }
}

// =====================================================
// LCD orientation instructions
// =====================================================

void displayOrientationPrompt(byte position) {
  lcd.clear();

  switch (position) {
    case 0:
      // +X: LCD side faces upward
      lcd.setCursor(0, 0);
      lcd.print(F("+X LCD SIDE"));
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      lcd.print(F(" UP  Press B1"));
      break;

    case 1:
      // -X: BNC connector faces upward
      lcd.setCursor(0, 0);
      lcd.print(F("-X BNC SIDE"));
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      lcd.print(F(" UP  Press B1"));
      break;

    case 2:
      // +Y: User-interface LEDs face upward
      lcd.setCursor(0, 0);
      lcd.print(F("+Y UI LED SIDE"));
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      lcd.print(F(" UP  Press B1"));
      break;

    case 3:
      // -Y: Barrel jack faces upward
      lcd.setCursor(0, 0);
      lcd.print(F("-Y BARREL JACK"));
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      lcd.print(F(" UP  Press B1"));
      break;

    case 4:
      // +Z: LCD visible and text correctly oriented
      lcd.setCursor(0, 0);
      lcd.print(F("+Z LCD TEXT"));
      lcd.setCursor(0, 1);
      lcd.print(F("RIGHT WAY  B1"));
      break;

    case 5:
      // -Z: LCD visible but text upside down
      lcd.setCursor(0, 0);
      lcd.print(F("-Z LCD TEXT"));
      lcd.setCursor(0, 1);
      lcd.print(F("UPSIDE DOWN B1"));
      break;
  }
}

void printSerialOrientationPrompt(byte position) {
  Serial.print(F("Place box in position: "));

  switch (position) {
    case 0:
      Serial.println(F("+X: LCD side facing upwards."));
      break;

    case 1:
      Serial.println(F("-X: BNC connector facing upwards."));
      break;

    case 2:
      Serial.println(F("+Y: User-interface LEDs facing upwards."));
      break;

    case 3:
      Serial.println(F("-Y: Barrel jack facing upwards."));
      break;

    case 4:
      Serial.println(F("+Z: LCD visible with text the right way around."));
      break;

    case 5:
      Serial.println(F("-Z: LCD visible with text upside down."));
      break;
  }

  Serial.println(F("Then press B1 to record."));
}

void printOrientationNameSerial(byte position) {
  switch (position) {
    case 0:
      Serial.print(F("+X axis UP"));
      break;
    case 1:
      Serial.print(F("-X axis UP"));
      break;
    case 2:
      Serial.print(F("+Y axis UP"));
      break;
    case 3:
      Serial.print(F("-Y axis UP"));
      break;
    case 4:
      Serial.print(F("+Z axis UP"));
      break;
    case 5:
      Serial.print(F("-Z axis UP"));
      break;
  }
}

void displaySavedReading(byte position, byte axis, float rawValue) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Saved "));

  switch (position) {
    case 0:
      lcd.print(F("+X"));
      break;
    case 1:
      lcd.print(F("-X"));
      break;
    case 2:
      lcd.print(F("+Y"));
      break;
    case 3:
      lcd.print(F("-Y"));
      break;
    case 4:
      lcd.print(F("+Z"));
      break;
    case 5:
      lcd.print(F("-Z"));
      break;
  }

  lcd.setCursor(0, 1);

  if (axis == 0) {
    lcd.print(F("X:"));
  } else if (axis == 1) {
    lcd.print(F("Y:"));
  } else {
    lcd.print(F("Z:"));
  }

  lcd.print(rawValue, 0);
  lcd.print(F(" "));
  lcd.print(adcToVoltage(rawValue), 2);
  lcd.print(F("V"));
}

// =====================================================
// Live calibrated display
// =====================================================

void displayLiveAcceleration() {
  float raw[3];
  float g[3];

  readXYZAverage(raw, LIVE_SAMPLES);

  for (byte axis = 0; axis < 3; axis++) {
    g[axis] = (raw[axis] - cal.offset[axis]) / cal.scale[axis];
  }

  float magnitude = sqrt(
    g[0] * g[0] +
    g[1] * g[1] +
    g[2] * g[2]
  );

  clearLCDRow(0);
  lcd.setCursor(0, 0);
  lcd.print(F("X:"));
  printSignedLCD(g[0]);
  lcd.print(F(" Y:"));
  printSignedLCD(g[1]);

  clearLCDRow(1);
  lcd.setCursor(0, 1);
  lcd.print(F("Z:"));
  printSignedLCD(g[2]);
  lcd.print(F(" M:"));
  lcd.print(magnitude, 2);

  Serial.print(F("X = "));
  Serial.print(g[0], 2);
  Serial.print(F(" g, Y = "));
  Serial.print(g[1], 2);
  Serial.print(F(" g, Z = "));
  Serial.print(g[2], 2);
  Serial.print(F(" g, Magnitude = "));
  Serial.print(magnitude, 2);
  Serial.println(F(" g"));
}

void displayWaitingScreen() {
  clearLCDRow(0);
  lcd.setCursor(0, 0);
  lcd.print(F("No calibration"));

  clearLCDRow(1);
  lcd.setCursor(0, 1);
  lcd.print(F("Press B2 start"));
}

// =====================================================
// ADC reading functions
// =====================================================

void readXYZAverage(float raw[3], int numberOfSamples) {
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  for (int sample = 0; sample < numberOfSamples; sample++) {
    analogRead(X_PIN);
    delayMicroseconds(200);
    sumX += analogRead(X_PIN);

    analogRead(Y_PIN);
    delayMicroseconds(200);
    sumY += analogRead(Y_PIN);

    analogRead(Z_PIN);
    delayMicroseconds(200);
    sumZ += analogRead(Z_PIN);

    delay(3);
  }

  raw[0] = sumX / float(numberOfSamples);
  raw[1] = sumY / float(numberOfSamples);
  raw[2] = sumZ / float(numberOfSamples);
}

float adcToVoltage(float adcValue) {
  return adcValue * ADC_REF_VOLTAGE / 1023.0;
}

bool nearADCRail(float value) {
  return value < ADC_LOW_LIMIT || value > ADC_HIGH_LIMIT;
}

// =====================================================
// EEPROM functions
// =====================================================

bool loadCalibration() {
  EEPROM.get(EEPROM_ADDR, cal);

  if (cal.magic != CAL_MAGIC) {
    return false;
  }

  for (byte axis = 0; axis < 3; axis++) {
    if (fabs(cal.scale[axis]) < 20.0) {
      return false;
    }
  }

  return true;
}

// =====================================================
// Button functions
// =====================================================

bool buttonPressed(byte pin) {
  if (digitalRead(pin) == HIGH) {
    delay(30);

    if (digitalRead(pin) == HIGH) {
      while (digitalRead(pin) == HIGH) {
        delay(5);
      }

      delay(30);
      return true;
    }
  }

  return false;
}

void waitForButton(byte pin) {
  while (digitalRead(pin) == LOW) {
    delay(5);
  }

  delay(30);

  while (digitalRead(pin) == HIGH) {
    delay(5);
  }

  delay(30);
}

// =====================================================
// Display and serial helper functions
// =====================================================

void clearLCDRow(byte row) {
  lcd.setCursor(0, row);
  lcd.print(F("                "));
}

void printSignedLCD(float value) {
  if (value >= 0) {
    lcd.print(F("+"));
  }

  lcd.print(value, 2);
}

void printRawVoltages(float raw[3]) {
  Serial.print(F("Raw X = "));
  Serial.print(raw[0], 2);
  Serial.print(F("  Voltage = "));
  Serial.print(adcToVoltage(raw[0]), 3);
  Serial.print(F(" V, Raw Y = "));
  Serial.print(raw[1], 2);
  Serial.print(F("  Voltage = "));
  Serial.print(adcToVoltage(raw[1]), 3);
  Serial.print(F(" V, Raw Z = "));
  Serial.print(raw[2], 2);
  Serial.print(F("  Voltage = "));
  Serial.print(adcToVoltage(raw[2]), 3);
  Serial.println(F(" V"));
}

void printCalibrationData() {
  Serial.println();
  Serial.println(F("Calibration data:"));

  Serial.print(F("X offset = "));
  Serial.print(cal.offset[0], 2);
  Serial.print(F(", X scale = "));
  Serial.println(cal.scale[0], 2);

  Serial.print(F("Y offset = "));
  Serial.print(cal.offset[1], 2);
  Serial.print(F(", Y scale = "));
  Serial.println(cal.scale[1], 2);

  Serial.print(F("Z offset = "));
  Serial.print(cal.offset[2], 2);
  Serial.print(F(", Z scale = "));
  Serial.println(cal.scale[2], 2);

  Serial.println();
}

void allLEDsOff() {
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
}