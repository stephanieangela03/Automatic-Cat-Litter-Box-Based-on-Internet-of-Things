#include "arduino_secrets.h"
#include <ESP32Servo.h>
#include "thingProperties.h"

const int dirPin = 12;
const int stepPin = 13;
const int limit1Pin = 14;
const int limit2Pin = 15;
const int servoPin = 4;
const int pirSensorPin = 19;
const int ledPin = 21;
const int trigWastePin = 22;
const int echoWastePin = 23;
const int trigSandLevelPin = 25;
const int echoSandLevelPin = 26;
const int trigRefillPin = 27;
const int echoRefillPin = 32;
const int refillServoPin = 5;

int motorSpeed = 5000;
bool direction;
bool ignoreLimit1 = false;
bool ignoreLimit2 = false;

Servo myServo;
Servo refillServo;
bool programStarted = false;
bool servoOpened = false;
bool programFinished = false;

bool motionDetected = false;
bool waitingAfterMotion = false;
unsigned long motionStopTime = 0;
const unsigned long MOTION_DELAY = 150000;
bool motionEverDetected = false;

const float WASTE_FULL_THRESHOLD = 2.0;
const float SAND_LOW_THRESHOLD = 15.0;
const float REFILL_EMPTY_THRESHOLD = 6.0;

bool refillServoOpened = false;
unsigned long refillOpenTime = 0;
const unsigned long REFILL_OPEN_DURATION = 1000;

bool refillMode = false;
int refillStepCount = 0;
const int REFILL_STEPS = 1000;
bool refillReversed = false;

bool stoppedByMotion = false;
bool justFinishedRefill = false;
bool firstRunAfterStart = true;

bool cleaningTriggered = false;
bool refillTriggered = false;
bool autoRefillTriggered = false;
float lastWasteReading = 0;
float lastRefillReading = 0;
float lastSandReading = 0;
bool cloudConnected = false;

bool refillPausedByMotion = false;
unsigned long refillPauseStartTime = 0;
int savedRefillStepCount = 0;
bool savedRefillReversed = false;
bool savedRefillDirection = true;

bool cloudCleaningRequest = false;
bool cloudRefillRequest = false;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(limit1Pin, INPUT_PULLUP);
  pinMode(limit2Pin, INPUT_PULLUP);
  pinMode(pirSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(trigWastePin, OUTPUT);
  pinMode(echoWastePin, INPUT);
  pinMode(trigSandLevelPin, OUTPUT);
  pinMode(echoSandLevelPin, INPUT);
  pinMode(trigRefillPin, OUTPUT);
  pinMode(echoRefillPin, INPUT);
  
  myServo.attach(servoPin);
  myServo.write(0);
  
  refillServo.attach(refillServoPin);
  refillServo.write(0);
  
  digitalWrite(ledPin, LOW);
  
  Serial.begin(115200);
  
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
  manualCleaning = false;
  manualRefill = false;
  wasteLevel = 0.0;
  refillLevel = 0.0;
  status = "";
  
  for(int i = 0; i < 30; i++) {
    delay(1000);
    ArduinoCloud.update();
  }
  
  direction = true;
  digitalWrite(dirPin, direction);
  
  status = "System initialized";
  cloudConnected = true;
}

long getFilteredDistance(int trigPin, int echoPin, int samples = 5) {
  long total = 0;
  int validSamples = 0;
  
  for (int i = 0; i < samples + 2; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration > 0) {
      long distance = duration * 0.034 / 2;
      
      if (distance > 0.5 && distance < 50.0) {
        total += distance;
        validSamples++;
      }
    }
    delay(20);
  }
  
  if (validSamples >= samples) {
    return total / validSamples;
  } else {
    return -1;
  }
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

void loop() {
  ArduinoCloud.update();
  handleCloudCommands();
  
  if (programStarted && !programFinished) {
    if (firstRunAfterStart) {
      if (stoppedByMotion) {
        direction = true;
      } else {
        direction = true;
      }
      digitalWrite(dirPin, direction);
      firstRunAfterStart = false;
    }
  } else if (!programStarted || programFinished) {
    firstRunAfterStart = true;
  }
  
  bool currentMotion = digitalRead(pirSensorPin) == HIGH;
  
  if (currentMotion) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
  
  handleMotionDetection(currentMotion);
  
  if (refillServoOpened && (millis() - refillOpenTime >= REFILL_OPEN_DURATION)) {
    refillServo.write(0);
    refillServoOpened = false;
    
    delay(2000);
    
    refillMode = true;
    refillStepCount = 0;
    refillReversed = false;
    ignoreLimit2 = false;
    direction = true;
    digitalWrite(dirPin, direction);
    
    if (refillTriggered) {
      manualRefill = false;
      refillTriggered = false;
    }
  }
  
  if ((!programStarted || programFinished) && !refillMode && !refillPausedByMotion) {
    return;
  }
  
  if (refillMode || refillPausedByMotion) {
    handleRefillMotorWithMotion();
    return;
  }
  
  if (ignoreLimit1 && digitalRead(limit1Pin) == HIGH) {
    ignoreLimit1 = false;
  }
  if (ignoreLimit2 && digitalRead(limit2Pin) == HIGH) {
    ignoreLimit2 = false;
  }
  
  if (!ignoreLimit1 && direction && digitalRead(limit1Pin) == LOW) {
    delay(5000);
    direction = !direction;
    digitalWrite(dirPin, direction);
    ignoreLimit1 = true;
  }
  
  if (!ignoreLimit2 && !direction && digitalRead(limit2Pin) == LOW) {
    delay(5000);
    direction = !direction;
    digitalWrite(dirPin, direction);
    ignoreLimit2 = true;
    
    programFinished = true;
    
    if (servoOpened) {
      myServo.write(0);
      delay(1000);
      
      servoOpened = false;
      
      delay(5000);
      checkSensorsAfterCleaning();
    }
    
    if (cleaningTriggered) {
      manualCleaning = false;
      cleaningTriggered = false;
    }
    
    status = "Cleaning complete";
    ArduinoCloud.update();
    
    return;
  }
  
  if (ignoreLimit1 && digitalRead(limit1Pin) == HIGH) {
    ignoreLimit1 = false;
  }
  
  if (ignoreLimit2 && digitalRead(limit2Pin) == HIGH) {
    ignoreLimit2 = false;
  }
  
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(motorSpeed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(motorSpeed);
}

void handleRefillMotorWithMotion() {
  if (refillPausedByMotion) {
    if (!motionDetected && (millis() - refillPauseStartTime >= MOTION_DELAY)) {
      refillPausedByMotion = false;
      refillMode = true;
      
      refillStepCount = savedRefillStepCount;
      refillReversed = savedRefillReversed;
      direction = savedRefillDirection;
      digitalWrite(dirPin, direction);
      
      status = "Refill resumed";
      ArduinoCloud.update();
      
      return;
    }
    
    return;
  }
  
  if (motionDetected && refillMode && !refillPausedByMotion) {
    savedRefillStepCount = refillStepCount;
    savedRefillReversed = refillReversed;
    savedRefillDirection = direction;
    
    refillMode = false;
    refillPausedByMotion = true;
    refillPauseStartTime = millis();
    
    status = "Refill paused: Motion detected";
    ArduinoCloud.update();
    return;
  }
  
  if (refillMode) {
    if (refillStepCount < REFILL_STEPS) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(motorSpeed);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(motorSpeed);
      refillStepCount++;
    } else {
      if (!refillReversed) {
        direction = !direction;
        digitalWrite(dirPin, direction);
        refillReversed = true;
        status = "Refill: Returning home";
        ArduinoCloud.update();
      }
      
      if (!ignoreLimit2 && !direction && digitalRead(limit2Pin) == LOW) {
        delay(5000);
        refillMode = false;
        refillPausedByMotion = false;
        direction = !direction; 
        programFinished = true;
        justFinishedRefill = true;
        ignoreLimit2 = true;
        
        waitingAfterMotion = false;
        motionDetected = false;
        
        if (refillTriggered) {
          manualRefill = false;
          refillTriggered = false;
        }
        
        status = "Refill complete";
        ArduinoCloud.update();
        
        delay(3000);
        status = "Cleaning complete";
        ArduinoCloud.update();
        return;
      }
      
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(motorSpeed);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(motorSpeed);
    }
  }
}

void checkSensorsAfterCleaning() {
  lastWasteReading = getFilteredDistance(trigWastePin, echoWastePin, 7);
  lastSandReading = getFilteredDistance(trigSandLevelPin, echoSandLevelPin, 7);
  lastRefillReading = getFilteredDistance(trigRefillPin, echoRefillPin, 7);

  updateCloudVariables();

  status = "Cleaning complete";
  ArduinoCloud.update();
  delay(3000);
  
  if (lastWasteReading > 0 && lastWasteReading <= WASTE_FULL_THRESHOLD) {
    status = "Waste container full (" + String(wasteLevel, 0) + "%)";
    ArduinoCloud.update();
    delay(3000);
  } 
  
  if (lastRefillReading >= REFILL_EMPTY_THRESHOLD) {
    status = "Refill container empty (" + String(refillLevel, 0) + "%)";
    ArduinoCloud.update();
    delay(3000);
  }
  
  if (!(lastWasteReading > 0 && lastWasteReading <= WASTE_FULL_THRESHOLD) &&
      !(lastRefillReading >= REFILL_EMPTY_THRESHOLD)) {
    status = "Cleaning complete";
    ArduinoCloud.update();
  }
  
  if (!refillMode && !motionDetected && !refillPausedByMotion && 
      lastSandReading >= 0 && lastSandReading > SAND_LOW_THRESHOLD) {
    delay(5000);
    
    refillServo.write(180);
    refillServoOpened = true;
    refillOpenTime = millis();
    autoRefillTriggered = true;
    
    status = "Sand low. Auto-refill started";
    ArduinoCloud.update();
  }
}

float calculatePercentage(float reading, float minFull, float maxEmpty) {
  if (reading < 0) return -1;
  
  if (reading < minFull) reading = minFull;
  if (reading > maxEmpty) reading = maxEmpty;
  
  float normalized = (reading - minFull) / (maxEmpty - minFull);
  float percentage = (1.0 - normalized) * 100.0;
  
  if (percentage > 100) percentage = 100;
  if (percentage < 0) percentage = 0;
  
  return percentage;
}

void updateCloudVariables() {
  float wastePercent = calculatePercentage(lastWasteReading, 2.0, 7.0);
  if (wastePercent >= 0) wasteLevel = wastePercent;
  
  float refillPercent = calculatePercentage(lastRefillReading, 2.0, 6.0);
  if (refillPercent >= 0) refillLevel = refillPercent;
}

void updateStatus() {
  String msg = "";
  
  if (refillPausedByMotion) {
    msg = "Refill paused: Motion detected";
  } else if (refillMode) {
    if (refillStepCount < REFILL_STEPS) {
      msg = "Refilling";
    } else {
      msg = "Refill: Returning home";
    }
  } else if (programFinished) {
    msg = "Cleaning complete";
  } else if (motionDetected && !programStarted && !programFinished && motionEverDetected && waitingAfterMotion) {
    msg = "Motion detected. Auto-cleaning will start in 3 minutes if no motion";
  } else if (stoppedByMotion && motionDetected) {
    msg = "Stopped: Motion detected";
  } else {
    msg = "System ready";
  }
  
  status = msg;
}

void handleCloudCommands() {
  static bool lastCleaningState = false;
  static bool lastRefillState = false;
  
  if (manualCleaning && !lastCleaningState) {
    if (programStarted && !programFinished) {
      status = "Cleaning already in progress";
      return;
    }
    
    if (refillMode || refillPausedByMotion) {
      status = "Refill in progress. Wait until refill completes";
      return;
    }
    
    resetCleaningFlags();
    
    if (!servoOpened) {
      myServo.write(90);
      servoOpened = true;
      delay(1000);
    }
    
    direction = true;
    digitalWrite(dirPin, direction);
    
    programStarted = true;
    programFinished = false;
    firstRunAfterStart = true;
    cleaningTriggered = true;
    
    status = "Manual cleaning started";
  }
  
  if (manualRefill && !lastRefillState) {
    if (programStarted && !programFinished) {
      status = "Cleaning in progress. Wait until cleaning completes";
      return;
    }
    
    if (refillMode || refillPausedByMotion) {
      status = "Refill already in progress";
      return;
    }
    
    refillServo.write(180);
    refillServoOpened = true;
    refillOpenTime = millis();
    refillTriggered = true;
    
    status = "Manual refill started";
  }
  
  lastCleaningState = manualCleaning;
  lastRefillState = manualRefill;
}

void handleMotionDetection(bool currentMotion) {
  if (currentMotion) {
    if (!motionDetected) {
      motionDetected = true;
      motionEverDetected = true;
      updateStatus();
      ArduinoCloud.update();
    }
    
    waitingAfterMotion = true;
    motionStopTime = millis();
    
    if (!(refillMode || refillPausedByMotion)) {
      if (programFinished) {
        status = "Motion detected after cleaning";
        ArduinoCloud.update();
      }
      
      if (programStarted && !programFinished) {
        programStarted = false;
        stoppedByMotion = true;
        status = "Stopped: Motion detected";
      }
    }
  } else {
    motionDetected = false;
    
    if (!(refillMode || refillPausedByMotion)) {
      if (!waitingAfterMotion && !programStarted && !programFinished && motionEverDetected) {
        waitingAfterMotion = true;
        motionStopTime = millis();
        updateStatus();
        ArduinoCloud.update();
      }
      
      if (waitingAfterMotion && !programStarted && !programFinished) {
        unsigned long elapsed = millis() - motionStopTime;
        
        if (motionEverDetected && elapsed >= MOTION_DELAY) {
          waitingAfterMotion = false;
          
          programStarted = true;
          stoppedByMotion = false;
          
          justFinishedRefill = false;
          
          if (!servoOpened) {
            myServo.write(90);
            servoOpened = true;
            delay(500);
          }
          
          status = "Auto cleaning started";
        }
      }
    }
  }
}

void resetCleaningFlags() {
  programStarted = false;
  programFinished = false;
  refillMode = false;
  refillPausedByMotion = false;
  stoppedByMotion = false;
  firstRunAfterStart = true;
  ignoreLimit1 = false;
  ignoreLimit2 = false;
  refillStepCount = 0;
  refillReversed = false;
}

void onManualCleaningChange() {
}

void onManualRefillChange() {
}