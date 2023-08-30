// OSR-Alpha4_ESP32
// by TempestMAx 5-3-22
// Please copy, share, learn, innovate, give attribution.
// Decodes T-code commands and uses them to control seßrvos and vibration motors
// It can handle:
//   10x linear channels (L0, L1, L2... L9)
//   10x rotation channels (R0, R1, R2... L9) 
//   10x vibration channels (V0, V1, V2... V9)
//   10x auxilliary channels (A0, A1, A2... A9)
// This code is for the ESP32 DevKit v1 and is designed to drive the SR6 stroker robot, 
// but is also able to drive the OSR2. See below for servo pin assignments
// Have fun, play safe!
// History:
// Alpha3 - First ESP32 release, 9-7-21
// Alpha4 - Support for T-wist 4 added - standard servo now default, parallax is an option, 5-3-22
// Enhanced with Bluetooth LE support by Sorites Paradox (2023-08-30)

// ----------------------------
//   Settings
// ----------------------------

// Device IDs, for external reference
#define FIRMWARE_ID "SR6-Alpha4-ESP32-Bluetooth-Enhanced.ino"  // Device and firmware version
#define TCODE_VER "TCode v0.3"  // Current version of TCode
#define BLE_DEVICE_NAME "OSR-ESP32" // Display name of Bluetooth LE device.
#define BLE_TCODE_SERVICE_UUID "ff1b451d-3070-4276-9c81-5dc5ea1043bc"
#define BLE_TCODE_CHARACTERISTIC_UUID "c5f1543e-338d-47a0-8525-01e3c621359d"

#define OSR2_MODE true // (true/false) Switch servo outputs to OSR2 mode

// Servo microseconds per radian
// (Standard: 637 μs/rad)
// (LW-20: 700 μs/rad)
#define ms_per_rad 637  // (μs/rad)

// Pin assignments
// T-wist feedback goes on digital pin 2
#define LowerLeftServo_PIN 15    // Lower Left Servo (OSR2 Left Servo)
#define UpperLeftServo_PIN 2     // Upper Left Servo
#define LowerRightServo_PIN 13   // Lower Right Servo (OSR2 Right Servo)
#define UpperRightServo_PIN 12   // Upper Right Servo
#define LeftPitchServo_PIN 4     // Left Pitch Servo (OSR2 Pitch Servo)
#define RightPitchServo_PIN 14   // Right Pitch Servo
#define TwistServo_PIN 27        // Twist Servo
#define ValveServo_PIN 25        // Valve Servo
#define TwistFeedback_PIN 26     // Twist Servo Feedback
#define Vibe0_PIN 18             // Vibration motor 1
#define Vibe1_PIN 19             // Vibration motor 2

// Arm servo zeros
// Change these to adjust arm positions
// (1500 = centre, 1 complete step = 160)
#define LowerLeftServo_ZERO 1500         // Lower Left Servo (OSR2 Left Servo)
#define UpperLeftServo_ZERO 1500         // Upper Left Servo 
#define LowerRightServo_ZERO 1500        // Lower Right Servo (OSR2 Right Servo)
#define UpperRightServo_ZERO 1500        // Upper Right Servo 
#define LeftPitchServo_ZERO 1500         // Left Pitch Servo (OSR2 Pitch Servo)
#define RightPitchServo_ZERO 1500        // Right Pitch Servo
#define TwistServo_ZERO 1500
#define ValveServo_ZERO 1500

// Servo operating frequencies
#define PitchServo_Freq 330 // Pitch Servos
#define MainServo_Freq 330  // Main Servos
#define TwistServo_Freq 50  // Twist Servo
#define ValveServo_Freq 50  // Valve Servo
#define VibePWM_Freq 8000   // Vibe motor control PWM frequency

// Other functions
#define TWIST_PARALLAX false      // (true/false) Parallax 360 feedback servo on twist (t-wist3)
#define REVERSE_TWIST_SERVO false // (true/false) Reverse twist servo direction 
#define VALVE_DEFAULT 5000        // Auto-valve default suction level (low-high, 0-9999) 
#define REVERSE_VALVE_SERVO true  // (true/false) Reverse T-Valve direction
#define VIBE_TIMEOUT 2000         // Timeout for vibration channels (milliseconds).
#define LUBE_V1 false             // (true/false) Lube pump installed instead of vibration channel 1
#define Lube_PIN 23               // Lube manual input button pin (Connect pin to +5V for ON)
#define Lube_SPEED 255            // Lube pump speed (0-255)
#define MIN_SMOOTH_INTERVAL 3     // Minimum auto-smooth ramp interval for live commands (ms)
#define MAX_SMOOTH_INTERVAL 100   // Maximum auto-smooth ramp interval for live commands (ms)

// T-Code Channels
#define CHANNELS 10                // Number of channels of each type (LRVA)


// ----------------------------
//  Auto Settings
// ----------------------------
// Do not change

// Servo PWM channels
#define LowerLeftServo_PWM 0     // Lower Left Servo
#define UpperLeftServo_PWM 1     // Upper Left Servo
#define LowerRightServo_PWM 2    // Lower Right Servo
#define UpperRightServo_PWM 3    // Upper Right Servo
#define LeftPitchServo_PWM 4     // Left Pitch Servo
#define RightPitchServo_PWM 5    // Right Pitch Servo
#define TwistServo_PWM 6         // Twist Servo
#define ValveServo_PWM 7         // Valve Servo
#define TwistFeedback_PWM 8      // Twist Servo
#define Vibe0_PWM 9              // Vibration motor 1
#define Vibe1_PWM 10             // Vibration motor 2

// Servo Pulse intervals
#define MainServo_Int 1000000/MainServo_Freq
#define PitchServo_Int 1000000/PitchServo_Freq
#define TwistServo_Int 1000000/TwistServo_Freq
#define ValveServo_Int 1000000/ValveServo_Freq

// Libraries used
#include <EEPROM.h> // Permanent memory
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// ---------------------------------------------
// Functions to handle serial input and output.
// ---------------------------------------------
void beginSerial () {
  Serial.begin(115200);
}

void serialPrint (String input) {
  Serial.print(input);
}

void serialPrint (int input) {
  Serial.print(input);
}

void serialPrintln (String input) {
  Serial.println(input);
}

void serialPrintln (int input) {
  Serial.println(input);
}

// -----------------------------
// Class to handle each axis
// -----------------------------
class Axis {

  public:
  // Setup function
  Axis() {

    // Set default dynamic parameters
    rampStartTime = 0;
    rampStart = 5000;
    rampStopTime = rampStart;
    rampStop = rampStart;

    // Set Empty Name
    Name = "";
    lastT = 0;

    // Live command auto-smooth
    minInterval = MAX_SMOOTH_INTERVAL;
      
  }

  // Function to set the axis dynamic parameters
  void Set(int x, char ext, long y) {
    unsigned long t = millis(); // This is the time now
    x = constrain(x,0,9999);
    y = constrain(y,0,9999999);
    // Set ramp parameters, based on inputs
    // Live command
    if ( y == 0 || ( ext != 'S' && ext != 'I' ) ) {
      // update auto-smooth regulator
      int lastInterval = t - rampStartTime;
      if ( lastInterval > minInterval && minInterval < MAX_SMOOTH_INTERVAL ) { minInterval += 1; }
      else if ( lastInterval < minInterval && minInterval > MIN_SMOOTH_INTERVAL ) { minInterval -= 1; } 
      // Set ramp parameters
      rampStart = GetPosition();
      rampStopTime = t + minInterval;  
    } 
    // Speed command
    else if ( ext == 'S' ) {
      rampStart = GetPosition();  // Start from current position
      int d = x - rampStart;  // Distance to move
      if (d<0) { d = -d; }
      long dt = d;  // Time interval (time = dist/speed)
      dt *= 100;
      dt /= y; 
      rampStopTime = t + dt;  // Time to arrive at new position
      //if (rampStopTime < t + minInterval) { rampStopTime = t + minInterval; }
    }
    // Interval command
    else if ( ext == 'I' ) {
      rampStart = GetPosition();  // Start from current position
      rampStopTime = t + y;  // Time to arrive at new position
      //if (rampStopTime < t + minInterval) { rampStopTime = t + minInterval; }
    }
    rampStartTime = t;
    rampStop = x;
    lastT = t;
  }

  // Function to return the current position of this axis
  int GetPosition() {
    int x; // This is the current axis position, 0-9999
    unsigned long t = millis(); 
    if (t > rampStopTime) {
      x = rampStop;
    } else if (t > rampStartTime) { 
      x = map(t,rampStartTime,rampStopTime,rampStart,rampStop);
    } else {
      x = rampStart;
    }
    x = constrain(x,0,9999);
    return x;
  }

  // Function to stop axis movement at current position
  void Stop() {
    unsigned long t = millis(); // This is the time now
    rampStart = GetPosition();
    rampStartTime = t;
    rampStop = rampStart;
    rampStopTime = t;
  }

  // Public variables
  String Name;  // Function name of this axis
  unsigned long lastT;  //

  private:
  
  // Movement positions
  int rampStart;
  unsigned long rampStartTime;
  int rampStop;
  unsigned long rampStopTime;

  // Live command auto-smooth regulator
  int minInterval;

};


// -----------------------------
// Class to manage Toy Comms
// -----------------------------
class TCode {
  
  public:
  // Setup function
  TCode(String firmware, String tcode) {
    firmwareID = firmware;
    tcodeID = tcode;

    // Vibe channels start at 0
    for (int i = 0; i < CHANNELS; i++) { Vibration[i].Set(0,' ',0); }
    
  }

  // Function to name and activate axis
  void RegisterAxis(String ID, String axisName) {
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': Linear[channel].Name = axisName; break;
        case 'R': Rotation[channel].Name = axisName; break;
        case 'V': Vibration[channel].Name = axisName; break;
        case 'A': Auxiliary[channel].Name = axisName; break;
      }
    }
  }

  // Function to read off individual bytes as input
  void ByteInput(byte inByte) {
    bufferString += (char)inByte;  // Add new character to string
    
    if (inByte=='\n') {  // Execute string on newline
      bufferString.trim();  // Remove spaces, etc, from buffer
      executeString(bufferString); // Execute string
      bufferString = ""; // Clear input string
    }
  }

  // Function to read off whole strings as input
  void StringInput(String inString) {
    bufferString = inString;  // Replace existing buffer with input string
    bufferString.trim();  // Remove spaces, etc, from buffer
    executeString(bufferString); // Execute string
    bufferString = ""; // Clear input string
  }

  // Function to set an axis
  void AxisInput(String ID, int magnitude, char extension, long extMagnitude) {
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': Linear[channel].Set(magnitude,extension,extMagnitude); break;
        case 'R': Rotation[channel].Set(magnitude,extension,extMagnitude); break;
        case 'V': Vibration[channel].Set(magnitude,extension,extMagnitude); break;
        case 'A': Auxiliary[channel].Set(magnitude,extension,extMagnitude); break;
      }
    }
  }

  // Function to read the current position of an axis
  int AxisRead(String ID) {
    int x = 5000; // This is the return variable
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': x = Linear[channel].GetPosition(); break;
        case 'R': x = Rotation[channel].GetPosition(); break;
        case 'V': x = Vibration[channel].GetPosition(); break;
        case 'A': x = Auxiliary[channel].GetPosition(); break;
      }
    }
    return x;
  }

  // Function to query when an axis was last commanded
  unsigned long AxisLast(String ID) {
    unsigned long t = 0; // Return time
    char type = ID.charAt(0);
    int channel = ID.charAt(1) - '0';
    if ((0 <= channel && channel < CHANNELS)) {
      switch(type) {
        // Axis commands
        case 'L': t = Linear[channel].lastT; break;
        case 'R': t = Rotation[channel].lastT; break;
        case 'V': t = Vibration[channel].lastT; break;
        case 'A': t = Auxiliary[channel].lastT; break;
      }
    }
    return t;
  }

  private:
  // Strings
  String firmwareID;
  String tcodeID;
  String bufferString; // String to hold incomming commands

  // Declare axes
  Axis Linear[CHANNELS];
  Axis Rotation[CHANNELS];
  Axis Vibration[CHANNELS];
  Axis Auxiliary[CHANNELS];

  // Function to divide up and execute input string
  void executeString(String bufferString) {
    int index = bufferString.indexOf(' ');  // Look for spaces in string
    while (index > 0) {
      readCmd(bufferString.substring(0,index));  // Read off first command
      bufferString = bufferString.substring(index+1);  // Remove first command from string
      bufferString.trim();
      index = bufferString.indexOf(' ');  // Look for next space
    }
    readCmd(bufferString);  // Read off last command
  }

  // Function to process the individual commands
  void readCmd(String command) {
    command.toUpperCase();
  
    // Switch between command types
    switch( command.charAt(0) ) {
      // Axis commands
      case 'L':
      case 'R':
      case 'V':
      case 'A':
        axisCmd(command);
      break;
  
      // Device commands
      case 'D':
        deviceCmd(command);
      break;
  
      // Setup commands
      case '$':
        setupCmd(command);
      break; 
    }
  }

  // Function to read and interpret axis commands
  void axisCmd(String command) {
  
    char type = command.charAt(0);  // Type of command - LRVA
    boolean valid = true;  // Command validity flag, valid by default
  
    // Check for channel number
    int channel = command.charAt(1) - '0';
    if (channel < 0 || channel >= CHANNELS) {valid = false;}
    channel = constrain(channel,0,CHANNELS);
  
    // Check for an extension
    char extension = ' ';
    int index = command.indexOf('S',2);
    if (index > 0) {
      extension = 'S';
    } else {
      index = command.indexOf('I',2);
      if (index > 0) {
        extension = 'I';
      }
    }
    if (index < 0) { index = command.length(); }
    
    // Get command magnitude
    String magString = command.substring(2,index);
    magString = magString.substring(0,4);
    while (magString.length() < 4) { magString += '0'; }
    int magnitude = magString.toInt();
    if (magnitude == 0 && magString.charAt(0) != '0') { valid = false; } // Invalidate if zero returned, but not a number
  
    // Get extension magnitude
    long extMagnitude = 0;
    if ( extension != ' ') {
      magString = command.substring(index+1);
      magString = magString.substring(0,8);
      extMagnitude = magString.toInt();
    }
    if (extMagnitude == 0) { extension = ' '; }

    // Switch between command types
    if (valid) {
      switch(type) {
        // Axis commands
        case 'L': Linear[channel].Set(magnitude,extension,extMagnitude); break;
        case 'R': Rotation[channel].Set(magnitude,extension,extMagnitude); break;
        case 'V': Vibration[channel].Set(magnitude,extension,extMagnitude); break;
        case 'A': Auxiliary[channel].Set(magnitude,extension,extMagnitude); break;
      }
    }

  }

  // Function to identify and execute device commands
  void deviceCmd(String command) {
    int i;
    // Remove "D"
    command = command.substring(1);

    // Look for device stop command
    if (command.substring(0,4) == "STOP") {
        for (i = 0; i < 10; i++) { Linear[i].Stop(); }
        for (i = 0; i < 10; i++) { Rotation[i].Stop(); }
        for (i = 0; i < 10; i++) { Vibration[i].Set(0,' ',0); }
        for (i = 0; i < 10; i++) { Auxiliary[i].Stop(); }  
    } else {
      // Look for numbered device commands
      int commandNumber = command.toInt();
      if (commandNumber==0 && command.charAt(0)!='0' ) { command = -1; }
      switch( commandNumber ) {
        case 0:
          serialPrintln(firmwareID);
        break;
  
        case 1:
          serialPrintln(tcodeID);
        break;
  
        case 2:
          for (i = 0; i < 10; i++) { axisRow("L" + String(i), 8*i, Linear[i].Name); }
          for (i = 0; i < 10; i++) { axisRow("R" + String(i), 8*i+80, Rotation[i].Name); }
          for (i = 0; i < 10; i++) { axisRow("V" + String(i), 8*i+160, Vibration[i].Name); }
          for (i = 0; i < 10; i++) { axisRow("A" + String(i), 8*i+240, Auxiliary[i].Name); }             
        break;
      }
    }
  }

  // Function to modify axis preference values
  void setupCmd(String command) {
    int minVal,maxVal;
    String minValString,maxValString;
    boolean valid;
    // Axis type
    char type = command.charAt(1); 
    switch (type) {
      case 'L':
      case 'R':
      case 'V':
      case 'A':
      valid = true;
      break;

      default:
      type = ' ';
      valid = false;
      break;
    }
    // Axis channel number
    int channel = (command.substring(2,3)).toInt();
    if (channel == 0 && command.charAt(2) != '0') {
      valid = false;
    }
    // Input numbers
    int index1 = command.indexOf('-');  
    if (index1 !=3) { valid = false; }
    int index2 = command.indexOf('-',index1+1);  // Look for spaces in string
    if (index2 <=3) { valid = false; }
    if (valid) {
      // Min value
      minValString = command.substring(4,index2);
      minValString = minValString.substring(0,4);
      while (minValString.length() < 4) { minValString += '0'; }
      minVal = minValString.toInt();
      if ( minVal == 0 && minValString.charAt(0)!='0' ) { valid = false; }
      // Max value
      maxValString = command.substring(index2+1);
      maxValString = maxValString.substring(0,4);
      while (maxValString.length() < 4) { maxValString += '0'; }
      maxVal = maxValString.toInt();
      if ( maxVal == 0 && maxValString.charAt(0)!='0' ) { valid = false; }     
    }
    // If a valid command, save axis preferences to EEPROM
    if (valid) {
      int memIndex = 0;
      switch (type) {
        case 'L': memIndex = 0; break;
        case 'R': memIndex = 80; break;
        case 'V': memIndex = 160; break;
        case 'A': memIndex = 240; break;
      }
      memIndex += 8*channel;
      minVal = constrain(minVal,0,9999);
      EEPROM.put(memIndex, minVal-1);
      minVal = constrain(maxVal,0,9999);
      EEPROM.put(memIndex+4, maxVal-10000);
      // Output that axis changed successfully
      switch (type) {
        case 'L': axisRow("L" + String(channel), memIndex, Linear[channel].Name); break;
        case 'R': axisRow("R" + String(channel), memIndex, Rotation[channel].Name); break;
        case 'V': axisRow("V" + String(channel), memIndex, Vibration[channel].Name); break;
        case 'A': axisRow("A" + String(channel), memIndex, Auxiliary[channel].Name); break;             
      }
    }
  }
 
  // Function to print the details of an axis
  void axisRow(String axisID, int memIndex, String axisName) {
    int low, high;
    if (axisName != "") {
      EEPROM.get(memIndex,low);
      low = constrain(low,-1,9998);
      EEPROM.get(memIndex + 4,high);
      high = constrain(high,-10000,-1);
      serialPrint(axisID);
      serialPrint(" ");
      serialPrint(low + 1);
      serialPrint(" ");
      serialPrint(high + 10000);
      serialPrint(" ");
      serialPrintln(axisName);
    }
  }
    
};

// ----------------------------
//   SETUP
// ----------------------------
// This code runs once, on startup

// Declare classes
// This uses the t-code object above
TCode tcode(FIRMWARE_ID, TCODE_VER);

// Declare operating variables
// Position variables
int xLin,yLin,zLin;
// Rotation variables
int xRot,yRot,zRot;
// Vibration variables
int vibe0,vibe1;
// Lube variables
int lube;
// Valve variables
int valveCmd,suckCmd;
// Velocity tracker variables, for valve
int xLast;
unsigned long tLast;
float upVel,valvePos;
// Twist position monitor variables
volatile int twistPulseLength = 0;
volatile int twistPulseCycle = 1099;
volatile int twistPulseStart = 0;
float twistServoAngPos = 0.5;
int twistTurns = 0;
float twistPos;

void readSerial () {
  while (Serial.available() > 0) {
    tcode.ByteInput(Serial.read());
  }
}

// ----------------------------------------
// Functions to handle Bluetooth LE Setup.
//-----------------------------------------
class BLETCodeControlCallback: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    const char* input = pCharacteristic->getValue().c_str();
    const int length = strlen(input);

    for (int i = 0; i < length; i++) {
      tcode.ByteInput(input[i]);
    }
  }
};

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("A client has connected via BLE.");
    };
    void onDisconnect(BLEServer* pServer) {
      Serial.println("A client has disconnected from BLE.");
      pServer->startAdvertising(); 
    }
};

BLECharacteristic tcodeCharacteristic(
  BLE_TCODE_CHARACTERISTIC_UUID,
  BLECharacteristic::PROPERTY_READ   |
  BLECharacteristic::PROPERTY_WRITE_NR
);

void setupBLE () {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(BLE_TCODE_SERVICE_UUID);
  
  pService->addCharacteristic(&tcodeCharacteristic);
  tcodeCharacteristic.setValue("");
  tcodeCharacteristic.setCallbacks(new BLETCodeControlCallback());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_TCODE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);

  // Functions that help with iPhone connections issue
  // https://github.com/nkolban/esp32-snippets/issues/768
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  serialPrintln("Started BLE Server.");
}

// Setup function
// This is run once, when the arduino starts
void setup() {

  beginSerial();
  setupBLE();
  tcode.StringInput("D0");
  tcode.StringInput("D1");

  // Set SR6 arms to startup positions
  if (!OSR2_MODE) { tcode.StringInput("R2750"); }

  // #ESP32# Enable EEPROM
  EEPROM.begin(320);

  // Register device axes
  tcode.RegisterAxis("L0", "Up");
  if (!OSR2_MODE) {
    tcode.RegisterAxis("L1", "Forward");
    tcode.RegisterAxis("L2", "Left");
  }
  tcode.RegisterAxis("R0", "Twist");
  tcode.RegisterAxis("R1", "Roll");
  tcode.RegisterAxis("R2", "Pitch");
  tcode.RegisterAxis("V0", "Vibe1");
  if (!LUBE_V1) { tcode.RegisterAxis("V1", "Vibe2"); }
  tcode.RegisterAxis("A0", "Valve");
  tcode.RegisterAxis("A1", "Suck");
  tcode.AxisInput("A1",VALVE_DEFAULT,'I',3000);
  if (LUBE_V1) {
    tcode.RegisterAxis("A2", "Lube");
    tcode.AxisInput("A2",0,' ',0);
    pinMode(Lube_PIN,INPUT);
  }

  // Setup Servo PWM channels
  // Lower Left Servo
  ledcSetup(LowerLeftServo_PWM,MainServo_Freq,16);
  ledcAttachPin(LowerLeftServo_PIN,LowerLeftServo_PWM);
  // Upper Left Servo
  ledcSetup(UpperLeftServo_PWM,MainServo_Freq,16);
  ledcAttachPin(UpperLeftServo_PIN,UpperLeftServo_PWM);
  // Lower Right Servo
  ledcSetup(LowerRightServo_PWM,MainServo_Freq,16);
  ledcAttachPin(LowerRightServo_PIN,LowerRightServo_PWM);
  // Upper Right Servo
  ledcSetup(UpperRightServo_PWM,MainServo_Freq,16);
  ledcAttachPin(UpperRightServo_PIN,UpperRightServo_PWM);
  // Left Pitch Servo
  ledcSetup(LeftPitchServo_PWM,PitchServo_Freq,16);
  ledcAttachPin(LeftPitchServo_PIN,LeftPitchServo_PWM);
  // Right Pitch Servo
  ledcSetup(RightPitchServo_PWM,PitchServo_Freq,16);
  ledcAttachPin(RightPitchServo_PIN,RightPitchServo_PWM);
  // Twist Servo
  ledcSetup(TwistServo_PWM,TwistServo_Freq,16);
  ledcAttachPin(TwistServo_PIN,TwistServo_PWM);
  // Valve Servo
  ledcSetup(ValveServo_PWM,ValveServo_Freq,16);
  ledcAttachPin(ValveServo_PIN,ValveServo_PWM);

  // Set vibration PWM pins
  // Vibe0 Pin
  ledcSetup(Vibe0_PWM,VibePWM_Freq,8);
  ledcAttachPin(Vibe0_PIN,Vibe0_PWM);
  // Vibe1 Pin
  ledcSetup(Vibe1_PWM,VibePWM_Freq,8);
  ledcAttachPin(Vibe1_PIN,Vibe1_PWM); 

  // Initiate position tracking for twist
  if (TWIST_PARALLAX) {
    pinMode(TwistFeedback_PIN,INPUT);
    attachInterrupt(TwistFeedback_PIN, twistRising, RISING);
  }
  
  // Signal done
  serialPrintln("Ready!");

}

// ----------------------------
//   MAIN
// ----------------------------
// This loop runs continuously

void loop() {
  readSerial();

  // Collect inputs
  // These functions query the t-code object for the position/level at a specified time
  // Number recieved will be an integer, 0-9999
  xLin = tcode.AxisRead("L0");
  if (!OSR2_MODE) {
    yLin = tcode.AxisRead("L1");
    zLin = tcode.AxisRead("L2");
  }
  xRot = tcode.AxisRead("R0");
  yRot = tcode.AxisRead("R1");
  zRot = tcode.AxisRead("R2");
  vibe0 = tcode.AxisRead("V0");
  if (!LUBE_V1) { vibe1 = tcode.AxisRead("V1"); }
  valveCmd = tcode.AxisRead("A0");
  suckCmd = tcode.AxisRead("A1");
  if (LUBE_V1) { lube = tcode.AxisRead("A2"); }

  // If you want to mix your servos differently, enter your code below:

  // If t-wist3, calculate twist position
  if (TWIST_PARALLAX) { 
    float dutyCycle = twistPulseLength;
    dutyCycle = dutyCycle/twistPulseCycle;
    float angPos = (dutyCycle - 0.029)/0.942;
    angPos = constrain(angPos,0,1) - 0.5;
    if (angPos - twistServoAngPos < - 0.8) { twistTurns += 1; }
    if (angPos - twistServoAngPos > 0.8) { twistTurns -= 1; }
    twistServoAngPos = angPos;
    twistPos = 1000*(angPos + twistTurns);
  }

  // Calculate valve position
  // Track receiver velocity
  unsigned long t = millis();
  float upVelNow;
  if (t > tLast) {
    upVelNow = xLin - xLast;
    upVelNow /= t - tLast;
    upVel = (upVelNow + 9*upVel)/10;
  }
  tLast = t;
  xLast = xLin;
  // Use suck command if most recent
  boolean suck;
  if (tcode.AxisLast("A1") >= tcode.AxisLast("A0")) {
    suck = true;
    valveCmd = suckCmd;
  } else {
    suck = false;
  }
  // Set valve position
  if (suck) {
    if (upVel < -5) {
      valveCmd = 0;  
    } else if ( upVel < 0 ) {
      valveCmd = map(100*upVel,0,-500,suckCmd,0);
    }
  }
  valvePos = (9*valvePos + map(valveCmd,0,9999,0,1000))/10;

  // OSR2 Kinematics
  if (OSR2_MODE) {
    // Calculate arm angles
    // Linear scale inputs to servo appropriate numbers
    int stroke,roll,pitch;
    stroke = map(xLin,0,9999,-350,350);
    roll   = map(yRot,0,9999,-180,180);
    pitch  = map(zRot,0,9999,-350,350);
    ledcWrite(LowerLeftServo_PWM, map(LowerLeftServo_ZERO + stroke + roll,0,MainServo_Int,0,65535));
    ledcWrite(LowerRightServo_PWM, map(LowerRightServo_ZERO - stroke + roll,0,MainServo_Int,0,65535));
    ledcWrite(LeftPitchServo_PWM, map(LeftPitchServo_ZERO - pitch,0,PitchServo_Int,0,65535));
    // Unused servo pins.
    ledcWrite(UpperLeftServo_PWM, map(UpperLeftServo_ZERO,0,MainServo_Int,0,65535));
    ledcWrite(RightPitchServo_PWM, map(RightPitchServo_ZERO,0,PitchServo_Int,0,65535));
    ledcWrite(UpperRightServo_PWM, map(UpperRightServo_ZERO,0,MainServo_Int,0,65535));
  }
  
  // SR6 Kinematics
  else {
    // Calculate arm angles
    int roll,pitch,fwd,thrust,side;
    int out1,out2,out3,out4,out5,out6;
    roll = map(yRot,0,9999,-3000,3000);
    pitch = map(zRot,0,9999,-2500,2500);
    fwd = map(yLin,0,9999,-3000,3000);
    thrust = map(xLin,0,9999,-6000,6000);
    side = map(zLin,0,9999,-3000,3000);
    // Main arms
    out1 = SetMainServo(16248 - fwd, 1500 + thrust + roll); // Lower left servo
    out2 = SetMainServo(16248 - fwd, 1500 - thrust - roll); // Upper left servo
    out5 = SetMainServo(16248 - fwd, 1500 - thrust + roll); // Upper right servo
    out6 = SetMainServo(16248 - fwd, 1500 + thrust - roll); // Lower right servo
    // Pitchers
    out3 = SetPitchServo(16248 - fwd, 4500 - thrust,  side - 1.5*roll, -pitch);
    out4 = SetPitchServo(16248 - fwd, 4500 - thrust, -side + 1.5*roll, -pitch);
    // Set Servos
    ledcWrite(LowerLeftServo_PWM, map(LowerLeftServo_ZERO - out1,0,MainServo_Int,0,65535));
    ledcWrite(UpperLeftServo_PWM, map(UpperLeftServo_ZERO + out2,0,MainServo_Int,0,65535));
    ledcWrite(LeftPitchServo_PWM, map(constrain(LeftPitchServo_ZERO - out3,LeftPitchServo_ZERO-600,LeftPitchServo_ZERO+1000),0,PitchServo_Int,0,65535));
    ledcWrite(RightPitchServo_PWM, map(constrain(RightPitchServo_ZERO + out4,RightPitchServo_ZERO-1000,RightPitchServo_ZERO+600),0,PitchServo_Int,0,65535));
    ledcWrite(UpperRightServo_PWM, map(UpperRightServo_ZERO - out5,0,MainServo_Int,0,65535));
    ledcWrite(LowerRightServo_PWM, map(LowerRightServo_ZERO + out6,0,MainServo_Int,0,65535));
  }

  // Twist and valve
  int twist,valve;
  if (TWIST_PARALLAX) { 
    twist  = (xRot - map(twistPos,-1500,1500,9999,0))/5;
    twist  = constrain(twist, -750, 750);
  } else {
    twist  = map(xRot,0,9999,1000,-1000);
    if (REVERSE_TWIST_SERVO) { twist = -twist; }
  }
  valve  = valvePos - 500;
  valve  = constrain(valve, -500, 500);
  if (REVERSE_VALVE_SERVO) { valve = -valve; }
  // Set Servos
  ledcWrite(TwistServo_PWM, map(TwistServo_ZERO + twist,0,TwistServo_Int,0,65535));
  ledcWrite(ValveServo_PWM, map(ValveServo_ZERO + valve,0,ValveServo_Int,0,65535));

  // Done with servo channels

  // Output vibration channels
  // These should drive PWM pins connected to vibration motors via MOSFETs or H-bridges.
  if (vibe0 > 0 && vibe0 <= 9999) {
    ledcWrite(Vibe0_PWM, map(vibe0,1,9999,31,255));
  } else {
    ledcWrite(Vibe0_PWM, 0);
  }
  if (!LUBE_V1 && vibe1 > 0 && vibe1 <= 9999) {
    ledcWrite(Vibe1_PWM, map(vibe1,1,9999,31,255));
  } else {
    ledcWrite(Vibe1_PWM, 0);
  }
  // Vibe timeout functions - shuts the vibne channels down if not commanded for a specified interval
  if (millis() - tcode.AxisLast("V0") > VIBE_TIMEOUT) { tcode.AxisInput("V0",0,'I',500); }
  if (!LUBE_V1 && millis() - tcode.AxisLast("V1") > VIBE_TIMEOUT) { tcode.AxisInput("V1",0,'I',500); }
  
  // Done with vibration channels

  // Lube functions
  if (LUBE_V1) {
    if (lube > 0 && lube <= 9999) {
      ledcWrite(Vibe1_PWM, map(lube,1,9999,127,255));
    } else if (digitalRead(Lube_PIN) == HIGH) {
      ledcWrite(Vibe1_PWM,Lube_SPEED);
    } else { 
      ledcWrite(Vibe1_PWM,0);
    }
    if (millis() - tcode.AxisLast("A2") > 500) { tcode.AxisInput("A2",0,' ',0); } // Auto cutoff
  }

  // Done with lube
}


// Function to calculate the angle for the main arm servos
// Inputs are target x,y coords of receiver pivot in 1/100 of a mm
int SetMainServo(float x, float y) {
  x /= 100; y /= 100;          // Convert to mm
  float gamma = atan2(x,y);    // Angle of line from servo pivot to receiver pivot
  float csq = sq(x) + sq(y);   // Square of distance between servo pivot and receiver pivot
  float c = sqrt(csq);         // Distance between servo pivot and receiver pivot
  float beta = acos((csq - 28125)/(100*c));  // Angle between c-line and servo arm
  int out = ms_per_rad*(gamma + beta - 3.14159); // Servo signal output, from neutral
  return out;
}


// Function to calculate the angle for the pitcher arm servos
// Inputs are target x,y,z coords of receiver upper pivot in 1/100 of a mm
// Also pitch in 1/100 of a degree
int SetPitchServo(float x, float y, float z, float pitch) {
  pitch *= 0.0001745; // Convert to radians
  x += 5500*sin(0.2618 + pitch);
  y -= 5500*cos(0.2618 + pitch);
  x /= 100; y /= 100; z /= 100;   // Convert to mm
  float bsq = 36250 - sq(75 + z); // Equivalent arm length
  float gamma = atan2(x,y);       // Angle of line from servo pivot to receiver pivot
  float csq = sq(x) + sq(y);      // Square of distance between servo pivot and receiver pivot
  float c = sqrt(csq);            // Distance between servo pivot and receiver pivot
  float beta = acos((csq + 5625 - bsq)/(150*c)); // Angle between c-line and servo arm
  int out = ms_per_rad*(gamma + beta - 3.14159); // Servo signal output, from neutral
  return out;
}


// T-wist3 parallax position detection functions
void twistRising() {
  attachInterrupt(TwistFeedback_PIN, twistFalling, FALLING);
  twistPulseCycle = micros()-twistPulseStart;
  twistPulseStart = micros();
}
void twistFalling() {
  attachInterrupt(TwistFeedback_PIN, twistRising, RISING);
  twistPulseLength = micros()-twistPulseStart;
}
