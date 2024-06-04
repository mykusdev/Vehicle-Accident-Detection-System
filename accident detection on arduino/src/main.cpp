#include <Wire.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>  

// Pin assignments for GSM module
const int gsmRxPin = 16;  // ESP32 RX pin (you can choose any GPIO)
const int gsmTxPin = 17;  // ESP32 TX pin (you can choose any GPIO)

HardwareSerial gsmSerial(1);  // Use hardware serial port 1 for GSM

// LCD settings
const int lcdAddress = 0x27;  // I2C address of your LCD
const int lcdCols = 20;       // Number of columns on your LCD
const int lcdRows = 4;        // Number of rows on your LCD

// Create LCD object
LiquidCrystal_I2C lcd(lcdAddress, lcdCols, lcdRows);

// Pin assignments
const int ledPin = 2;         // LED pin (built-in LED on ESP32)
const int buttonPin = 13;     // Button pin (you can choose any GPIO)
const int microphonePin = 34; // Microphone analog pin (ADC1_CH6)
const int xPin = 35;          // X-axis analog pin (ADC1_CH7)
const int yPin = 32;          // Y-axis analog pin (ADC1_CH4)
const int zPin = 33;          // Z-axis analog pin (ADC1_CH5)

// Threshold values for each axis
const int minXThreshold = -500;  // Example minimum threshold for x-axis
const int maxXThreshold = 500;   // Example maximum threshold for x-axis
const int minYThreshold = -500;  // Example minimum threshold for y-axis
const int maxYThreshold = 500;   // Example maximum threshold for y-axis
const int minZThreshold = -500;  // Example minimum threshold for z-axis
const int maxZThreshold = 500;   // Example maximum threshold for z-axis

// Other constants
const int microphoneThreshold = 100;

double accidentLatitude = 0.0;  // Variable to store accident latitude
double accidentLongitude = 0.0; // Variable to store accident longitude

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200); // Higher baud rate for ESP32
  
  // Initialize the Wire library with the correct pins
  Wire.begin(21, 22); // SDA = 21, SCL = 22
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor

  // Start GPS
  Serial2.begin(9600, SERIAL_8N1, 4, 5);  // Use hardware serial port 2 for GPS (RX=4, TX=5)
}

bool isAccident(int xValue, int yValue, int zValue) {
  // Check if any axis value is outside the specified range
  return (xValue < minXThreshold || xValue > maxXThreshold ||
          yValue < minYThreshold || yValue > maxYThreshold ||
          zValue < minZThreshold || zValue > maxZThreshold);
}

void triggerAlarm() {
  digitalWrite(ledPin, HIGH);
  Serial.println("Accident detected! Alarm activated.");
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Accident detected!");
  delay(1000);
}

bool userResponded() {
  // Check if the button is pressed (user response)
  return digitalRead(buttonPin) == LOW; // Change to LOW due to pull-up resistor
}

void sendSms(const char* phoneNumber, const char* message) {
  // Initialize GSM module
  gsmSerial.begin(9600, SERIAL_8N1, gsmRxPin, gsmTxPin); // Set RX and TX pins for GSM
  
  // Wait for the GSM module to initialize
  delay(1000);

  // Initiating communication
  gsmSerial.print("AT+CMGF=1\r");  // Set SMS mode to text
  delay(100);
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(phoneNumber);
  gsmSerial.print("\"\r");
  delay(100);  // Delay for GSM module processing

  lcd.clear();
  lcd.print("Initiating communication...");
  delay(1500);  // Delay for LCD display

  // Send the message
  gsmSerial.print(message);
  delay(100);

  lcd.clear();
  lcd.print("Sending SMS...");
  delay(1500);

  gsmSerial.write(26);  // Ctrl+Z to end the SMS
  delay(1000);

  lcd.clear();
  lcd.print("SMS sent!");
  delay(1500);

  // Close communication with the GSM module
  gsmSerial.end();

  lcd.clear();
  lcd.print("Close communication with GSM Module");
  delay(10000);
}

void notifyEmergencyServices() {
  Serial.println("Notifying emergency services...");
  lcd.clear();
  lcd.print("Notifying emergency services...");
  delay(2000);
  
  // Print the stored GPS coordinates on the LCD
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Latitude: ");
  lcd.print(accidentLatitude, 6);
  lcd.setCursor(0, 2);
  lcd.print("Longitude: ");
  lcd.print(accidentLongitude, 6);

  // Send SMS to the specified phone number
  sendSms("+917001234567", ("An accident has occurred at Latitude: " + String(accidentLatitude, 6) +
                            " and Longitude: " + String(accidentLongitude, 6)).c_str());

  delay(2000);
}

void resetSystem() {
  digitalWrite(ledPin, LOW);
  Serial.println("Alarm turned off. System reset.");
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Alarm turned off.");
  lcd.setCursor(0, 2);
  lcd.print("System reset.");
  delay(10000);
}

void loop() {
  lcd.clear();

  // Read accelerometer data
  int xValue = analogRead(xPin);
  int yValue = analogRead(yPin);
  int zValue = analogRead(zPin);

  // Read microphone data
  int microphoneValue = analogRead(microphonePin);

  // Check for accident conditions
  if (isAccident(xValue, yValue, zValue) && microphoneValue > microphoneThreshold) {
    triggerAlarm();

    if (gps.location.isValid()) {
      accidentLatitude = gps.location.lat();
      accidentLongitude = gps.location.lng();
    }

    Serial.println("Push the button if you are okay.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Push the button");
    lcd.setCursor(0, 1);
    lcd.print("if you are okay.");
    lcd.setCursor(0, 2);
    lcd.print("Wait 3s");

    delay(3000);

    int countdown = 10; // Set the countdown duration
    bool buttonPressed = false;
    for (int i = countdown; i > 0; i--) {
      digitalWrite(ledPin, HIGH);
      lcd.clear();
      lcd.print("Waiting for ");
      lcd.print(i);
      lcd.setCursor(0, 1);
      lcd.print(" seconds.");

      // Inner loop for more frequent button checks
      for (int j = 0; j < 1000; j++) {
          if (digitalRead(buttonPin) == LOW) { // Change to LOW due to pull-up resistor
              buttonPressed = true;
              break; // Exit the inner loop if the button is pressed
          }
          delay(1); // Check button every 1ms
      }

      if (buttonPressed) {
          break; // Exit the outer loop if the button is pressed
      }
    }

    if (buttonPressed) {
      resetSystem();
      return;
    }

    // If the user does not respond within the countdown period, notify emergency services
    notifyEmergencyServices();
  }
}
