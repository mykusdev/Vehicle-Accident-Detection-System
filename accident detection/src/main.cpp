#include <Wire.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>
#include <TensorFlowLite_ESP32.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
// #include <ArduinoJson.h>
#include <SPI.h>
// #include <SdFat.h>
// #include <SPIFFS.h> // Include SPIFFS library
#include "model.tflite.h"

// Define a structure to hold hospital data
struct Hospital {
  String name;
  double latitude;
  double longitude;
  String phone;
  int bedsAvailable;
};

  // Define hospital data directly in the code
  Hospital hospitals[] = {
    {"Apollo Gleneagles Hospital", 22.5351, 88.3643, "+91 33 2320 3040", 200},
    {"Belle Vue Clinic", 22.5365, 88.3592, "+91 33 2287 2321", 150},
    {"Fortis Hospital Kolkata", 22.5226, 88.3654, "+91 33 6628 4444", 180},
    {"Medica Superspecialty Hospital", 22.5717, 88.4222, "+91 33 6652 0000", 220},
    {"Desun Hospital and Heart Institute", 22.4995, 88.3859, "+91 33 7122 2200", 0}, // No beds available
    {"AMRI Hospital", 22.5762, 88.4048, "+91 33 6614 7700", 170},
    {"Ruby General Hospital", 22.5177, 88.4003, "+91 33 3987 6767", 0}, // No beds available
    {"Woodlands Hospital", 22.5415, 88.3508, "+91 33 6622 9900", 190},
    {"Peerless Hospital", 22.4931, 88.3795, "+91 33 4011 1222", 160},
    {"Columbia Asia Hospital", 22.6038, 88.4271, "+91 33 3989 8969", 0}, // No beds available
    {"Rabindranath Tagore International Institute of Cardiac Sciences", 22.5763, 88.4493, "+91 33 2556 0051", 240},
    {"CMRI Hospital", 22.5455, 88.3491, "+91 33 3090 3090", 230},
    {"ILS Hospitals", 22.5969, 88.3675, "+91 33 4033 3000", 180},
    {"NH Rabindranath Tagore Surgical Centre", 22.6303, 88.3764, "+91 33 4050 6500", 170},
    {"Vivekananda Institute of Medical Sciences", 22.6643, 88.4245, "+91 33 2680 6101", 220},
    {"IQ City Medical College and Narayana Multispeciality Hospital", 23.6215, 87.0834, "+91 81700 09111", 0}, // No beds available
    {"AMRI Hospitals Mukundapur", 22.4956, 88.4006, "+91 33 6614 7700", 200},
    {"Peerless Hospital Durgapur", 23.5276, 87.3311, "+91 343 254 4300", 180},
    {"B.C. Roy Memorial Hospital", 22.5322, 88.3633, "+91 33 2350 8122", 150},
    {"EEDF", 22.5636, 88.3681, "+91 33 2356 7331", 170}
};

int numHospitals = sizeof(hospitals) / sizeof(hospitals[0]);

// Pin assignments for GSM module
const int gsmRxPin = 16;  // ESP32 RX pin
const int gsmTxPin = 17;  // ESP32 TX pin

HardwareSerial gsmSerial(1);  // Use hardware serial port 1 for GSM

// LCD settings
const int lcdAddress = 0x27;  // I2C address of your LCD
const int lcdCols = 20;       // Number of columns on your LCD
const int lcdRows = 4;        // Number of rows on your LCD

// Create LCD object
LiquidCrystal_I2C lcd(lcdAddress, lcdCols, lcdRows);

// Pin assignments
const int ledPin = 2;         // LED pin (built-in LED on ESP32)
const int buttonPin = 13;     // Button pin
const int microphonePin = 34; // Microphone analog pin
const int xPin = 35;          // X-axis analog pin
const int yPin = 32;          // Y-axis analog pin
const int zPin = 33;          // Z-axis analog pin

const int microphoneThreshold = 100;

double accidentLatitude = 23.52;  // Variable to store accident latitude
double accidentLongitude = 87.33;

TinyGPSPlus gps;

// TensorFlow Lite variables
tflite::MicroErrorReporter tflErrorReporter;
tflite::ErrorReporter* errorReporter = &tflErrorReporter;

const tflite::Model* model;
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;

constexpr int tensorArenaSize = 2 * 1024;
uint8_t tensorArena[tensorArenaSize];

// Function prototypes
void parseHospitals(const char* jsonString, Hospital hospitals[], int numHospitals);

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

  // Load TensorFlow Lite model
  model = tflite::GetModel(model_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    errorReporter->Report("Model provided is schema version %d not equal to supported version %d.",
                          model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  static tflite::AllOpsResolver resolver;
  interpreter = new tflite::MicroInterpreter(model, resolver, tensorArena, tensorArenaSize, errorReporter);
  interpreter->AllocateTensors();

  input = interpreter->input(0);
  output = interpreter->output(0);
}

void triggerAlarm() {
  digitalWrite(ledPin, HIGH);
  Serial.println("Accident detected! Alarm activated.");
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Accident detected!");
  delay(1000);
}

// Calculate distance between two coordinates using Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  // Convert latitude and longitude from degrees to radians
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  // Haversine formula
  double dlon = lon2 - lon1;
  double dlat = lat2 - lat1;
  double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = 6371 * c; // Radius of the Earth in km
  return distance;
}

// Find the nearest hospital with available beds
int findNearestHospital(Hospital hospitals[], int numHospitals, double accidentLat, double accidentLon) {
  double minDistance = DBL_MAX;
  int nearestHospitalIndex = -1;

  for (int i = 0; i < numHospitals; i++) {
    double distance = calculateDistance(accidentLat, accidentLon, hospitals[i].latitude, hospitals[i].longitude);
    Serial.print("Distance to ");
    Serial.print(hospitals[i].name);
    Serial.print(": ");
    Serial.println(distance);
    if (distance < minDistance && hospitals[i].bedsAvailable > 0) {
      minDistance = distance;
      nearestHospitalIndex = i;
    }
  }

  Serial.print("Nearest hospital index: ");
  Serial.println(nearestHospitalIndex);
  return nearestHospitalIndex;
}

bool userResponded() {
  // Check if the button is pressed (user response)
  return digitalRead(buttonPin) == LOW; // Change to LOW due to pull-up resistor
}

void sendSms(String phoneNumber, const char* message, String hospitalName) {
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
  delay(100);  //
  lcd.clear();
  lcd.print("Initiating communication...");
  delay(1500);  // Delay for LCD display

  // Send the message
  gsmSerial.print(message);
  delay(100);

  // End the SMS with a Ctrl+Z character
  gsmSerial.write(26);
  delay(1000);  // Wait for the SMS to be sent
  Serial.println("SMS sent.");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SMS sent to");
  lcd.setCursor(0,1);
  lcd.print(phoneNumber);
  delay(2000);
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print(hospitalName);
  delay(2000);  // Delay for LCD display
}

void notifyEmergencyServices() {
  Serial.println("Notifying emergency services...");
  lcd.clear();

  lcd.setCursor(0, 1);
  lcd.print("Notifying emergency");

  lcd.setCursor(0, 2);
  lcd.print("services...");

  if (gps.location.isValid()) {
    accidentLatitude = gps.location.lat();
    accidentLongitude = gps.location.lng();
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Latitude: ");
  lcd.print(accidentLatitude, 6);
  lcd.setCursor(0, 1);
  lcd.print("Longitude: ");
  lcd.print(accidentLongitude, 6);

  int nearestHospitalIndex = findNearestHospital(hospitals, numHospitals, accidentLatitude, accidentLongitude);

  if (nearestHospitalIndex != -1) {
    // Send SMS to the nearest hospital
    sendSms(hospitals[nearestHospitalIndex].phone, "Emergency: Accident nearby, beds needed.", hospitals[nearestHospitalIndex].name);
  } else {
    // Send SMS to a default emergency contact
    sendSms("+917001065717", ("An accident has occurred at Latitude: " + String(accidentLatitude, 6) +
                              " and Longitude: " + String(accidentLongitude, 6)).c_str(), "Sent to saved contact");
  }

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

  // Normalize the input values and set them to the model's input tensor
  input->data.f[0] = xValue / 1024.0f;
  input->data.f[1] = yValue / 1024.0f;
  input->data.f[2] = zValue / 1024.0f;
  input->data.f[3] = microphoneValue / 1024.0f;

  // Run inference
  TfLiteStatus invokeStatus = interpreter->Invoke();
  if (invokeStatus != kTfLiteOk) {
    Serial.println("Invoke failed");
    return;
  }

  // Get the result
  float accidentProbability = output->data.f[0];

  // Simulate accident
  accidentProbability = 0.7;

  if (accidentProbability > 0.5) {
    // Detected an accident
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

    int countdown = 5; // Set the countdown duration
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
        delay(1);
      }

      if (buttonPressed) {
        break; // Exit the outer loop if the button is pressed
      }

      digitalWrite(ledPin, LOW);
      delay(500); // Adjust delay as needed
    }

    if (buttonPressed) {
      resetSystem();
    } else {
      notifyEmergencyServices();
    }
  } else {
    // No accident detected
    digitalWrite(ledPin, LOW);
    lcd.setCursor(0, 1);
    lcd.print("No accident detected.");
    delay(1000);
  }
}
