arduino


2(Traffic Light)
// Experiment: 2A 4 way traffic light using Arduino

#define Red_N 13
#define Yellow_N 12
#define Green_N 11
#define Red_E 10
#define Yellow_E 9
#define Green_E 8
#define Red_S 7
#define Yellow_S 6
#define Green_S 5
#define Red_W 4
#define Yellow_W 3
#define Green_W 2
bool red1[] =
{
true,true,true,true,true,true,true, true,true,true,true,true,true,true, true,true,true,true,true,true,true, false,false,false,false,false,false,false
};
bool yellow1[] =
{
false,false,false,false,false,false,false, false,false,false,false,false,false,false, false,false,false,false,false,false,false,
false,false,false,false,false,false,true
};
bool green1[] =
{
false,false,false,false,false,false,false, false,false,false,false,false,false,false, false,false,false,false,false,false,false, true,true,true,true,true,true,false
};
bool red2[] =
{
true,true,true,true,true,true,true, true,true,true,true,true,true,true, false,false,false,false,false,false,false, true,true,true,true,true,true,true
};
bool yellow2[] =
{
false,false,false,false,false,false,false, false,false,false,false,false,false,false, false,false,false,false,false,false,true, false,false,false,false,false,false,false
};
bool green2[] =
{
false,false,false,false,false,false,false, false,false,false,false,false,false,false, true,true,true,true,true,true,false, false,false,false,false,false,false,false
};
bool red3[] =
{
true,true,true,true,true,true,true, false,false,false,false,false,false,false, true,true,true,true,true,true,true, true,true,true,true,true,true,true
};
bool yellow3[] =
{
false,false,false,false,false,false,false, false,false,false,false,false,false,true, false,false,false,false,false,false,false, false,false,false,false,false,false,false
};
bool green3[] =
{
false,false,false,false,false,false,false, true,true,true,true,true,true,false, false,false,false,false,false,false,false, false,false,false,false,false,false,false
};
bool red4[] =
{
false,false,false,false,false,false,false, true,true,true,true,true,true,true, true,true,true,true,true,true,true, true,true,true,true,true,true,true
};
bool yellow4[] =
{
false,false,false,false,false,false,true, false,false,false,false,false,false,false, false,false,false,false,false,false,false, false,false,false,false,false,false,false
};
bool green4[] =
{
true,true,true,true,true,true,false, false,false,false,false,false,false,false, false,false,false,false,false,false,false, false,false,false,false,false,false,false
};
void setup() { Serial.begin(9600); pinMode(Red_N,OUTPUT); pinMode(Yellow_N,OUTPUT); pinMode(Green_N,OUTPUT); pinMode(Red_E,OUTPUT); pinMode(Yellow_E,OUTPUT); pinMode(Green_E,OUTPUT); pinMode(Red_S,OUTPUT); pinMode(Yellow_S,OUTPUT); pinMode(Green_S,OUTPUT); pinMode(Red_W,OUTPUT); pinMode(Yellow_W,OUTPUT); pinMode(Green_W,OUTPUT);
}
void loop()
{
for (int i = 0; i<28;i++)
{
digitalWrite(Red_N, red1[i]); digitalWrite(Yellow_N, yellow1[i]); digitalWrite(Green_N, green1[i]); Serial.print(red1[i]); Serial.print(yellow1[i]); Serial.println(green1[i]); digitalWrite(Red_E, red2[i]); digitalWrite(Yellow_E, yellow2[i]); digitalWrite(Green_E, green2[i]); Serial.print(red2[i]); Serial.print(yellow2[i]); Serial.println(green2[i]); digitalWrite(Red_S, red3[i]); digitalWrite(Yellow_S, yellow3[i]); digitalWrite(Green_S, green3[i]); Serial.print(red3[i]); Serial.print(yellow3[i]); Serial.println(green3[i]); digitalWrite(Red_W, red4[i]); digitalWrite(Yellow_W, yellow4[i]); digitalWrite(Green_W, green4[i]); Serial.print(red4[i]); Serial.print(yellow4[i]); Serial.println(green4[i]); delay(5000);
}
}


// Experiment: 3AA Interfacing LM35 temp sensor with arduino and displaying output on
OLED display

#include <Wire.h> // Include Wire library for I2C communication
#include <Adafruit_GFX.h> // Include Adafruit GFX library for graphics display
#include <Adafruit_SSD1306.h> // Include Adafruit SSD1306 library for OLED display
#define SCREEN_WIDTH 128 // OLED display width in pixels
#define SCREEN_HEIGHT 64 // OLED display height in pixels
// Create an SSD1306 display object connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
const int sensorPin = A0; // Define the analog pin for the temperature sensor (LM35)
float sensorValue; // Variable to store the analog reading
float voltageOut; // Variable to store the converted voltage from the sensor
float temperatureC; // Variable to store temperature in Celsius
float temperatureF; // Variable to store temperature in Fahrenheit
// Setup function
void setup() {
pinMode(sensorPin, INPUT); // Set the sensor pin as input
Serial.begin(9600); // Initialize serial communication at 9600 baud for debugging
// Initialize the OLED display
if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // SSD1306_SWITCHCAPVCC sets the OLED with internal voltage generator
Serial.println(F("SSD1306 allocation failed")); // Display allocation error
for(;;); // Stop if display initialization fails
}
delay(2000); // Wait for 2 seconds before starting
display.clearDisplay(); // Clear the display
display.setTextColor(WHITE); // Set text color to white (on black background)
}
// Main loop function
void loop() {
// Read the analog sensor value and convert it to voltage
sensorValue = analogRead(sensorPin); // Read analog input from the sensor
voltageOut = sensorValue * (5.0 / 1024); // Convert analog reading to voltage (assuming 5V system)
// Calculate temperature in Celsius using LM35 formula
temperatureC = voltageOut * 100; // LM35 outputs 10mV per degree Celsius
// Convert temperature from Celsius to Fahrenheit
temperatureF = (temperatureC * 1.8) + 32; // Convert Celsius to Fahrenheit
// Print the temperature readings to the Serial Monitor
Serial.print("Temperature (°C): ");
Serial.print(temperatureC);
Serial.print(" Temperature (°F): ");
Serial.println(temperatureF);
// Clear the OLED display
display.clearDisplay();
// Display temperature in Celsius on OLED
display.setTextSize(1); // Set text size to 1 (small)
display.setCursor(0, 0); // Set cursor to the top-left corner
display.print("Temperature: ");
display.setTextSize(2); // Set text size to 2 (larger)
display.setCursor(0, 10); // Set cursor slightly lower
display.print(temperatureC); // Print temperature in Celsius
display.print(" "); // Add a space
display.setTextSize(1); // Set text size to 1 for degree symbol
display.cp437(true); // Enable extended ASCII for degree symbol
display.write(167); // Write degree symbol (ASCII 167)
display.setTextSize(2); // Set text size back to 2
display.print("C"); // Print "C" for Celsius
// Display temperature in Fahrenheit on OLED
display.setTextSize(1); // Set text size to 1 (small)
display.setCursor(0, 35); // Set cursor lower to display Fahrenheit
display.print("Temperature: ");
display.setTextSize(2); // Set text size to 2 (larger)
display.setCursor(0, 45); // Set cursor slightly lower
display.print(temperatureF); // Print temperature in Fahrenheit
display.print(" "); // Add a space
display.setTextSize(1); // Set text size to 1 for degree symbol
display.cp437(true); // Enable extended ASCII for degree symbol
display.write(167); // Write degree symbol (ASCII 167)
display.setTextSize(2); // Set text size back to 2
display.print("F"); // Print "F" for Fahrenheit
display.display(); // Send buffer data to display
delay(1000); // Wait for 1 second before refreshing
}

// Experiment: 3A Interfacing LM35 temp sensor with arduino and displaying output on
LCD display

#include <LiquidCrystal_I2C.h> // Include the LiquidCrystal I2C library to handle the LCD
LiquidCrystal_I2C lcd(0x3F, 16, 2); // Initialize the LCD at address 0x3F with 16 columns and 2 rows
// Custom character for the degree symbol
byte Degree[] = {
B00111, // Top part of the degree symbol
B00101, // Middle part of the degree symbol
B00111, // Bottom part of the degree symbol
B00000, // Empty part
B00000, // Empty part
B00000, // Empty part
B00000, // Empty part
B00000 // Empty part
};
#define sensorPin A0 // Define the analog pin A0 as sensorPin for reading the temperature
void setup() {
lcd.init(); // Initialize the LCD
lcd.backlight(); // Turn on the LCD backlight
lcd.createChar(0, Degree); // Create a custom character for the degree symbol and store it in index0
}
void loop() {
int reading = analogRead(sensorPin); // Read the analog value from the sensor
float voltage = reading * (5.0 / 1024.0); // Convert the analog reading to voltage
float temperatureC = voltage * 100; // Convert the voltage to temperature in Celsius
lcd.setCursor(0, 0); // Set cursor to the first row, first column
lcd.print("Temperature:"); // Print the label "Temperature:" on the LCD
lcd.setCursor(0, 1); // Set cursor to the second row, first column
lcd.print(temperatureC, 1); // Print the temperature in Celsius with 1 decimal place
lcd.write(0); // Print the custom degree symbol
lcd.print("C "); // Print "C " after the temperature in Celsius
float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0; // Convert Celsius to Fahrenheit
lcd.print(temperatureF, 1); // Print the temperature in Fahrenheit with 1 decimal place
lcd.write(0); // Print the custom degree symbol
lcd.print("F "); // Print "F " after the temperature in Fahrenheit
delay(1000); // Wait for 1 second before updating the display again
}


// Experiment: 3B Interfacing DHT11 temp sensor with arduino and displaying output on
serial monitor

#include "DHT.h" // Include the DHT library to communicate with the DHT sensor
#define DHTPIN 2 // Define the pin where the DHT sensor is connected
#define DHTTYPE DHT11 // Specify that the sensor type is DHT11
DHT dht(DHTPIN, DHTTYPE); // Create a DHT object with the specified pin and sensor type
void setup() {
Serial.begin(9600); // Initialize serial communication at 9600 bps
Serial.println("DHTxx test!"); // Print a message to the Serial Monitor for debugging purposes
dht.begin(); // Initialize the DHT sensor
}
void loop() {
delay(2000); // Wait 2 seconds between sensor readings (DHT11 requires at least 2 seconds between readings)
// Read the humidity from the DHT sensor
float h = dht.readHumidity();
// Read the temperature in Celsius
float t = dht.readTemperature();
// Read the temperature in Fahrenheit (passing 'true' to get Fahrenheit)
float f = dht.readTemperature(true);
// Check if any readings failed (if NaN), and if so, print an error message
if (isnan(h) || isnan(t) || isnan(f)) {
Serial.println("Failed to read from DHT sensor!"); // Print error if sensor reading fails
return; // Exit the loop if readings failed
}
// Compute the heat index (apparent temperature) in Fahrenheit using temperature (F) and humidity
float hi = dht.computeHeatIndex(f, h);
// Print humidity to the Serial Monitor
Serial.print("Humidity: ");
Serial.print(h);
Serial.print(" %\t"); // Add tab space
// Print temperature in Celsius to the Serial Monitor
Serial.print("Temperature: ");
Serial.print(t);
Serial.print(" *C "); // Print Celsius symbol
// Print temperature in Fahrenheit to the Serial Monitor
Serial.print(f);
Serial.print(" *F\t"); // Print Fahrenheit symbol and add tab space
// Print the heat index (apparent temperature) in Fahrenheit
Serial.print("Heat index: ");
Serial.print(hi);
Serial.println(" *F"); // Print Fahrenheit symbol and move to the next line
}



// Experiment: 3C Interfacing Soil moisture sensor YL-69 or HL-69 with arduino

int rainPin = A0; // Define the pin (A0) where the rain/moisture sensor is connected
int greenLED = 6; // Define the pin for the green LED
int redLED = 7; // Define the pin for the red LED
int thresholdValue = 800; // Set the threshold value to determine if watering is needed
void setup() {
pinMode(rainPin, INPUT); // Set the rainPin as an input to read sensor values
pinMode(greenLED, OUTPUT); // Set the green LED pin as an output
pinMode(redLED, OUTPUT); // Set the red LED pin as an output
digitalWrite(greenLED, LOW); // Initialize the green LED as off
digitalWrite(redLED, LOW); // Initialize the red LED as off
Serial.begin(9600); // Initialize serial communication at 9600 baud for debugging
}
void loop() {
int sensorValue = analogRead(rainPin); // Read the analog value from the rain sensor
Serial.print(sensorValue); // Print the sensor value to the Serial Monitor
// Check if the sensor value is less than the threshold (plant doesn't need watering)
if (sensorValue < thresholdValue) {
Serial.println(" - Doesn't need watering"); // Print message indicating no watering needed
digitalWrite(redLED, LOW); // Turn off the red LED
digitalWrite(greenLED, HIGH); // Turn on the green LED (indicating sufficient moisture)
}
// If the sensor value is above the threshold (plant needs watering)
else {
Serial.println(" - Time to water your plant"); // Print message indicating watering needed
digitalWrite(redLED, HIGH); // Turn on the red LED (indicating dry soil)
digitalWrite(greenLED, LOW); // Turn off the green LED
}
delay(500); // Wait for 500 milliseconds before taking the next reading
}



// Experiment: 3D Interfacing Ultrasonic Sensor HC-SR04 with Arduino

#include "NewPing.h" // Include the NewPing library to handle ultrasonic sensor operations
#define TRIGGER_PIN 9 // Define the pin connected to the trigger pin of the ultrasonic sensor
#define ECHO_PIN 10 // Define the pin connected to the echo pin of the ultrasonic sensor
#define MAX_DISTANCE 400 // Set the maximum distance (in cm) the sensor can measure (400 cm)
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Create a NewPing object to manage the ultrasonic sensor with the defined trigger and echo pins, and max distance
void setup() {
Serial.begin(9600); // Initialize serial communication at 9600 baud for debugging
}
void loop() {
Serial.print("Distance = "); // Print the label "Distance = " to the Serial Monitor
Serial.print(sonar.ping_cm()); // Measure the distance using the ultrasonic sensor and print the result in centimeters
Serial.println(" cm"); // Print " cm" and move to the next line
delay(500); // Wait for 500 milliseconds before taking the next reading
}



// Experiment: 3E Interfacing IR Sensor module with Arduino and display output on 7 segment
Display

#include <TM1637Display.h> // Include the library for the TM1637 7-segment display
#define CLK 2 // Define the clock pin for the TM1637 display (connected to pin 2)
#define DIO 3 // Define the data pin for the TM1637 display (connected to pin 3)
#define IR_SENSOR_PIN 4 // Define the pin where the IR sensor is connected (pin 4)
TM1637Display display(CLK, DIO); // Create a display object using the clock and data pins
int objectCount = 0; // Initialize the object count to 0
int sensorState = 0; // Variable to store the current state of the IR sensor
int lastSensorState = 0; // Variable to store the previous state of the IR sensor
void setup() {
display.setBrightness(0x0f); // Set the display brightness (0x0f is maximum brightness)
pinMode(IR_SENSOR_PIN, INPUT); // Set the IR sensor pin as an input
display.showNumberDec(objectCount); // Display the initial count (0) on the 7-segment display
}
void loop() {
sensorState = digitalRead(IR_SENSOR_PIN); // Read the current state of the IR sensor (HIGH or LOW)
// Check if the sensor has detected an object (rising edge detection: when sensorState changes from LOW to HIGH)
if (sensorState == HIGH && lastSensorState == LOW) {
objectCount++; // Increment the object count when an object is detected
display.showNumberDec(objectCount); // Update the display to show the new object count
}
lastSensorState = sensorState; // Store the current sensor state for the next loop iteration
}


// Experiment: 3F Interfacing PIR Motion Sensor module with Arduino

int led = 13; // Define the pin for the LED (pin 13, usually built-in on Arduino boards)
int sensor = 2; // Define the pin for the motion sensor (pin 2)
int state = LOW; // Variable to store the current state of motion (initially no motion, LOW)
int val = 0; // Variable to store the sensor reading
void setup() {
pinMode(led, OUTPUT); // Set the LED pin as an output
pinMode(sensor, INPUT); // Set the motion sensor pin as an input
Serial.begin(9600); // Initialize serial communication at 9600 baud for debugging
}
void loop() {
val = digitalRead(sensor); // Read the value from the motion sensor (HIGH or LOW)
// Check if motion is detected (sensor reads HIGH)
if (val == HIGH) {
digitalWrite(led, HIGH); // Turn the LED on when motion is detected
delay(100); // Small delay to avoid bouncing or rapid state changes
// If the previous state was LOW (no motion), change it to HIGH and print "Motion detected!"
if (state == LOW) {
Serial.println("Motion detected!"); // Print a message to the Serial Monitor
state = HIGH; // Update the state to HIGH (motion detected)
}
}
// If no motion is detected (sensor reads LOW)
else {
digitalWrite(led, LOW); // Turn the LED off when no motion is detected
delay(200); // Small delay to avoid bouncing
// If the previous state was HIGH (motion detected), change it to LOW and print "Motion stopped!"
if (state == HIGH) {
Serial.println("Motion stopped!"); // Print a message to the Serial Monitor
state = LOW; // Update the state to LOW (no motion)
}
}
}



// Experiment: 3G Interfacing of Servo motor module with Arduino

#include <Servo.h> // Include the Servo library to control the servo motor
int servoPin = 9; // Define the pin where the servo motor is connected (pin 9)
Servo servo; // Create a Servo object to control the servo motor
int angle = 0; // Variable to store the servo position in degrees (starting at 0)
void setup() {
servo.attach(servoPin); // Attach the servo object to the specified pin
}
void loop() {
// Sweep the servo from 0 degrees to 180 degrees
for(angle = 0; angle < 180; angle++) {
servo.write(angle); // Set the servo to the current angle
delay(15); // Wait 15 milliseconds to allow the servo to move to the position
}
// Sweep the servo from 180 degrees back to 0 degrees
for(angle = 180; angle > 0; angle--) {
servo.write(angle); // Set the servo to the current angle
delay(15); // Wait 15 milliseconds to allow the servo to move to the position
}
}


// Experiment: 4A Interfacing of inbuilt LED for ESP01 and program to blink LED for every 1 second



Code:

void setup() {
// The setup function runs once when you press reset or power the board pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
}
void loop() {
// The loop function runs over and over again forever

digitalWrite(LED_BUILTIN, HIGH);	// turn the LED on (HIGH is the voltage level) delay(1000); // wait for a second
digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW delay(1000); // wait for a second
}



// Experiment: 4B Interfacing of Relay module to ESP01 controlling the applications connected to Relay through GPIO0 port pin


Code:


void setup() {
// The setup function runs once when you press reset or power the board pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output. pinMode(0, OUTPUT);
}
void loop() {
//The loop function runs over and over again forever digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level) digitalWrite(0, LOW);
delay(1000); // wait for a second

digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW digitalWrite(0, HIGH);
delay(1000); // wait for a second
}



// Experiment: 4C Initialization of ESP01 wifi module and controlling the Relay/LED connected to GPIO0 port pin of ESP0 module using BLYNK Cloud Platform in AP mode



Code:
#define BLYNK_TEMPLATE_ID " TMPL353n_n6hR " //Blynk template ID #define BLYNK_TEMPLATE_NAME "Preshit Motghare " //Blynk template name
#define BLYNK_AUTH_TOKEN "MzKtHEr-APbgPM_iRVzEzf2mǪTStYbFl" //Blynk authentication token

// Include the necessary libraries #define BLYNK_PRINT Serial #include <ESP8266WiFi.h> #include <BlynkSimpleEsp8266.h>
// WiFi and Blynk authentication details char auth[] = BLYNK_AUTH_TOKEN; // Blynk authentication toke
char ssid[] = "phone"; // WiFi name
char pass[] = "12345687"; // WiFi password

// This function is triggered when the virtual button on the Blynk app (V0) is pressed BLYNK_WRITE(V0) {
digitalWrite(LED_BUILTIN, param.asInt()); // Control the built-in LED based on button state (1 for ON, 0 for OFF)
}
void setup()
{
pinMode(LED_BUILTIN, OUTPUT); // Set the built-in LED pin as output
Blynk.begin(auth, ssid, pass, "blynk.cloud", 80); // Connect to Blynk cloud and initialize Blynk
}
void loop()
{
Blynk.run(); // Continuously run the Blynk process
}



// Experiment: 4D Interface WiFi ESP01/ESP8266 module with Arduino UNO/NANO and control four LEDs/Relays connected to Arduino board through WiFi module Access Point (AP) without a router using BLYNK Cloud platform.

Code:
#define BLYNK_TEMPLATE_NAME "SK"
#define BLYNK_AUTH_TOKEN "VJcYMY8_BcV4_UfTh0DpǪqFnBunUH0pX" #define BLYNK_TEMPLATE_ID "TMPL36-fKaLin"
#define BLYNK_PRINT Serial

#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

// Your WiFi credentials char ssid[] = "phone"; char pass[] = "12345687";

// Software Serial pins #include <SoftwareSerial.h>
SoftwareSerial EspSerial(2, 3); // RX, TX

// ESP8266 baud rate
#define ESP8266_BAUD 115200 // Change to 115200 ESP8266 wifi(&EspSerial);
void setup()
{
// Debug console Serial.begin(115200);

// Set ESP8266 baud rate EspSerial.begin(ESP8266_BAUD)
; delay(10);

// Initialize Blynk
Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass, "blynk.cloud", 80); Void loop()
{
Blynk.run(); // Continuously run the Blynk process
}




//Title: Build an nodeMCU ESP8266 based web-server and demonstrate the
application to control the LEDs connected to respective GPIO ports of ESP6266 NodeMCU

#include <ESP8266WiFi.h> // Include the WiFi library for ESP8266
#include <ESP8266WebServer.h> // Include the web server library for ESP8266

/* Define your WiFi SSID and Password */
const char* ssid = "oplus_co_apavac"; // WiFi network name
const char* password = "12345678"; // WiFi network password
/* Define the IP Address details for the ESP8266 in AP mode */
IPAddress local_ip(192, 168, 1, 1); // Local IP address for the ESP8266
IPAddress gateway(192, 168, 1, 1); // Gateway address
IPAddress subnet(255, 255, 255, 0); // Subnet mask
ESP8266WebServer server(80); // Create a web server on port 80
// Define GPIO pins for LEDs
uint8_t LED1pin = 2; // GPIO2 corresponds to D4 (LED1)
bool LED1status = LOW; // Variable to track LED1 status
uint8_t LED2pin = 14; // GPIO14 corresponds to D5 (LED2)
bool LED2status = LOW; // Variable to track LED2 status
void setup() {
  Serial.begin(115200); // Start the serial communication at 115200 baud rate
  pinMode(LED1pin, OUTPUT); // Set LED1 pin as output
  pinMode(LED2pin, OUTPUT); // Set LED2 pin as output
  // Start the WiFi in Access Point mode
  WiFi.softAP(ssid, password); // Create an access point with the given SSID and password
  WiFi.softAPConfig(local_ip, gateway, subnet); // Configure the IP address, gateway, and subnet
  delay(100); // Delay to allow WiFi setup to complete

  // Define server routes and their associated handler functions
  server.on("/", handle_OnConnect); // Handle requests to the root URL
  server.on("/led1on", handle_led1on); // Handle request to turn LED1 on
  server.on("/led1off", handle_led1off); // Handle request to turn LED1 off
  server.on("/led2on", handle_led2on); // Handle request to turn LED2 on
  server.on("/led2off", handle_led2off); // Handle request to turn LED2 off
  server.onNotFound(handle_NotFound); // Handle requests for non-existent routes
  server.begin(); // Start the web server
  Serial.println("HTTP server started"); // Print a message to the serial monitor
}
void loop() {
  server.handleClient(); // Handle incoming client requests
  // Control LED1 based on its status
  if (LED1status) {
    digitalWrite(LED1pin, HIGH); // Turn LED1 on
  } else {
    digitalWrite(LED1pin, LOW); // Turn LED1 off
  }
  // Control LED2 based on its status
  if (LED2status) {
    digitalWrite(LED2pin, HIGH); // Turn LED2 on
  } else {
    digitalWrite(LED2pin, LOW); // Turn LED2 off
  }
}
// Function to handle requests to the root URL
void handle_OnConnect() {
  // Set both LEDs to OFF when the root page is accessed
  LED1status = LOW; 
  LED2status = LOW;
  Serial.println("GPIO7 Status: OFF | GPIO6 Status: OFF"); // Print LED statuses
  server.send(200, "text/html", SendHTML(LED1status, LED2status)); // Send HTML response
}
// Function to handle turning LED1 on
void handle_led1on() {
  LED1status = HIGH; // Set LED1 status to ON
  Serial.println("GPIO7 Status: ON"); // Print LED status
  server.send(200, "text/html", SendHTML(true, LED2status)); // Send updated HTML response
}
// Function to handle turning LED1 off
void handle_led1off() {
  LED1status = LOW; // Set LED1 status to OFF
  Serial.println("GPIO7 Status: OFF"); // Print LED status
  server.send(200, "text/html", SendHTML(false, LED2status)); // Send updated HTML response
}
// Function to handle turning LED2 on
void handle_led2on() {
  LED2status = HIGH; // Set LED2 status to ON
  Serial.println("GPIO6 Status: ON"); // Print LED status
  server.send(200, "text/html", SendHTML(LED1status, true)); // Send updated HTML response
}
// Function to handle turning LED2 off
void handle_led2off() {
  LED2status = LOW; // Set LED2 status to OFF
  Serial.println("GPIO6 Status: OFF"); // Print LED status
  server.send(200, "text/html", SendHTML(LED1status, false)); // Send updated HTML response
}

// Function to handle requests for non-existent routes
void handle_NotFound() {
  server.send(404, "text/plain", "Not found"); // Send 404 response
}

// Function to generate HTML response for the web page
String SendHTML(uint8_t led1stat, uint8_t led2stat) {
  String ptr = "<!DOCTYPE html> <html>\n"; // Start HTML document

  // Add HTML head with meta and styles
  ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<title>LED Control</title>\n";
  ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr += "body { margin-top: 50px; } h1 { color: #444444; margin: 50px auto 30px; } h3 { color: #444444; margin-bottom: 50px; }\n";
  ptr += ".button { display: block; width: 80px; background-color: #1abc9c; border: none; color: white; padding: 13px 30px; text-decoration: none; font-size: 25px; margin: 0px auto 35px; cursor: pointer; border-radius: 4px; }\n";
  ptr += ".button-on { background-color: #1abc9c; }\n";
  ptr += ".button-on:active { background-color: #16a085; }\n";
  ptr += ".button-off { background-color: #34495e; }\n";
  ptr += ".button-off:active { background-color: #2c3e50; }\n";
  ptr += "p { font-size: 14px; color: #888; margin-bottom: 10px; }\n";
  ptr += " h1{ color:#FFA500 }\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";
  ptr += "<h1>ESP8266 Web Server</h1>\n"; // Main title
  ptr += "<h3>Preshit Motghare</h3>\n"; // Subtitle
  // Display status and control buttons for LED1
  if (led1stat) {
    ptr += "<p>LED1 Status: ON</p><a class=\"button button-off\" href=\"/led1off\">OFF</a>\n";
  } else {
    ptr += "<p>LED1 Status: OFF</p><a class=\"button button-on\" href=\"/led1on\">ON</a>\n";
  }
  // Display status and control buttons for LED2
  if (led2stat) {
    ptr += "<p>LED2 Status: ON</p><a class=\"button button-off\" href=\"/led2off\">OFF</a>\n";
  } else {
    ptr += "<p>LED2 Status: OFF</p><a class=\"button button-on\" href=\"/led2on\">ON</a>\n";
  }
  ptr += "</body>\n"; // End of body
  ptr += "</html>\n"; // End of HTML document
  return ptr; // Return the generated HTML
}




//Title: Build an nodeMCU ESP8266 based web-server and demonstrate the application to control the relays connected to respective GPIO ports of ESP6266 NodeMCU

// Load Wi-Fi library for ESP8266
#include <ESP8266WiFi.h>
// Replace with your Wi-Fi network credentials
const char* ssid = "oplus_co_apavac"; // WiFi network name
const char* password = "12345678"; // WiFi network password
// Set up the web server on port 80
WiFiServer server(80);
// Variable to store the HTTP request received from clients
String header;
// Auxiliary variables to store the current output state of GPIO pins
String output5State = "off";  // State of GPIO pin 5
String output4State = "off";   // State of GPIO pin 4
// Assign output variables to GPIO pins (output pin numbers)
const int output5 = 5;         // GPIO pin 5
const int output4 = 4;         // GPIO pin 4
// Current time for tracking client connection
unsigned long currentTime = millis();
// Previous time for tracking the connection duration
unsigned long previousTime = 0;
// Define timeout time in milliseconds (e.g., 2000ms = 2s)
const long timeoutTime = 2000;
void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud rate
  // Initialize GPIO pins as outputs
  pinMode(output5, OUTPUT);
  pinMode(output4, OUTPUT);
  // Set initial output states to LOW (off)
  digitalWrite(output5, LOW);
  digitalWrite(output4, LOW);
  // Connect to the Wi-Fi network with the specified SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  // Start Wi-Fi connection
  // Wait until the ESP8266 is connected to the Wi-Fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);  // Wait for 500 ms before checking again
    Serial.print(".");  // Print a dot for each attempt to connect
  }
  // Print local IP address and start the web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());  // Print the assigned IP address
  server.begin();  // Start the web server
}
void loop() {
  WiFiClient client = server.available();  // Check for incoming clients
  if (client) {  // If a new client connects
    Serial.println("New Client.");  // Print message to the serial monitor
    String currentLine = "";  // Variable to hold the incoming data from the client
    currentTime = millis();  // Get the current time
    previousTime = currentTime;  // Set previous time to current time
    // Loop while the client is connected and within the timeout limit
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      currentTime = millis();  // Update current time
      if (client.available()) {  // If there are bytes to read from the client
        char c = client.read();  // Read a byte from the client
        Serial.write(c);  // Print the byte to the serial monitor
        header += c;  // Append the byte to the header string
        if (c == '\n') {  // If the byte is a newline character
          // If the current line is blank, we received two newlines in a row,
          // indicating the end of the client's HTTP request.
          if (currentLine.length() == 0) {
            // Send an HTTP response
            client.println("HTTP/1.1 200 OK");  // HTTP response code
            client.println("Content-type:text/html");  // Content type of the response
            client.println("Connection: close");  // Close the connection after response
            client.println();  // Blank line to indicate end of headers
            // Process GPIO commands based on the HTTP request
            if (header.indexOf("GET /5/on") >= 0) {
              Serial.println("GPIO 5 on");  // Log the action
              output5State = "on";  // Update state
              digitalWrite(output5, HIGH);  // Turn GPIO 5 on
            } else if (header.indexOf("GET /5/off") >= 0) {
              Serial.println("GPIO 5 off");  // Log the action
              output5State = "off";  // Update state
              digitalWrite(output5, LOW);  // Turn GPIO 5 off
            } else if (header.indexOf("GET /4/on") >= 0) {
              Serial.println("GPIO 4 on");  // Log the action
              output4State = "on";  // Update state
              digitalWrite(output4, HIGH);  // Turn GPIO 4 on
            } else if (header.indexOf("GET /4/off") >= 0) {
              Serial.println("GPIO 4 off");  // Log the action
              output4State = "off";  // Update state
              digitalWrite(output4, LOW);  // Turn GPIO 4 off
            }
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the buttons
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println("h1{color:#FF0000;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");
            // Web Page Heading
            client.println("<body><h1>Preshit Motghare</h1>");
            client.println("<h2>313037</h2>");
            client.println("<h2>22210307</h2>");
             client.println("<h4>Using Access Point (API) Mode</h4>");
            // Display current state and ON/OFF buttons for GPIO 5  
            client.println("<p>GPIO 5 - State " + output5State + "</p>");
            // If output5State is off, display the ON button      
            if (output5State == "off") {
              client.println("<p><a href=\"/5/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/5/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
               
            // Display current state and ON/OFF buttons for GPIO 4  
            client.println("<p>GPIO 4 - State " + output4State + "</p>");
            // If output4State is off, display the ON button      
            if (output4State == "off") {
              client.println("<p><a href=\"/4/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/4/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
           
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop after sending the response
            break;
          } else { // If a newline character is received, clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // If the character is not a carriage return
          currentLine += c;      // Add it to the currentLine string
        }
      }
    }
    // Clear the header variable for the next client
    header = "";
    // Close the connection with the client
    client.stop();  // Stop the client connection
    Serial.println("Client disconnected.");  // Log disconnection
    Serial.println("");  // Print an empty line for readability
  }
}






6th
To setup and use ESP32 Cam with Micro USB Wi-Fi Camera
Code:



#include "esp_camera.h"
#include <WiFi.h>

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

const char* ssid = "Phone";
const char* password = "12345678";

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  
config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

 
 WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000);
}




Changed  CPP Code:

// Define an array of names for 2 enrolled faces
const char* face_names[2] = {"Preshit", "Ayush"};

static int run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes){
    dl_matrix3du_t *aligned_face = NULL;
    int matched_id = -1; // Initialize to -1 for no match

    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        Serial.println("Could not allocate face recognition buffer");
        return matched_id;
    }
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){
        if (is_enrolling == 1){
            int8_t left_sample_face = enroll_face(&id_list, aligned_face);

            if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                Serial.printf("Enrolling Face ID: %d\n", id_list.tail);
            }
            Serial.printf("Enrolling Face ID: %d sample %d\n", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
           
 if (left_sample_face == 0){
                
is_enrolling = 0;

                Serial.printf("Enrolled Face ID: %d\n", id_list.tail);
            }
        } else {
            matched_id = recognize_face(&id_list, aligned_face);
            if (matched_id >= 0 && matched_id < 2) {
                Serial.printf("Match Face ID: %u\n", matched_id);
                rgb_printf(image_matrix, FACE_COLOR_GREEN, "Hello %s", face_names[matched_id]);
            } else {
                Serial.println("No Match Found");
                rgb_print(image_matrix, FACE_COLOR_RED, "Intruder Alert!");
                matched_id = -1;
            }
        }
    } else {
        Serial.println("Face Not Aligned");
        //rgb_print(image_matrix, FACE_COLOR_YELLOW, "Human Detected");
    }

    dl_matrix3du_free(aligned_face);
    return matched_id;
}




Code: 7a: Interfacing LEDs with Raspberry Pi GPIO pins.


# Import the RPi.GPIO module to control GPIO pins on the Raspberry Pi
import RPi.GPIO as GPIO
# Import the sleep function from the time module to add delays in the code
from time import sleep
# Disable any warning messages related to GPIO channel usage
GPIO.setwarnings(False)
# Set the pin numbering mode to BOARD (refers to the physical pin numbers on the Raspberry Pi)
GPIO.setmode(GPIO.BOARD)
# Set pin 31 as an output pin (can be used to control an LED, relay, etc.)
GPIO.setup(31, GPIO.OUT)
# Infinite loop to continuously blink the LED or control the connected output device
while True:
    # Turn on the output on pin 31 (e.g., LED will light up)
    GPIO.output(31, True)
    # Wait for 1 second
    sleep(1)
    # Turn off the output on pin 31 (e.g., LED will turn off)
    GPIO.output(31, False)
    # Wait for 1 second
    sleep(1)


Code:7b: Interfacing DHT11 with Raspberry Pi GPIO pins.


import Adafruit_DHT
import time
# Define the sensor and the GPIO pin
DHT_SENSOR = Adafruit_DHT.DHT11
DHT_PIN = 6  # GPIO pin where the DHT11 is connected

while True:
    # Read the humidity and temperature
    humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
    # Check if the reading was successful
    if humidity is not None and temperature is not None:
        print(f'Temperature={temperature:.1f}°C  Humidity={humidity:.1f}%')
    else:
        print('Failed to retrieve data from humidity sensor')
    # Wait for a second before the next read
    time.sleep(1)




CODE8:
import time
import Adafruit_DHT
import urllib.request
pin = 5
print("Name : Preshit Motghare")
print("Roll No : 313037")
print(“PRN : 22210307”)
while True:
humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, pin)
if humidity is not None and temperature is not None:
print('Temp={0:0.1f}*C Humidity={1:0.1f}%'.format(temperature, humidity))
else:
print('Failed to get reading. Try again!')
time.sleep(2)
f =
urllib.request.urlopen('https://api.thingspeak.com/update?api_key=O2LRG4WZQAXM5VHC={0}&fiel
d2={1}'.format(temperature, humidity))
print(f.read())
f.close()

9th
Index code9:
<!DOCTYPE html>
<html>
<head>
<title>GPIO Control Web App</title>
<link rel="stylesheet" href="style.css">
</head>
<body>
<img class="img" src= "/logo.png" width="300px" height="200px" />
<h1>Smart Home Automation Using Raspberry Pi</h1>
<h2>Name: Preshit Motghare </h2>
<h2>Roll no: 313037</h2>
<h2>PRN: 22210307</h2>
<h2>Batch: C2</h2>
<h4>date of performance: 22/10/2024</h4>
<h2>Status of Actuators</h2>
<h3>Red Led --> {{ledRed}}</h3>
<h3>Yellow Led --> {{ledYellow}}</h3>
<h3>Green Led --> {{ledGreen}}</h3>
<h2>LED Control</h2>
<h3>Red Led Control ==> <a href="/ledRed/on" class="button">Turn on</a><a href="/ledRed/off"
class="button">Turn off</a></h3>
<h3>Yellow Led Control ==> <a href="/ledYellow/on" class="button">Turn on</a><a
href="/ledYellow/off" class="button">Turn off</a></h3>
<h3>Green Led Control ==> <a href="/ledGreen/on" class="button">Turn on</a><a
href="/ledGreen/off" class="button">Turn off</a></h3>
</body>
</html>
CSS code :
body{
text-align: center;
background-color: #44b09e;
background-image: linear-gradient(315deg, #44b09e 0%, #e0d2c7 74%);
color: #000000;
}
.button{
font: bold 16px Arial;
background-color: #EEEEEE;
padding:1px;
border: 1px solid #CCCCCC;
}



App code:
import RPi.GPIO as GPIO
from flask import Flask, render_template
app = Flask(__name__)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
ledRed = 13
ledYellow = 19
ledGreen = 26
ledRedSts=0
ledYellowSts=0
ledGreenSts=0
GPIO.setup(ledRed, GPIO.OUT)
GPIO.setup(ledYellow, GPIO.OUT)
GPIO.setup(ledGreen, GPIO.OUT)
GPIO.output(ledRed, GPIO.LOW)
GPIO.output(ledYellow, GPIO.LOW)
GPIO.output(ledGreen, GPIO.LOW)
@app.route('/')
def index():
ledRedSts=GPIO.input(ledRed)
ledYellowSts=GPIO.input(ledYellow)
ledGreenSts=GPIO.input(ledGreen)
templateData={'ledRed':ledRedSts,
'ledYellow': ledYellowSts,
'ledGreen': ledGreenSts}
return render_template('index.html', **templateData)
@app.route('/<deviceName>/<action>')
def do(deviceName, action):
if deviceName=="ledRed":
actuator=ledRed
if deviceName=="ledYellow":
actuator=ledYellow
if deviceName=="ledGreen":
actuator=ledGreen
if action=="on":
GPIO.output(actuator, GPIO.HIGH)
if action=="off":
GPIO.output(actuator, GPIO.LOW)
ledRedSts = GPIO.input(ledRed)
ledYellowSts = GPIO.input(ledYellow)
ledGreenSts = GPIO.input(ledGreen)
templateData = {'ledRed': ledRedSts,
'ledYellow': ledYellowSts,
'ledGreen': ledGreenSts}
return render_template('index.html', **templateData)
if __name__ == '__main__':
app.run(debug=True, host = '0.0.0.0')
