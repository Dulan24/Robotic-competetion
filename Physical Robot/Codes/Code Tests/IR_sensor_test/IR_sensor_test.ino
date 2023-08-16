// Define the pins used for the IR sensor
const int IR_SENSOR_ANALOG_PIN = A0;
const int IR_SENSOR_DIGITAL_PIN = 2;

void setup() {
  // Initialize the serial communication for debugging
  Serial.begin(9600);

  // Initialize the IR sensor analog pin as an input
  pinMode(IR_SENSOR_ANALOG_PIN, INPUT);

  // Initialize the IR sensor digital pin as an input
  pinMode(IR_SENSOR_DIGITAL_PIN, INPUT);
}

void loop() {
  // Read the analog voltage of the IR sensor
  int sensorAnalogValue = analogRead(IR_SENSOR_ANALOG_PIN);

  // Read the digital state of the IR sensor
  int sensorDigitalValue = digitalRead(IR_SENSOR_DIGITAL_PIN);

  // Print the sensor values to the serial monitor
  Serial.print("Analog Value: ");
  Serial.print(sensorAnalogValue);
  Serial.print(", Digital Value: ");
  Serial.println(sensorDigitalValue);

  // Wait for a short period before taking the next reading
  delay(100);
}
