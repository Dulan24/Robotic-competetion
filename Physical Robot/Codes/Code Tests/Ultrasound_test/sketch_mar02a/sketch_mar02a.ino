// Define the pins used for the sensor
const int trigPin = 12;
const int echoPin = 13;

// Define variables for the distance and duration
float distance;
long duration;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  
  // Set the trigPin as an output and echoPin as an input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Give the sensor some time to settle
  delay(1000);
}

void loop() {
  // Send a pulse to the sensor to trigger the measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance based on the speed of sound
  distance = duration * 0.034 / 2;
  
  // Print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Wait for a short period before taking another measurement
  delay(500);
}
