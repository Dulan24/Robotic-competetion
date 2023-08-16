int sensor_pins[6] = {A0, A1, A2, A3, A4, A5};
int sensor_values[6];
int motor_speed = 100;
int error = 0;
int last_error = 0;
int error_sum = 0;
int error_diff = 0;
int kp = 2;
int ki = 0.1;
int kd = 0.5;

void setup() {
  // Initialize the serial communication for debugging
  Serial.begin(9600);

  // Initialize the motor pins as outputs
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  // Initialize the sensor pins as inputs
  for (int i = 0; i < 6; i++) {
    pinMode(sensor_pins[i], INPUT);
  }
}

void loop() {
  // Read the sensor values and print them to the serial monitor
  for (int i = 0; i < 6; i++) {
    sensor_values[i] = analogRead(sensor_pins[i]);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensor_values[i]);
  }

  // Calculate the error based on the sensor values
  error = sensor_values[0] - sensor_values[5];

  // Calculate the sum and difference of errors
  error_sum += error;
  error_diff = error - last_error;
  last_error = error;

  // Calculate the PID output
  int pid_output = kp * error + ki * error_sum + kd * error_diff;

  // Adjust the motor speeds based on the PID output
  int left_motor_speed = motor_speed + pid_output;
  int right_motor_speed = motor_speed - pid_output;

  // Set the motor speeds
  analogWrite(3, left_motor_speed);
  analogWrite(5, 0);
  analogWrite(6, left_motor_speed);333
  analogWrite(9, right_motor_speed);
  analogWrite(10, 0);
  analogWrite(11, right_motor_speed);
}
