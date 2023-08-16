
#define R_motor_A 3
#define R_motor_B 4
#define L_motor_A 5
#define L_motor_B 6
#define threshold 40

#define R_motor_pwm 9
#define L_motor_pwm 10

int IR_val[6] = {0,0,0,0,0,0} ;
int IR_weight[6] = {-20,-10,-5,5,10,20};

int max_speed = 150 ;
int motor_mid_speed = 100 ;
int left_motor_speed = motor_mid_speed ;
int right_motor_speed = motor_mid_speed ;

float p , d , error ;
float i = 0 ;
float prev_error = 0 ;
float KP = 5.5 ;
float KI = 0 ;
float KD = 10 ;

  
void setup() {
  Serial.begin(9600) ;
  pinMode(A0, INPUT) ;
  pinMode(A1, INPUT) ;
  pinMode(A2, INPUT) ;
  pinMode(A3, INPUT) ;
  pinMode(A4, INPUT) ;
  pinMode(A5, INPUT) ;
  Serial.begin(9600);

  pinMode(3, OUTPUT) ;
  pinMode(4, OUTPUT) ;
  pinMode(5, OUTPUT) ;
  pinMode(6, OUTPUT) ;
  pinMode(9, OUTPUT) ;
  pinMode(10, OUTPUT) ;

  set_motor_speed()  ;

  delay(500) ;
  
}

void loop() {
  IR_val[0] = analogRead(A0) ;
  if (IR_val[0] > threshold ) {IR_val[0] = 1; }
  else {IR_val[0] = 0 ;}
 // Serial.print(analogRead(A0)) ;
  Serial.print(IR_val[0]) ;
  Serial.print(' ') ;
  
  IR_val[1] = analogRead(A1) ;
  if (IR_val[1] > threshold ) {IR_val[1] = 1; }
  else {IR_val[1] = 0 ;}
  //Serial.print(analogRead(A1)) ;
  Serial.print(IR_val[1]) ;
  Serial.print(' ') ;
  
  IR_val[2] = analogRead(A2) ;
  if (IR_val[2] > threshold ) {IR_val[2] = 1; }
  else {IR_val[2] = 0 ;}
  //Serial.print(analogRead(A2)) ;
  Serial.print(IR_val[2]) ;
  Serial.print(' ') ;
  
  IR_val[3] = analogRead(A3) ;
  if (IR_val[3] > threshold ) {IR_val[3] = 1; }
  else {IR_val[3] = 0 ;}
  //Serial.print(analogRead(A3)) ;
  Serial.print(IR_val[3]) ;
  Serial.print(' ') ;
  
  IR_val[4] = analogRead(A4) ;
  if (IR_val[4] > threshold ) {IR_val[4] = 1; }
  else {IR_val[4] = 0 ;}
  //Serial.print(analogRead(A4)) ;
  Serial.print(IR_val[4]) ;
  Serial.print(' ') ;
  
  IR_val[5] = analogRead(A5) ;
  if (IR_val[5] > threshold ) {IR_val[5] = 1; }
  else {IR_val[5] = 0 ;}
  //Serial.println(analogRead(A5)) ;
  Serial.println(IR_val[5]) ;
  Serial.print(' ') ;

  //delay(1000);

 if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 ){
      stop() ;
      while(true){}
      }
      
  if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 ){
      stop() ;
      while(true){}
      }

  PID() ;
  set_motor_speed() ;


  
}

void forward() {
  digitalWrite(R_motor_A, HIGH) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_A, HIGH) ;
  digitalWrite(L_motor_B, LOW) ;
}

void stop () {
  digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_A, LOW) ;
  digitalWrite(L_motor_B, LOW) ;
}

void set_motor_speed() {
  analogWrite(R_motor_pwm, right_motor_speed) ;
  analogWrite(L_motor_pwm, left_motor_speed) ;
   
  }
  
void PID() {
  error = 0 ;

  for (int i=0; i<=5; i++) {
    error += IR_val[i]*IR_weight[i] ;
    }

  p = error ;
  i = i + error ;
  d = error - prev_error ;
  prev_error = error ;

  float PID_val = (p*KP + i*KI + d*KD) ;
  int PID_val_int = round(PID_val);

  forward();
  right_motor_speed = motor_mid_speed + PID_val ;
  left_motor_speed = motor_mid_speed - PID_val ;

  if (right_motor_speed < 0)         {right_motor_speed = 0 ; }
  if (right_motor_speed > max_speed) {right_motor_speed = max_speed ; }
  if (left_motor_speed < 0)          {left_motor_speed = 0 ; }
  if (left_motor_speed > max_speed)  {left_motor_speed = max_speed ; }
 
  }
