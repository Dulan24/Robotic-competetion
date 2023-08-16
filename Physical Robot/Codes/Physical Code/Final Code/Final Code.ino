#define MAX_SPEED 170
#define MIN_SPEED 0
#define BASE_SPEED 60
#define VALID_DIS 3.0 // 20cm
#define RANGE 0.5
#define MAX_SENSOR_VALUE 10000
#define MAX_OF_SENSOR 3000
#define DIST_PROPORTIONAL_CONST 0.034/2 //DEPENDS ON ENVIRONMENT AND SPEED OF SOUND
#define MAX_ALLIGN_ANGLE 0.25
#define CALIBRATION 42
#define FOLLOW_WALL_MAX 150
#define FOLLOW_BASE_SPEED 90
#define FOLLOW_WALL_MIN 0
#define MAX_DIFF 80
#define MAX_DIFF_BACK 50
#define SPEEDL 0
#define SPEEDR 200
#define COL_SPEED 220
#define ADD 30


//sensor 1 pins
const int trigPin1 = 10;
const int echoPin1 = 11;

//sensor 2 pins
const int trigPin2 = 12;
const int echoPin2 = 13;

//sensor 3 pins : front sensor
const int trigPin3 = 6;
const int echoPin3 = 2;

//left motor pins
const int in1L = 7;
const int in2L = 8;
const int enL = 9;

//right motor pins
const int in1R = 4;
const int in2R = 5;
const int enR = 3;

//motor speeds
int speedL = BASE_SPEED ;
int speedR = BASE_SPEED ;

//tuning constants
// float kp = 4.1;
float kp = 4.1;
float ki = 0 ;
float kd = 1 ;

float kp_a = 4.5 ;
float kd_a = 0.0 ;

float total_error = 0.0 ;
float prev_error = 0.0 ;

float error,difference,pid,distance1,distance2,dis,previousAngle = 0.0 ;
int pid_new, region ;
 
void setup(){
  pinMode(in1L,OUTPUT);
  pinMode(in2L,OUTPUT);
  pinMode(in1R,OUTPUT);
  pinMode(in2R,OUTPUT);
  pinMode(enL,OUTPUT);
  pinMode(enR,OUTPUT);                          
  
  pinMode(trigPin1,OUTPUT);
  pinMode(echoPin1,INPUT);
  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);

  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);

  Serial.begin(9600);
  }

void loop() {

////////////////////////////////////////////////////////////
      //delay(1000);
      Serial.println("//////////////////////////////////////////////");
   distance1 = sensor_output(10,11);
   distance2 = sensor_output(12,13);
   float dis = current_distance(distance1, distance2);

   //if (distance1 > (VALID_DIS + RANGE) && distance1 > (VALID_DIS + RANGE)  ) {kp_a = 0.1*kp_a ;}

      Serial.print("distance1 = ") ;
      Serial.println(distance1) ;
      Serial.print("distance2 = ") ;
      Serial.println(distance2) ;

////////////////////////////////////////////////////////////
    if (distance1 - distance2 > MAX_DIFF ) {
      stopM();
      stopM();
      stopM();
      speedL = SPEEDL ; 
      speedR = SPEEDR ;

      
      Serial.print("SPEEDL = ") ;
      Serial.println(SPEEDL) ;
      Serial.print("SPEEDR = ") ;
      Serial.println(SPEEDR) ;
      
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, HIGH);
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, HIGH); 
}




if (distance2 - distance1 > MAX_DIFF )  {
  stopM();
  speedL = BASE_SPEED + ADD;
  speedR = BASE_SPEED + ADD ;
  setspeed();
  forward() ;
  delay(500);
  stopM();
  }


    
    
////////////////////////////////////////////////////////////
    if (distance1 < 2 && distance2 > 3) {
     stopM();
     stopM();
     stopM();
     speedL = COL_SPEED ;
     speedR = SPEEDL ;
     forward();
     stopM(); 
     stopM();
     stopM(); 
     
      }


//////////////////PID calculation///////////////////////////////
    int pid_val = PID(dis) ;
    
  Serial.print("pid_val = ") ;
  Serial.println(pid_val) ;

   if ((VALID_DIS - RANGE) <= dis <=  (VALID_DIS + RANGE) ) {region = 0 ;}
   if (dis < (VALID_DIS - RANGE)) {region = -1 ;} // turn right
   if ((VALID_DIS + RANGE) < dis) {region =  1 ;} //  turn left

  Serial.print("region = ") ;
  Serial.println(region) ;
/////////////////////////////////////////////////////////////////


////////////////Check region and set motor speed///////////////
  if (region == 0) { 

     speedL = BASE_SPEED ;
     speedR = BASE_SPEED ;

     Serial.print("speedL = ") ;
     Serial.println(speedL) ;
     Serial.print("speedR = ") ;
     Serial.println(speedR) ;
     
     setspeed();
     forward() ;
     follow_wall();
    }

  else {
    speedL = BASE_SPEED - pid_val ;
       if (speedL < 0) {speedL = MIN_SPEED ;}
       if (speedL >  MAX_SPEED) {speedL = MAX_SPEED ;}

    speedR = BASE_SPEED + pid_val ;
       if (speedR < 0) {speedR = MIN_SPEED ;}
       if (speedR >  MAX_SPEED) {speedR = MAX_SPEED ;}

     Serial.print("speedL = ") ;
     Serial.println(speedL) ;
     Serial.print("speedR = ") ;
     Serial.println(speedR) ;

    setspeed();
    forward() ;
    follow_wall();
    
    }
  
  }

void setspeed() {
  analogWrite(enL, speedL);
  analogWrite(enR, speedR);
}

void forward() {
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  }

void stopM() {
  digitalWrite(in1L, LOW);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, LOW);
  digitalWrite(in2R, LOW);
  }

float sensor_output(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  
 if(duration==0.0||duration>=MAX_SENSOR_VALUE){
    duration = MAX_OF_SENSOR;
    }
    

  float distance = duration*DIST_PROPORTIONAL_CONST;
   
  return distance; 
}

float current_distance(float read1, float read2){
  float distance = (read1 + read2)/2;  //taking average of two values

  return distance;
  }

int PID(float dis)  {
  error = dis - VALID_DIS ;
  total_error += error ;
  difference = error - prev_error ;

  pid = (kp*error) + (ki*total_error) + (kd*difference) ;
  prev_error = error ;
  pid_new = int(pid) ;

  return pid_new ;
  
  }

int follow_wall() {
  distance1 = sensor_output(trigPin1, echoPin1);
  distance2 = sensor_output(trigPin2, echoPin2);
  float angle = distance2-distance1;//if +ve turn left else turn right 

if (abs(angle) > MAX_ALLIGN_ANGLE ) {

//  Serial.print("angle = ") ;
//  Serial.println( angle) ;
  
  float derivativeA = angle - previousAngle;
  float outputA = kp_a*angle + kd_a*derivativeA;
  int output_new = (int)outputA ;

//  Serial.print("output_new = ") ;
//  Serial.println( output_new) ;

  previousAngle = angle;

      if (distance2 > distance1) {
        speedL = FOLLOW_BASE_SPEED + abs(output_new)+ CALIBRATION;
        speedR = FOLLOW_BASE_SPEED - abs(output_new) - CALIBRATION;

           if (speedL < 0) {speedL = FOLLOW_WALL_MIN ;}
           if (speedL >  MAX_SPEED) {speedL = FOLLOW_WALL_MAX ;}
           
               Serial.print("follow_wall_speedL = ") ;
               Serial.println( speedL) ;

            if (speedR < 0) {speedR = FOLLOW_WALL_MIN ;}
            if (speedR >  MAX_SPEED) {speedR = FOLLOW_WALL_MAX ;}
            
               Serial.print("follow_wall_speedR = ") ;
               Serial.println( speedR) ;

            setspeed();
            forward(); 
            }

      if (distance2 < distance1) {
        speedL = FOLLOW_BASE_SPEED  - abs(output_new) - CALIBRATION;
        speedR = FOLLOW_BASE_SPEED + abs(output_new) + CALIBRATION ;
        
           if (speedL < 0) {speedL = FOLLOW_WALL_MIN;}
           if (speedL >  MAX_SPEED) {speedL = FOLLOW_WALL_MAX ;}
           
                Serial.print("follow_wall_speedL = ") ;
                Serial.println( speedL) ;

            if (speedR < 0) {speedR = FOLLOW_WALL_MIN ;}
            if (speedR >  MAX_SPEED) {speedR = FOLLOW_WALL_MAX ;}
            
                Serial.print("follow_wall_speedR = ") ;
                Serial.println( speedR) ;

            setspeed();
            forward(); 
            }
            
}
  }

  

 
