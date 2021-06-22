#define enA 6              //
#define enB 5
#define dirA1 4
#define dirA2 7
#define dirB1 2
#define dirB2 3
#define wheel_base 0.120
#define TRIG_PIN 13 
#define ECHO_PIN 12
#define MAX_DISTANCE 200 
#define DHTPIN A1     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

#include <NewPing.h>
#include <Servo.h> 
//#include <ros.h>
//#include <geometry_msgs/Twist.h>
#include <DHT.h>;

//ros::NodeHandle nh;

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 

Servo myservo;

DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino  

int speedo, vel;
int distance = 100;
int IRSensor1 = 10; // connect ir sensor to arduino pin 10
int IRSensor2 = 9; // connect ir sensor to arduino pin 9
int IRSensor3 = 8; // connect ir sensor to arduino pin 8
int IRSensor4 = A0; // connect ir sensor to arduino pin A0
float hum;  //Stores humidity value
float temp; //Stores temperature value

/*
void driverCallback(const geometry_msgs::Twist& vel){

    float linear = vel.linear.x;
    float angular = vel.angular.z; 

    float left_speed = linear - angular*wheel_base/2;
    float right_speed = linear + angular*wheel_base/2;

    int left_en = (255 * left_speed/0.5);
    int right_en = (255 * right_speed/0.5);
    
    motorMove(left_en, right_en);
    
  }
  */

//ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &driverCallback);

void setup() {
  Serial.begin (9600);
  myservo.attach(11);  
  myservo.write(115); 
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);

  pinMode (IRSensor1, INPUT); // sensor pin INPUT
  pinMode (IRSensor2, INPUT); // sensor pin INPUT
  pinMode (IRSensor3, INPUT); // sensor pin INPUT
  pinMode (IRSensor4, INPUT); // sensor pin INPUT

  dht.begin();
  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(dirA1, OUTPUT);
  pinMode(dirA2, OUTPUT);
  pinMode(dirB1, OUTPUT);
  pinMode(dirB2, OUTPUT);

  //nh.initNode();
  //nh.subscribe(sub);

}

void loop() {
  motorMove(0);
  //detectobstacle();
  //detectwhiteline();
  //temperaturevalue();
 // nh.spinOnce();
  delay(1);
}

void detectobstacle()
{
 int distanceR = 0;
 int distanceL =  0;
 delay(40);
 
 if(distance<=15)
 {
  motorStop();
  distanceR = lookRight();
  delay(200);
  distanceL = lookLeft();
  delay(200);

  if(distanceR>=distanceL)
  {
   motorMove(1);
  }else
  {
   motorMove(-1);
  }
 }else
 {
  motorMove(0);
 }
 distance = readPing();
}

int lookRight()
{
    myservo.write(50); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
}

int lookLeft()
{
    myservo.write(170); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
    delay(100);
}

int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  return cm;
}

void detectwhiteline()
{
  int statusSensor1 = digitalRead(IRSensor1);
  Serial.print("Sensor1= ");
  Serial.println(statusSensor1);
  int statusSensor2 = digitalRead(IRSensor2);
  Serial.print("Sensor2= ");
  Serial.println(statusSensor2);
  int statusSensor3 = digitalRead(IRSensor3);
  Serial.print("Sensor3= ");
  Serial.println(statusSensor3);
  int statusSensor4 = digitalRead(IRSensor4);
  Serial.print("Sensor4= "); 
  Serial.println(statusSensor4);
  delay(1000);

  if (statusSensor1 == 0)
  {
    motorMove(1);
  }
  else if (statusSensor2 == 0)
  {
    motorMove(-1);
  }
  else if (statusSensor3 == 0)
  {
    motorMove(1);
  }
  else if (statusSensor4 == 0)
  {
    motorMove(-1);
  }
  else
  {
    Serial.println("In the map");
  }
}

void temperaturevalue(){
    delay(2000);
    //Read data and store it to variables hum and temp
    hum = dht.readHumidity();
    temp= dht.readTemperature();
    //Print temp and humidity values to serial monitor
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.print(" %, Temp: ");
    Serial.print(temp);
    Serial.println(" Celsius");
    //delay(10000); //Delay 2 sec.
}

/*

void motorMove(int left, int right){

    if (right >= 0){
       digitalWrite(dirA1, HIGH);
       digitalWrite(dirA2, LOW); 
    }
    else{
       digitalWrite(dirA1, LOW);
       digitalWrite(dirA2, HIGH);
    }
  
    if (left >= 0){
       digitalWrite(dirB1, HIGH);
       digitalWrite(dirB2, LOW); 
    }
    else{
       digitalWrite(dirB1, LOW);
       digitalWrite(dirB2, HIGH);
    }
    analogWrite(enA, abs(right));
    analogWrite(enB, abs(left));
  }
  */

void motorMove(int dir){
    speedo = 255;

    //vel = speedo * dir;

    if (dir >= 1){
      digitalWrite(dirA1, HIGH);
      digitalWrite(dirA2, LOW);
      analogWrite(enA, speedo);
      digitalWrite(dirB1, LOW);
      digitalWrite(dirB2, HIGH);
      analogWrite(enB, speedo);
    }
    else if (dir <= -1){
      digitalWrite(dirA1, LOW);
      digitalWrite(dirA2, HIGH);
      analogWrite(enA, speedo);
      digitalWrite(dirB1, HIGH);
      digitalWrite(dirB2, LOW);
      analogWrite(enB, speedo);
    }
    else {
      digitalWrite(dirA1, HIGH);
      digitalWrite(dirA2, LOW);
      analogWrite(enA, speedo);
      digitalWrite(dirB1, HIGH);
      digitalWrite(dirB2, LOW);
      analogWrite(enB, speedo);
    }
  }

void motorStop(){
    digitalWrite(dirA1, LOW);
    digitalWrite(dirA2, LOW);
    analogWrite(enA, 0);
    digitalWrite(dirB1, LOW);
    digitalWrite(dirB2, LOW);
    analogWrite(enB, 0);
  }
  
  
