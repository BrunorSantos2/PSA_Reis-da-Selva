// Slave: Serial Communication between two Arduino Boardds
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h> //including library for I2C communication

char number  = ' ';     //Comunicação entre dois Arduinos
boolean automatico;

//Definir entrada motores
    const int enL = 5;   //enA - Motor Esquerda
    const int inL1 = 7;  //
    const int inL2 = 6;  //
    
    const int enR = 3;    //enB - Motor Direita
    const int inR3 = 2;  //
    const int inR4 = 4;  //

//Definir entrada sensor ultrasónico frente 
    #define echoPin 11
    #define trigPin 10
    #define max_distance 50
  
    NewPing sonar(trigPin, echoPin, max_distance);

//Definir servo
    Servo servo;



//definir sensor IR
    int left_sensor_pin = A0;
    int right_sensor_pin = A3;

    int left_sensor_state;
    int right_sensor_state;

    int left_sensor_turn;
    int right_sensor_turn;
//
    int distance = 0;
    int leftDistance;
    int rightDistance;
    int distanceTurn;
    
    boolean object;
    boolean detetar;
    boolean linha;

   
  //Definir Bussola
#define DECLINATION 1.43                    // declination (in degrees)
unsigned char CMPS2_address = 0x30;         //I2C address of the device
float Max[2], Mid[2], Min[2], X, Y;         //store highest, middle and lowest values, x component and y component
    
Definir Encoders
#define encoder1A  0       //signal A of left encoder  (white wire)
#define encoder1B  1       //signal B of left encoder  (yellow wire)
#define encoder2A  2       //signal A of right encoder  (white wire)
#define encoder2B  3       //signal B of right encoder  (yellow wire)

volatile int encoderLeftPosition = 0;     // counts of left encoder 
volatile int encoderRightPosition = 0;    // counts of right encoder 
float  DIAMETER  = 65  ;                  // wheel diameter (in mm)  
float distanceLeftWheel, distanceRightWheel, Dc, Orientation_change;
float ENCODER_RESOLUTION = 333.3;         //encoder resolution (in pulses per revolution)  where in Rover 5,  1000 state changes per 3 wheel rotations 
int x = 0;                                // x initial coordinate of mobile robot 
int y = 0;                                // y initial coordinate of mobile robot 
float Orientation  = 0;                   // The initial orientation of mobile robot 
float WHEELBASE=130  ;                    //  the wheelbase of the mobile robot in mm
float CIRCUMSTANCE =PI * DIAMETER  ;



void setup() {
  
   Serial.begin(9600);
   Serial.println("START");

  //outputs motores
    pinMode(enR, OUTPUT);
    pinMode(inR3, OUTPUT);
    pinMode(inR4, OUTPUT);
    pinMode(enL, OUTPUT);
    pinMode(inL1, OUTPUT);   
    pinMode(inL2, OUTPUT);

  //sensor ultrasonic
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

  //Output Bussola
  pinMode(13, OUTPUT);
  delay(10);
  CMPS2_init(); //initialize the compass
    
    
    
    //sensore IR
    pinMode(left_sensor_pin, INPUT);
    pinMode(right_sensor_pin, INPUT);
  
  //servo
    servo.attach(8);
    servo.write(90);

   detetar = true;
   automatico = false;
}

void loop() {
String readString;
String Q;
  
    //retrieving and displaying the heading of the compass
  float measured_angle = CMPS2_getHeading();
  Serial.print("Heading = ");
  Serial.print(measured_angle);
  Serial.print("°");
  Serial.print('\t');

  CMPS2_decodeHeading(measured_angle);  //get direction

    
    while (Serial.available()) {
      delay(1);
    if (Serial.available() >0) {    
      char c = Serial.read();  //gets one byte from serial buffer
    if (isControl(c)) {
      //'Serial.println("it's a control character");
      break;
    }
      readString += c; //makes the string readString    
      Q=readString;
     if(Q == "9"){
      automatico = true;
      Serial.println("auto1");
      }
     else if (Q == "8"){
      Serial.println("manual1");
      automatico = false;
      }
    }
  }   
    if (automatico == true){ ///
      Serial.println("auto2");
      infra();
      }
    else if(automatico == false){
      manual();
      }

}


void manual() {
  
String readString;
String Q;
 
while (Serial.available()) {
      delay(1);
    if (Serial.available() >0) {
      char c = Serial.read();  //gets one byte from serial buffer
    if (isControl(c)) {
      //'Serial.println("it's a control character");
      break;
    }
      readString += c; //makes the string readString    
    }
 }   
 
Q = readString;

   if (Q=="1") { 
      Serial.println("Front");
      digitalWrite(inR3, HIGH);   //Andar frente
      digitalWrite(inR4, LOW);
      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
      digitalWrite(enR, 255);
      digitalWrite(enL , 255);  
      }

    if (Q=="2") { 
      Serial.println("Back");
      digitalWrite(inR3, LOW);   //Andar trás
      digitalWrite(inR4, HIGH);
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, HIGH);
      digitalWrite(enR, 255);
      digitalWrite(enL , 255);  
      }

    if (Q=="3") { 
      Serial.println("Left");
      digitalWrite(inR3, HIGH);   //Andar Esquerdo
      digitalWrite(inR4, LOW);
      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
      digitalWrite(enR, 255);
      digitalWrite(enL , 150);  
      }

    if (Q=="4") { 
      Serial.println("Left 360");
      digitalWrite(inR3, HIGH);   //Andar 360 Esquerdo
      digitalWrite(inR4, LOW);
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, HIGH);
      digitalWrite(enR, 255);
      digitalWrite(enL , 255);  
      }

    if (Q=="5") { 
      Serial.println("Right");
      digitalWrite(inR3, HIGH);   //Andar Direito
      digitalWrite(inR4, LOW);
      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
      digitalWrite(enR, 155);
      digitalWrite(enL , 255);  
      }

    if (Q=="6") { 
      Serial.println("Right 360");
      digitalWrite(inR3, LOW);   //Andar 360 Direito
      digitalWrite(inR4, HIGH);
      digitalWrite(inL1, HIGH);
      digitalWrite(inL2, LOW);
      digitalWrite(enR, 255);
      digitalWrite(enL , 255);  
      }

    if (Q=="7") { 
      Serial.println("Stop");
      digitalWrite(inR3, LOW);   //Parar
      digitalWrite(inR4, LOW);
      digitalWrite(inL1, LOW);
      digitalWrite(inL2, LOW);
      digitalWrite(enR, 0);
      digitalWrite(enL , 0);  
      }
      
}  


////////////////////////////////////////////////////////////////////////////////////////////////////

void Stop() {
  digitalWrite(inR3, LOW);   //Parar
  digitalWrite(inR4, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  analogWrite(enR, 0);
  analogWrite(enL , 0);

}

void moveForward() {
  digitalWrite(inR3, HIGH);   //Andar frente
  digitalWrite(inR4, LOW);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  analogWrite(enR, 100);
  analogWrite(enL , 100);
}

void moveBackward() {
  digitalWrite(inR3, LOW);   //Andar trás
  digitalWrite(inR4, HIGH);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  analogWrite(enR, 100);
  analogWrite(enL , 100);
}
void slow (){                //Andar devagar
 moveForward();
 delay(50);
 Stop();
 delay(100); 
 
 slow();
  }

  
void moveRight() {
  digitalWrite(inR3, LOW);   //Andar Direito
  digitalWrite(inR4, HIGH);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  analogWrite(enR, 120);
  analogWrite(enL , 120); 
}


void moveLeft() {
  digitalWrite(inR3, HIGH);   //Andar Esquerdo
  digitalWrite(inR4, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  analogWrite(enR, 120);
  analogWrite(enL , 120);  
}


void Right360() {
  digitalWrite(inR3, LOW);   //Andar Direito 360
  digitalWrite(inR4, HIGH);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  analogWrite(enR, 150);
  analogWrite(enL , 150); 
}


void Left360() {
  digitalWrite(inR3, HIGH);   //Andar Esquerdo 360
  digitalWrite(inR4, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  analogWrite(enR, 150);
  analogWrite(enL , 150);  
}

///////////////////////////////////////////////////////////////////////////////////////////

void infra() {               //Seguir infravermelhos

  left_sensor_state = analogRead(left_sensor_pin);
  right_sensor_state = analogRead(right_sensor_pin);
  
   if(right_sensor_state < 200 && left_sensor_state < 200) {       //Direita branca && Esquerda branco
    objectAvoid();
    //forward
    
    
  }else if(right_sensor_state < 200 && left_sensor_state > 200) {  //Direita branca && Esquerda preto
    objectAvoid();
    //turn right
    Right360();
    //moveRight();
    
  }else if(right_sensor_state > 200 && left_sensor_state < 200) {  //Direita preto && Esquerda branco
    objectAvoid();
    //turn left
    Left360();
    //moveLeft();
    
  }else if(right_sensor_state > 200 && left_sensor_state > 200) {  //Direita preto && Esquerda preto
    objectAvoid();
    //stop
    Right360();
    delay(100);
    moveBackward();
    delay(100);
    Stop();
    
    
  }
}


void objectAvoid() {                          //Desviar do objeto
  distance = getDistance();
  if (distance <= 15 && detetar == true) {
    //stop
    Stop();
    Serial.println("Stop");
    lookLeft();
    lookRight();
    delay(100);
    if(rightDistance < leftDistance) {
      //left
      object = true;
      turn();
      Serial.println("moveLeft");
    } else {
      //right
      object = false;
      turn();
      Serial.println("moveRight");
    }
    delay(100); 
  }
  else {
    //forward
    //Serial.println("going forward"); 
    moveForward();
  }
}


int getDistance() {                    //Definir distancia
  
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
  
}


int lookLeft () {                      //Olhar esquerda
  //look left
  servo.write(150);
  delay(500);
  leftDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("Left:");
  Serial.print(leftDistance);
  return leftDistance;
  delay(100);
}

int lookRight() {                      //Olhar Direita
  //lock right
  servo.write(30);
  delay(500);
  rightDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("Right:");
  Serial.println(rightDistance);
  return rightDistance;
  delay(100);
}



void turn() {                          //Contornar obstáculo
  
  if (object == false) {
    Serial.println("turn Right");   //Virar Direita - Mais distância
   
    moveBackward();
    delay(400);
    Stop();
    delay(400);
    
    Right360();
    delay(600);
    Stop();
    delay(400);
    
    moveForward();
    delay(800);
    Stop();
    delay(400);
    
    Left360();
    delay(400);
    Stop();
    servo.write(180);
    delay(400);
    moveForward();
    delay(100);

    distanceTurn = getDistance();
    while (distanceTurn <= 25) {
        moveForward();
        delay(100);
        Stop();
        delay(100);
        
        distanceTurn = getDistance();
        Serial.println(distanceTurn);
    }
    moveForward();
    delay(400);
    servo.write(90);
    Stop();
    delay(500);    
    Left360();
    delay(300);
    left_sensor_turn = analogRead(left_sensor_pin);
    right_sensor_turn = analogRead(right_sensor_pin);
    while (right_sensor_turn > 200 || left_sensor_turn > 200) {
        moveForward();
        delay(100);
        Stop();
        delay(100);        
    left_sensor_turn = analogRead(left_sensor_pin);
    right_sensor_turn = analogRead(right_sensor_pin);
    }
    Stop();
    delay(500);
    detetar = false;
    linha = true;
    infra();
    
  }

  else if {object == true){
    Serial.println("turn Left");   //Virar Esquerda - Mais distância
   
    moveBackward();
    delay(400);
    Stop();
    delay(400);
    
    Left360();
    delay(600);
    Stop();
    delay(400);
    
    moveForward();
    delay(800);
    Stop();
    delay(400);
    
    Right360();
    delay(400);
    Stop();
    servo.write(0);
    delay(400);
    moveForward();
    delay(100);

    distanceTurn = getDistance();
    while (distanceTurn <= 25) {
        moveForward();
        delay(100);
        Stop();
        delay(100);
        
        distanceTurn = getDistance();
        Serial.println(distanceTurn);
    }
    moveForward();
    delay(400);
    servo.write(90);
    Stop();
    delay(500);    
    Righ360();
    delay(300);
    left_sensor_turn = analogRead(left_sensor_pin);
    right_sensor_turn = analogRead(right_sensor_pin);
    while (right_sensor_turn > 200 || left_sensor_turn > 200) {
        moveForward();
        delay(100);
        Stop();
        delay(100);        
    left_sensor_turn = analogRead(left_sensor_pin);
    right_sensor_turn = analogRead(right_sensor_pin);
    }
    Stop();
    delay(500);
    detetar = false;
    linha = true;
    infra();
    
  }


void encoder() {
  distanceLeftWheel = CIRCUMSTANCE * (encoderLeftPosition / ENCODER_RESOLUTION);       //  travel distance for the left and right wheel respectively 
  distanceRightWheel = CIRCUMSTANCE * (encoderRightPosition / ENCODER_RESOLUTION);     // which equal to pi * diameter of wheel * (encoder counts / encoder resolution ) 
  Dc=(distanceLeftWheel + distanceRightWheel) /2 ;            // incremental linear displacement of the robot's centerpoint C
  Serial.println("Dc = ");
  Serial.print(Dc);
  Orientation_change =(distanceRightWheel - distanceLeftWheel)/WHEELBASE;    // the robot's incremental change of orientation , where b is the wheelbase of the mobile robot ,
  Orientation = Orientation + Orientation_change ;          //  The robot's new relative orientation   
  x = x + Dc * cos(Orientation);                            // the relative position of the centerpoint for mobile robot 
  Serial.println("x = ");
  Serial.print(x);
  y = y + Dc * sin(Orientation);
  Serial.println("y = ");
  Serial.print(y);
}

void doEncoder(){
  //  ---------- For Encoder 1 (Left)  -----------
  if (digitalRead(encoder1A) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder1B) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoderLeftPosition = encoderLeftPosition - 1;         // CCW
    } 
    else {
      encoderLeftPosition = encoderLeftPosition + 1;         // ClockWise
    }
  }

  else  {                                      // found a high-to-low on channel A
    if (digitalRead(encoder1B) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
     encoderLeftPosition = encoderLeftPosition + 1;          // ClockWise
    } 
    else {
      encoderLeftPosition = encoderLeftPosition - 1;          // CCW
    } 
  }
  //  ------------ For Encoder 2 (Right)-------------
  if (digitalRead(encoder2A) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder2B) == LOW) {  // check channel B to see which way  encoder is turning                                      
      encoderRightPosition = encoderRightPosition - 1;         // CCW
    } 
    else {
      encoderRightPosition = encoderRightPosition + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoder2B) == LOW) {   // check channel B to see which way  encoder is turning
     encoderRightPosition = encoderRightPosition + 1;          // CW
    } 
    else {
     encoderRightPosition = encoderRightPosition - 1;          // CCW
    }
   }
}

void CMPS2_decodeHeading(float measured_angle) {
  //decoding heading angle according to datasheet
  if (measured_angle > 337.25 | measured_angle < 22.5) {
    Serial.println("North");
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
    if (measured_angle > 292.5) {
      Serial.println("North-West");
    }
    else if (measured_angle > 247.5) {
      Serial.println("West");
    }
    else if (measured_angle > 202.5) {
      Serial.println("South-West");
    }
    else if (measured_angle > 157.5) {
      Serial.println("South");
    }
    else if (measured_angle > 112.5) {
      Serial.println("South-East");
    }
    else if (measured_angle > 67.5) {
      Serial.println("East");
    }
    else {
      Serial.println("North-East");
    }
  }
}

float CMPS2_getHeading(void) {
  float components[2];
  CMPS2_set(false);   //set the polarity to normal
  CMPS2_read_XYZ();  //read X, Y, Z components of the magnetic field
  components[0] = X;  //save current results
  components[1] = Y;
  CMPS2_set(true);   //set the polarity to normal
  CMPS2_read_XYZ();  //read X, Y, Z components of the magnetic field

  //eliminate offset from all components
  components[0] = (components[0] - X) / 2.0;
  components[1] = (components[1] - Y) / 2.0;

  //variables for storing partial results
  float temp0 = 0;
  float temp1 = 0;
  //and for storing the final result
  float deg = 0;

  //calculate heading from components of the magnetic field
  //the formula is different in each quadrant
  if (components[0] < Mid[0])
  {
    if (components[1] > Mid[1])
    {
      //Quadrant 1
      temp0 = components[1] - Mid[1];
      temp1 = Mid[0] - components[0];
      deg = 90 - atan(temp0 / temp1) * (180 / 3.14159);
    }
    else
    {
      //Quadrant 2
      temp0 = Mid[1] - components[1];
      temp1 = Mid[0] - components[0];
      deg = 90 + atan(temp0 / temp1) * (180 / 3.14159);
    }
  }
  else {
    if (components[1] < Mid[1])
    {
      //Quadrant 3
      temp0 = Mid[1] - components[1];
      temp1 = components[0] - Mid[0];
      deg = 270 - atan(temp0 / temp1) * (180 / 3.14159);
    }
    else
    {
      //Quadrant 4
      temp0 = components[1] - Mid[1];
      temp1 = components[0] - Mid[0];
      deg = 270 + atan(temp0 / temp1) * (180 / 3.14159);
    }
  }

  //correct heading
  deg += DECLINATION;
  if (DECLINATION > 0)
  {
    if (deg > 360) {
      deg -= 360;
    }
  }
  else
  {
    if (deg < 0) {
      deg += 360;
    }
  }

  return deg;
}

//reads measurements in mG
void CMPS2_read_XYZ(void) {
  //command internal control register 0 bit 0 (measure)
  Wire.beginTransmission(CMPS2_address);
  Wire.write(0x07);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(8);

  //wait for measurement to be completed
  bool flag = false;
  while (!flag) {
    //jump to status register
    Wire.beginTransmission(CMPS2_address);
    Wire.write(0x06);
    Wire.endTransmission();

    //read its value
    Wire.requestFrom(CMPS2_address, (uint8_t)1);
    int temporal = 0;
    if (Wire.available()) {
      temporal = Wire.read();
    }

    //if the last bit is 1, data is ready
    temporal &= 1;
    if (temporal != 0) {
      flag = true;
    }
  }

  //move address pointer to first address
  Wire.beginTransmission(CMPS2_address);
  Wire.write(0x00);
  Wire.endTransmission();

  //save data
  Wire.requestFrom(CMPS2_address, (uint8_t)6);
  byte tmp[6] = {0, 0, 0, 0, 0, 0}; //array for raw data
  if (Wire.available()) {
    for (int i = 0; i < 6; i++) {
      tmp[i] = Wire.read(); //save it
    }
  }

  //initialize array for data
  float measured_data[2];

  //reconstruct raw data
  measured_data[0] = 1.0 * (int)(tmp[1] << 8 | tmp[0]); //x
  measured_data[1] = 1.0 * (int)(tmp[3] << 8 | tmp[2]); //y

  //convert raw data to mG
  for (int i = 0; i < 2; i++) {
    measured_data[i] = 0.48828125 * (float)measured_data[i];
  }

  X = measured_data[0];
  Y = measured_data[1];

  //correct minimum, mid and maximum values
  if (measured_data[0] > Max[0]) { //x max
    Max[0] = measured_data[0];
  }
  if (measured_data[0] < Min[0]) { //x min
    Min[0] = measured_data[0];
  }
  if (measured_data[1] > Max[1]) { //y max
    Max[1] = measured_data[1];
  }
  if (measured_data[1] < Min[1]) { //y min
    Min[1] = measured_data[1];
  }
  for (int i = 0; i < 2; i++) { //mid
    Mid[i] = (Max[i] + Min[i]) / 2;
  }

  return;
}

//initialize the compass
void CMPS2_init(void) {
  Wire.begin(); // initialization of I2C bus

  //command internal control register 0 for set operation
  Wire.beginTransmission(CMPS2_address);
  Wire.write(0x07);
  Wire.write(0x20);
  Wire.endTransmission();
  delay(10);

  //command internal control register 1 to 16 bit resolution, 8ms measurement time
  Wire.beginTransmission(CMPS2_address);
  Wire.write(0x08);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  //initialize minimum, mid and maximum values
  for (int i = 0; i < 2; i++) {
    Max[i] = -32768;  //smallest int on 16 bits
    Min[i] = 32767;  //largest int on 16 bits
    Mid[i] = 0;
  }
}

//sets/resets the sensor, changing the magnetic polarity of the sensing element
void CMPS2_set(bool reset) {
  //command internal control register 0 bit 7 (capacitor recharge)
  Wire.beginTransmission(CMPS2_address);
  Wire.write(0x07);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(50);

  if (reset) {
    //command internal control register 0 bit 6 (reset)
    Wire.beginTransmission(CMPS2_address);
    Wire.write(0x07);
    Wire.write(0x40);
    Wire.endTransmission();
    delay(10);
  }
  else {
    //command internal control register 0 bit 5 (set)
    Wire.beginTransmission(CMPS2_address);
    Wire.write(0x07);
    Wire.write(0x20);
    Wire.endTransmission();
    delay(10);
  }
  return;
}
