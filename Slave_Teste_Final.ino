// Slave: Serial Communication between two Arduino Boards
#include <NewPing.h>
#include <Servo.h>

char number  = ' ';     //Comunicação entre dois Arduinos



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
    
//Definir Encoders
//#define encoder1A  0       //signal A of left encoder  (white wire)
//#define encoder1B  1       //signal B of left encoder  (yellow wire)
//#define encoder2A  2       //signal A of right encoder  (white wire)
//#define encoder2B  3       //signal B of right encoder  (yellow wire)

//volatile int encoderLeftPosition = 0;     // counts of left encoder 
//volatile int encoderRightPosition = 0;    // counts of right encoder 
//float  DIAMETER  = 65  ;                  // wheel diameter (in mm)  
//float distanceLeftWheel, distanceRightWheel, Dc, Orientation_change;
//float ENCODER_RESOLUTION = 333.3;         //encoder resolution (in pulses per revolution)  where in Rover 5,  1000 state changes per 3 wheel rotations 
//int x = 0;                                // x initial coordinate of mobile robot 
//int y = 0;                                // y initial coordinate of mobile robot 
//float Orientation  = 0;                   // The initial orientation of mobile robot 
//float WHEELBASE=130  ;                    //  the wheelbase of the mobile robot in mm
//float CIRCUMSTANCE =PI * DIAMETER  ;



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

  //sensore IR
    pinMode(left_sensor_pin, INPUT);
    pinMode(right_sensor_pin, INPUT);
  
  //servo
    servo.attach(8);
    servo.write(90);

   detetar = true;
   
}

void loop() {
  infra();
//String readString;
//String Q;
// 
//while (Serial.available()) {
//      delay(1);
//    if (Serial.available() >0) {
//      char c = Serial.read();  //gets one byte from serial buffer
//    if (isControl(c)) {
//      //'Serial.println("it's a control character");
//      break;
//    }
//      readString += c; //makes the string readString    
//    }
// }   


//  Serial.println("Auto");
//  infra();  
//Q = readString;
//  
// if (Q  == "8" && Q != "9") { 
//  
//  Serial.println("Auto");
//  left_sensor_state = analogRead(left_sensor_pin);
//  right_sensor_state = analogRead(right_sensor_pin);
  
//   if(right_sensor_state < 200 && left_sensor_state < 200) {
//    objectAvoid();
//    //forward
//    
//  }else if(right_sensor_state < 200 && left_sensor_state > 200) {
//    objectAvoid();
//    //turn right
//    moveRight();
//    
//  }else if(right_sensor_state > 200 && left_sensor_state < 200) {
//    objectAvoid();
//    //turn left
//    moveLeft();
//    
//  }else if(right_sensor_state > 200 && left_sensor_state > 200) {
//    objectAvoid();
//    //stop
//    Stop();
    
//  }  
// }
//
// if (Q == "9" && Q != "8")  {
//  Serial.println("Manual");
//  manual();
// }  
// if (Q=="1") { 
//      Serial.println("Front");
//      digitalWrite(inR3, HIGH);   //Andar frente
//      digitalWrite(inR4, LOW);
//      digitalWrite(inL1, HIGH);
//      digitalWrite(inL2, LOW);
//      analogWrite(enR, 150);
//      analogWrite(enL , 150); 
//      Serial.println("anda caralho!"); 
//      }
//
//       if (Q=="2") { 
//      Serial.println("Back");
//      digitalWrite(inR3, LOW);   //Andar trás
//      digitalWrite(inR4, HIGH);
//      digitalWrite(inL1, LOW);
//      digitalWrite(inL2, HIGH);
//      analogWrite(enR, 150);
//      analogWrite(enL , 150);  
//      }
//
//       if (Q=="3") { 
//      Serial.println("Left");
//      digitalWrite(inR3, HIGH);   //Andar Esquerdo
//      digitalWrite(inR4, LOW);
//      digitalWrite(inL1, HIGH);
//      digitalWrite(inL2, LOW);
//      analogWrite(enR, 150);
//      analogWrite(enL , 100);  
//      }
//
//      if (Q=="4") { 
//      Serial.println("Left 360");
//      digitalWrite(inR3, HIGH);   //Andar 360 Esquerdo
//      digitalWrite(inR4, LOW);
//      digitalWrite(inL1, LOW);
//      digitalWrite(inL2, HIGH);
//      analogWrite(enR, 150);
//      analogWrite(enL , 150);  
//      }
//
//      if (Q=="5") { 
//      Serial.println("Right");
//      digitalWrite(inR3, HIGH);   //Andar Direito
//      digitalWrite(inR4, LOW);
//      digitalWrite(inL1, HIGH);
//      digitalWrite(inL2, LOW);
//      analogWrite(enR, 100);
//      analogWrite(enL , 150);  
//      }
//
//      if (Q=="6") { 
//      Serial.println("Right 360");
//      digitalWrite(inR3, LOW);   //Andar 360 Direito
//      digitalWrite(inR4, HIGH);
//      digitalWrite(inL1, HIGH);
//      digitalWrite(inL2, LOW);
//      analogWrite(enR, 150);
//      analogWrite(enL , 150);  
//      }
//
//      if (Q=="7") { 
//      Serial.println("Stop");
//      digitalWrite(inR3, LOW);   //Parar
//      digitalWrite(inR4, LOW);
//      digitalWrite(inL1, LOW);
//      digitalWrite(inL2, LOW);
//      analogWrite(enR, 0);
//      analogWrite(enL , 0);  
//      }

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
void slow (){
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
  digitalWrite(inR3, LOW);   //Andar Direito
  digitalWrite(inR4, HIGH);
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  analogWrite(enR, 150);
  analogWrite(enL , 150); 
}


void Left360() {
  digitalWrite(inR3, HIGH);   //Andar Esquerdo
  digitalWrite(inR4, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  analogWrite(enR, 150);
  analogWrite(enL , 150);  
}

///////////////////////////////////////////////////////////////////////////////////////////

void infra() {

  left_sensor_state = analogRead(left_sensor_pin);
  right_sensor_state = analogRead(right_sensor_pin);
  
   if(right_sensor_state < 200 && left_sensor_state < 200) {
    objectAvoid();
    //forward
    
    
  }else if(right_sensor_state < 200 && left_sensor_state > 200) {
    objectAvoid();
    //turn right
    Right360();
    //moveRight();
    
  }else if(right_sensor_state > 200 && left_sensor_state < 200) {
    objectAvoid();
    //turn left
    Left360();
    //moveLeft();
    
  }else if(right_sensor_state > 200 && left_sensor_state > 200) {
    objectAvoid();
    //stop
    moveBackward();
    delay(50);
    Stop();
    
    
  }
}


void objectAvoid() {
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


int getDistance() {
  
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
  
}


int lookLeft () {
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

int lookRight() {
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



void turn() {                     //Problema
  
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

//  else if {object == true)
//    Serial.println("turn Left");   //Virar Esquerda - Mais distância
//    moveBackward();
//    delay(400);
//    Stop();
//    delay(400);
//
//    
//    Left360();
//    delay(600);
//    Stop();
//    delay(400);
//    
//    moveForward();
//    delay(8000);
//    Stop();
//    delay(400);
//    
//    Right360();
//    delay(700);
//    Stop();
//    servo.write(0);
//    delay(400);
//
//    distanceTurn = getDistance();
//    while (distanceTurn <= 20) {
//        slow();
//    }
//  
//    Right360();
//    delay(300);
//
//    left_sensor_turn = analogRead(left_sensor_pin);
//    right_sensor_turn = analogRead(right_sensor_pin);
//      
//    while (right_sensor_turn > 200 || left_sensor_turn > 200) {
//          slow();
//    }
//       
//    Stop();
//    delay(500);
//    detetar = false;
//    infra();
//  }
}



//void encoder() {
//  distanceLeftWheel = CIRCUMSTANCE * (encoderLeftPosition / ENCODER_RESOLUTION);       //  travel distance for the left and right wheel respectively 
//  distanceRightWheel = CIRCUMSTANCE * (encoderRightPosition / ENCODER_RESOLUTION);     // which equal to pi * diameter of wheel * (encoder counts / encoder resolution ) 
//  Dc=(distanceLeftWheel + distanceRightWheel) /2 ;            // incremental linear displacement of the robot's centerpoint C
//  Serial.println("Dc = ");
//  Serial.print(Dc);
//  Orientation_change =(distanceRightWheel - distanceLeftWheel)/WHEELBASE;    // the robot's incremental change of orientation , where b is the wheelbase of the mobile robot ,
//  Orientation = Orientation + Orientation_change ;          //  The robot's new relative orientation   
//  x = x + Dc * cos(Orientation);                            // the relative position of the centerpoint for mobile robot 
//  Serial.println("x = ");
//  Serial.print(x);
//  y = y + Dc * sin(Orientation);
//  Serial.println("y = ");
//  Serial.print(y);
//}
//
//void doEncoder(){
//  //  ---------- For Encoder 1 (Left)  -----------
//  if (digitalRead(encoder1A) == HIGH) {   // found a low-to-high on channel A
//    if (digitalRead(encoder1B) == LOW) {  // check channel B to see which way
//                                             // encoder is turning
//      encoderLeftPosition = encoderLeftPosition - 1;         // CCW
//    } 
//    else {
//      encoderLeftPosition = encoderLeftPosition + 1;         // ClockWise
//    }
//  }
//
//  else  {                                      // found a high-to-low on channel A
//    if (digitalRead(encoder1B) == LOW) {   // check channel B to see which way
//                                              // encoder is turning  
//     encoderLeftPosition = encoderLeftPosition + 1;          // ClockWise
//    } 
//    else {
//      encoderLeftPosition = encoderLeftPosition - 1;          // CCW
//    } 
//  }
//  //  ------------ For Encoder 2 (Right)-------------
//  if (digitalRead(encoder2A) == HIGH) {   // found a low-to-high on channel A
//    if (digitalRead(encoder2B) == LOW) {  // check channel B to see which way  encoder is turning                                      
//      encoderRightPosition = encoderRightPosition - 1;         // CCW
//    } 
//    else {
//      encoderRightPosition = encoderRightPosition + 1;         // CW
//    }
//  }
//  else                                        // found a high-to-low on channel A
//  { 
//    if (digitalRead(encoder2B) == LOW) {   // check channel B to see which way  encoder is turning
//     encoderRightPosition = encoderRightPosition + 1;          // CW
//    } 
//    else {
//     encoderRightPosition = encoderRightPosition - 1;          // CCW
//    }
//   }
//}
