#include <Servo.h>

Servo clawServo; 
int clawAngle = 90;

void setup() {
  Serial.begin(9600);
  
  clawServo.attach(12);
  
  clawServo.write(clawAngle);
  Serial.print(clawAngle);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case 'o': 
        clawAngle = 0;
        break;
        
      case 'c':  // Закрыть
        clawAngle = 180;
        break;
        
      case 'k':
        clawAngle += 10;
        if (clawAngle > 180) clawAngle = 180;
        Serial.print(clawAngle);
        break;
        
      case 'l':
        clawAngle -= 10;
        if (clawAngle < 0) clawAngle = 0;
        Serial.print(clawAngle);
        break;
        
      case '1': clawAngle = 10; break;
      case '2': clawAngle = 20; break;
      case '3': clawAngle = 30; break;
      case '4': clawAngle = 40; break;
      case '5': clawAngle = 50; break;
      case '6': clawAngle = 60; break;
      case '7': clawAngle = 70; break;
      case '8': clawAngle = 80; break;
      case '9': clawAngle = 90; break;
        
      default:
        Serial.println("Команда: o, c, k, l, 1-9");
        break;
    }
    clawServo.write(clawAngle);
    
    while(Serial.available() > 0) {
      Serial.read();
    }
  }
}
