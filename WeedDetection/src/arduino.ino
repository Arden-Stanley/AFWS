/*
bool isTrue = false;
void setup() {
 Serial.begin(9600);
 pinMode(13, OUTPUT);
 

sketch_feb20a.ino
3 KB
﻿
gamerdude
gamerdude1
 
 
/*
bool isTrue = false;
void setup() {
 Serial.begin(9600);
 pinMode(13, OUTPUT);
 
}

void loop() {
  if(Serial.available() > 0){
    String verify = Serial.readString();
    int received_verify = verify.toInt();
    digitalWrite(13, LOW);
    
    if(isTrue == false){
      isTrue = true;
      Serial.println("Received");
      if(received_verify == 0){
        digitalWrite(13, HIGH);
        delay(5000);
        digitalWrite(13, LOW);
        delay(5000);
        isTrue = false;
      }
    }
  }

}


// Testing some things 

// Cell 0-4 Pin locations

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}


int cell_pin_01 = 13;
int cell_pin_02 = 14;
int cell_pin_03 = 15;
int cell_pin_04 = 16;

bool isTrue = false;
void setup() {
 Serial.begin(9600);

 while(!Serial && millis() < 5000);
 pinMode(cell_pin_01, OUTPUT);
 pinMode(cell_pin_02, OUTPUT);
 pinMode(cell_pin_03, OUTPUT);
 pinMode(cell_pin_04, OUTPUT);
 
 
}

void loop() {
  
  if(Serial.available() > 0){
    //digitalWrite(15, HIGH);
    String verify = Serial.readString();
    int received_verify = verify.toInt();
    
    // If cases
   // Let i be an integer for all integers 
   // Case i - weed in cell-0{i} 
    clearSerialBuffer();
    if(isTrue == false){
      isTrue = true;
      //Serial.println("Received");
      if(received_verify == 0){
        digitalWrite(cell_pin_01, HIGH);
        delay(3000);
        digitalWrite(cell_pin_01, LOW); 
        isTrue = false;
      }
      if(received_verify == 1){
        digitalWrite(cell_pin_02, HIGH);
        delay(3000);
        digitalWrite(cell_pin_02, LOW);
        isTrue = false;
      }
       if(received_verify == 2){
        digitalWrite(cell_pin_03, HIGH);
        delay(3000);
        digitalWrite(cell_pin_03, LOW);
        isTrue = false;
      }
      if(received_verify == 3){
        digitalWrite(cell_pin_04, HIGH);
        delay(3000);
        digitalWrite(cell_pin_04, LOW);
        isTrue = false;
      }
    }
  }

}

*/


unsigned long startTime = 0;
bool active[4] = {false, false, false, false};

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
}

void clearSerialBuffer() {
  while (Serial.available()) Serial.read();
}

void triggerCell(int cell) {
  digitalWrite(cell + 13, HIGH);   // activate
  startTime = millis();
  active[cell] = true;
}

void loop() {

  // Read newest serial command
  if (Serial.available()) {
    char c = Serial.read();
    clearSerialBuffer();

    if (c >= '0' && c <= '3') {
      int cell = c - '0';
      triggerCell(cell);
      Serial.print("ACK");
      Serial.println(cell);
    }
  }

  // Non-blocking timer control
  for (int i = 0; i < 4; i++) {
    if (active[i] && millis() - startTime > 3000) {
      digitalWrite(i + 13, LOW);   // deactivate
      active[i] = false;
    }
  }
}
