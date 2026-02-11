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

*/
// Testing some things 

bool isTrue = false;
void setup() {
 Serial.begin(9600);
 pinMode(13, OUTPUT);
 
}

void loop() {
  digitalWrite(13, HIGH);
  if(Serial.available() > 0){
    String verify = Serial.readString();
    int received_verify = verify.toInt();
    
    // If cases
   // Let i be an integer for all integers 
   // Case i - weed in cell-0{i} 
   
    if(isTrue == false){
      isTrue = true;
      Serial.println("Received");
      if(received_verify == 0){
        digitalWrite(13, LOW);
        delay(3000);
        digitalWrite(13, HIGH);
        delay(3000);
        isTrue = false;
      }
      if(received_verify == 1){
        digitalWrite(13, LOW);
        delay(3000);
        digitalWrite(13, HIGH);
        delay(3000);
        isTrue = false;
      }
       if(received_verify == 2){
        digitalWrite(13, LOW);
        delay(3000);
        digitalWrite(13, HIGH);
        delay(3000);
        isTrue = false;
      }
      if(received_verify == 3){
        digitalWrite(13, LOW);
        delay(3000);
        digitalWrite(13, HIGH);
        delay(3000);
        isTrue = false;
      }
    }
  }

}

