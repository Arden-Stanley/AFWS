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

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    char verify = Serial.parseInt();   // Reads one character 
    digitalWrite(13, LOW);         // Turns off the pump 

    if (verify == 0) {           // compare to character '0'
      digitalWrite(13, HIGH);      // turn on motor
      delay(5000);
      digitalWrite(13, LOW);       // turn off motor
      delay(5000);
    }
  }
}


