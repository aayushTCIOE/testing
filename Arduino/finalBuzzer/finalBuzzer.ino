bool getConditionFromApp() {
  return random(0, 2);  // Random true or false
}
void sendSMS() {
  Serial.println("Sending SMS..."); 
}
const int buzzerPin = 27;      
const int switchPin = 14;      

      
bool buzzerState = false;     
bool buttonPressed = false;   

void setup() {
  pinMode(buzzerPin, OUTPUT);         
  pinMode(switchPin, INPUT_PULLUP);   
  Serial.begin(115200);               
}



void loop() {
  
 bool condition = getConditionFromApp();  
  if (condition && !buzzerState) {
    digitalWrite(buzzerPin, HIGH);  
    buzzerState = true;
    Serial.println("Buzzer ON");

    unsigned long startTime = millis();  
    
    while (millis() - startTime < 10000) {  
      if (digitalRead(switchPin) == LOW) {  
        delay(200);  
        if (digitalRead(switchPin) == LOW) {  
          digitalWrite(buzzerPin, LOW);  
          Serial.println("Buzzer turned OFF - No SMS sent");
          buttonPressed = true;
          
         
          while (digitalRead(switchPin) == LOW);
          delay(200); 
          break; 
        }
      }
    }

    if (!buttonPressed) {
      Serial.println("Buzzer continues ringing - No SMS sent");
      //  digitalWrite(buzzerPin, HIGH); 
    }
   
    if (buttonPressed) {
      sendSMS();  
    }   
    buzzerState = false;
    buttonPressed = false;
  }  
  delay(1000);  
  }
