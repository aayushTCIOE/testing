// const int ledPin = 4;  // Using GPIO4

// void setup() {
//   pinMode(ledPin, OUTPUT);  // Set GPIO4 as output
// }

// void loop() {
//   digitalWrite(ledPin, HIGH);  // Turn LED on
//   delay(1000);                 // Wait 1 second
//   digitalWrite(ledPin, LOW);   // Turn LED off
//   delay(1000);                 // Wait 1 second
// }




// const int ledPin = 4;      // LED on GPIO4
// const int switchPin = 5;   // Let's use GPIO5 for the switch

// // Variable to store the condition
// bool condition = true;     // Starting condition is true

// void setup() {
//   pinMode(ledPin, OUTPUT);           // LED pin as output
//   pinMode(switchPin, INPUT_PULLUP);  // Switch pin as input with pullup
//   Serial.begin(115200);             // Start serial communication
// }

// void loop() {
//   // Check if condition is true
//   if (condition) {
//           Serial.println("LED is turned on");

//     digitalWrite(ledPin, HIGH);  // Turn LED on
    
//     // Check if switch is pressed (will read LOW when pressed due to INPUT_PULLUP)
//     if (digitalRead(switchPin) == LOW) {
//       digitalWrite(ledPin, LOW);    // Turn LED off
//       Serial.println("LED is turned off");
//       condition = false;            // Update condition
//       delay(200);                  // Small delay to avoid switch bounce
//     }
//   }
// }


// const int ledPin = 4;      
// const int switchPin = 5;   

// bool condition = true;     
// bool ledState = false;     

// void setup() {
//   pinMode(ledPin, OUTPUT);           // LED pin as output
//   pinMode(switchPin, INPUT_PULLUP);  // Enable internal pull-up resistor
//   Serial.begin(115200);              // Start serial communication
// }

// void loop() {
//   if (condition && !ledState) {
//     digitalWrite(ledPin, HIGH);  // Turn LED on
//     ledState = true;
//   }

//   // Check if switch is pressed (LOW due to pull-up)
//   if (digitalRead(switchPin) == LOW) {
//     delay(200);  // Debounce delay
//     if (digitalRead(switchPin) == LOW) {  // Confirm button is still pressed
//       digitalWrite(ledPin, LOW);  // Turn LED off
//       Serial.println("LED is turned off");
//       condition = false;  // Prevent message from being sent again
//       ledState = false;
      
//       // Wait for button release to prevent repeated triggering
//       while (digitalRead(switchPin) == LOW);
//       delay(200); // Extra debounce delay
//     }
//   }
// }


// const int ledPin = 4;      // LED on GPIO4
// const int switchPin = 5;   // Push button on GPIO5

// bool condition = true;     // Initial condition
// bool ledState = false;     // Track LED state

// void setup() {
//   pinMode(ledPin, OUTPUT);           // LED pin as output
//   pinMode(switchPin, INPUT_PULLUP);  // Enable internal pull-up resistor
//   Serial.begin(115200);              // Start serial communication
// }

// void loop() {
//   // If condition is true and LED is OFF, turn it ON
//   if (condition && !ledState) {
//     digitalWrite(ledPin, HIGH);  // Turn LED on
//     ledState = true;
//   }

//   // Check if switch is pressed (LOW due to pull-up)
//   if (digitalRead(switchPin) == LOW) {
//     delay(200);  // Debounce delay
//     if (digitalRead(switchPin) == LOW) {  // Confirm button is still pressed
//       digitalWrite(ledPin, LOW);  // Turn LED off
//       Serial.println("LED is turned off");
//       ledState = false;

//       condition = false;  // Temporarily set condition false

//       // Wait for button release to prevent repeated triggering
//       while (digitalRead(switchPin) == LOW);
//       delay(200); // Extra debounce delay

//       condition = true;  // Reset condition in the next iteration
//     }
//   }
// }

// const int buzzerPin = 4;      // Buzzer connected to GPIO4
// const int switchPin = 5;      // Push button on GPIO5

// bool condition = true;        // Initial condition
// bool buzzerState = false;     // Track Buzzer state

// void setup() {
//   pinMode(buzzerPin, OUTPUT);         // Buzzer pin as output
//   pinMode(switchPin, INPUT_PULLUP);   // Enable internal pull-up resistor
//   Serial.begin(115200);               // Start serial communication
// }

// void loop() {
//   // If condition is true and buzzer is OFF, turn it ON
//   if (condition && !buzzerState) {
//     digitalWrite(buzzerPin, HIGH);  // Turn Buzzer ON
//     buzzerState = true;
//   }

//   // Check if switch is pressed (LOW due to pull-up)
//   if (digitalRead(switchPin) == LOW) {
//     delay(200);  // Debounce delay
//     if (digitalRead(switchPin) == LOW) {  // Confirm button is still pressed
//       digitalWrite(buzzerPin, LOW);  // Turn Buzzer OFF
//       Serial.println("Buzzer is turned off");
//       buzzerState = false;

//       condition = false;  // Temporarily set condition false

//       // Wait for button release to prevent repeated triggering
//       while (digitalRead(switchPin) == LOW);
//       delay(200); // Extra debounce delay

//       // condition = true;  // Reset condition in the next iteration
//     }
//   }
// }


const int buzzerPin = 4;      
const int switchPin = 5;     

bool condition = true;        
bool buzzerState = false;    
bool buttonPressed = false;   

void setup() {
  pinMode(buzzerPin, OUTPUT);         
  pinMode(switchPin, INPUT_PULLUP);  
  Serial.begin(115200);               
}

void sendSMS() {
  Serial.println("Sending SMS..."); } 

void loop() {
  
  if (condition) {    
    digitalWrite(buzzerPin, HIGH);
    buzzerState = true;
    Serial.println("Buzzer ON - Waiting 10 seconds for button press...");

    unsigned long startTime = millis();  
   

    while (millis() - startTime < 10000) {  
      if (digitalRead(switchPin) == LOW) { 
        delay(200);  // Debounce delay
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
      digitalWrite(buzzerPin, LOW);  
      sendSMS();  
    }

   
    // buttonPressed = false;

    delay(1000); 
  }
}

