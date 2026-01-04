#include <Arduino.h>
#include <ArduinoQueue.h>

#define ll unsigned long
#define LAMP_PIN2 2 
#define LAMP_PIN4 4 
#define LAMP_PIN12 12 
#define LAMP_PIN27 27 
#define micros_min 100000 
#define micros_max 105000 
#define lamp_duration 500000
#define motorpin1 18
#define motorpin2 19
#define pwm 21

struct item{
  int index; 
  ll timestamp;
};

ArduinoQueue<item> block; 

String serial_input_buffer = ""; 
bool new_input_ready = false;
ll lamp2_time = 0; 
ll lamp4_time = 0;
ll lamp12_time = 0;
ll lamp27_time = 0;

ll timestamp() {
  return micros(); 
}

void handle_lamp_blink() {
  
  if (digitalRead(LAMP_PIN2) == HIGH && (micros() - lamp2_time >= lamp_duration)) {
    digitalWrite(LAMP_PIN2, LOW);
  }

  if (digitalRead(LAMP_PIN4) == HIGH && (micros() - lamp4_time >= lamp_duration)) {
    digitalWrite(LAMP_PIN4, LOW);
  }

  if (digitalRead(LAMP_PIN12) == HIGH && (micros() - lamp12_time >= lamp_duration)) {
    digitalWrite(LAMP_PIN12, LOW);
  }

  if (digitalRead(LAMP_PIN27) == HIGH && (micros() - lamp27_time >= lamp_duration)) {
    digitalWrite(LAMP_PIN27, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  serial_input_buffer.reserve(20);
  pinMode(LAMP_PIN2, OUTPUT);
  pinMode(LAMP_PIN4, OUTPUT);
  pinMode(LAMP_PIN12, OUTPUT);
  pinMode(LAMP_PIN27, OUTPUT);
  pinMode(motorpin1,OUTPUT);
  pinMode(motorpin2,OUTPUT);
  ledcAttach(pwm, 5000, 8);  
  ledcWrite(pwm, 255);       
  digitalWrite(motorpin1,HIGH);
  digitalWrite(motorpin2,LOW);
  for(int i=0;i<3;i++){
    digitalWrite(LAMP_PIN12,HIGH);
    digitalWrite(LAMP_PIN4,HIGH);
    delay(1000);
    digitalWrite(LAMP_PIN12,LOW);
    digitalWrite(LAMP_PIN4,LOW);
    digitalWrite(LAMP_PIN27,HIGH);
    digitalWrite(LAMP_PIN2,HIGH);
    delay(1000);
    digitalWrite(LAMP_PIN27,LOW);
    digitalWrite(LAMP_PIN2,LOW);
  }
}

void loop() {
  while (Serial.available()) {
    char inChar = Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      serial_input_buffer.trim(); 
      if (serial_input_buffer.length() > 0) {
        new_input_ready = true;
      }
      break; 
    } else {
      serial_input_buffer += inChar;
    }
  }

  if (new_input_ready) {
    int current_index = serial_input_buffer.toInt();
    
    serial_input_buffer = "";
    new_input_ready = false;
    
    if (current_index >= 1 && current_index <= 4) { 
        ll index_time = timestamp();
        block.enqueue(item{current_index, index_time});
        Serial.print("Block at index ");
        Serial.print(current_index);
        Serial.println(" added.");
    } else {
      Serial.println("No Action, index out of range");
    }
  }

  handle_lamp_blink();

  if (!block.isEmpty()) { 
    ll elapsed_time = timestamp() - block.front().timestamp;
    
    if (elapsed_time >= micros_min && elapsed_time <= micros_max) {
      if (block.front().index == 1) {
        digitalWrite(LAMP_PIN4, HIGH);
        lamp4_time = micros();
      } else if(block.front().index == 2){
        digitalWrite(LAMP_PIN2, HIGH);
        lamp2_time = micros();
      } else if(block.front().index == 3){
        digitalWrite(LAMP_PIN27, HIGH);
        lamp27_time = micros();
      } else if (block.front().index == 4) {
        digitalWrite(LAMP_PIN12, HIGH);
        lamp12_time = micros();
      }

      Serial.print("Block at index ");
      Serial.print(block.front().index);
      Serial.println(" had falled succesfully.");

      block.dequeue();

    } else if (elapsed_time > micros_max + 5000) { 
        Serial.print("Warning : Block ");
        Serial.print(block.front().index);
        Serial.println(" missed the gate. Dirty block entered the clean box.");
        block.dequeue();
    }
  }
}
