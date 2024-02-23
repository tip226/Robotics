HardwareSerial MySerial0(0);

int led_pin_red = D3;
int led_pin_blue = D4;

void setup() {
  pinMode(led_pin_red, OUTPUT);
  pinMode(led_pin_blue, OUTPUT);
  
  // Initialize serial communication
  Serial.begin(9600);
  MySerial0.begin(19200, SERIAL_8N1, -1, -1);
}

void loop() {
  if (MySerial0.available() > 0) {
    char inChar = (char)MySerial0.read();
    
    switch(inChar) {
      case 'R': // Red only
        digitalWrite(led_pin_red, HIGH);
        digitalWrite(led_pin_blue, LOW);
        Serial.println("Red Detected");
        break;
      case 'B': // Blue only
        digitalWrite(led_pin_red, LOW);
        digitalWrite(led_pin_blue, HIGH);
        Serial.println("Blue Detected");
        break;
      case 'RB': // Both Red and Blue
        digitalWrite(led_pin_red, HIGH);
        digitalWrite(led_pin_blue, HIGH);
        Serial.println("Red and Blue Detected");
        break;
      case 'N': // No color detected
        digitalWrite(led_pin_red, LOW);
        digitalWrite(led_pin_blue, LOW);
        Serial.println("No Color Detected");
        break;
    }
  }
}
