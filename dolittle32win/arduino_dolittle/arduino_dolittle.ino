void setup(){
  Serial.begin(9600);
  Serial.write(' ');
}
void loop(){
  int wait=5;
  if (Serial.available() > 0) {
    byte in = Serial.read();
    byte cmd = in & 0xe0;
    byte port = in & 0x1f;

    switch (cmd) {
    case 0x40: // pinMode(Dout)
      pinMode(port ,OUTPUT);
      delay(wait);
      break;
    case 0x60: // pinMode(Din)
      pinMode(port ,INPUT);
      delay(wait);
      break;
    case 0x80: // Dout
      delay(wait);
      if (Serial.read() == 1) { digitalWrite(port, HIGH); } else { digitalWrite(port, LOW); }
      break;
    case 0xA0: // Din
      Serial.write(digitalRead(port));
      break;
    case 0xC0: // Aout
      delay(wait);
      analogWrite(port, Serial.read());
      break;
    case 0xE0: // Ain
      Serial.write(analogRead(port)/4);
      break;

    default : 
      break;
    }
  }
}
