#define IR 2    // digital pin input for ir sensor
int detection = HIGH;    // no obstacle
int i = 0;
// array digital pin for: green led(3,4,5) - white led (6,7,8)- red led (9,10,11)
int LedPIN[] = {3, 4, 5, 6, 7, 8, 9, 10, 11};   
 
void setup() {
  pinMode(IR, INPUT);
  Serial.begin(9600);
}
void loop() {
  detection = digitalRead(IR);
  if(detection == LOW){ 
    BlinkLED();
  }          
  else{                 
    LedOFF();
  }
delay(1);
}
