#define wheel0 50 // Initialize pin 4 to drive a wheel
#define wheel1 51 
#define wheel2 52 
#define wheel3 53 

int state = 0; // To read state of serial data from HC05 module

void stopWheel(){
  digitalWrite(wheel0, LOW); // Turn wheels off
  digitalWrite(wheel1, LOW);
  digitalWrite(wheel2, LOW);
  digitalWrite(wheel3, LOW);
}

void forwardWheel(){
  digitalWrite(wheel0, HIGH); // Turn wheels all on
  digitalWrite(wheel1, LOW);
  digitalWrite(wheel2, HIGH);
  digitalWrite(wheel3, LOW);
}

void leftTurnWheel(){
  digitalWrite(wheel0, LOW); // Turn left wheels back, right wheels forward
  digitalWrite(wheel1, HIGH);
  digitalWrite(wheel2, HIGH);
  digitalWrite(wheel3, LOW);
}

void rightTurnWheel(){
  digitalWrite(wheel0, HIGH); // Turn left wheels forward, right wheeels back
  digitalWrite(wheel1, LOW);
  digitalWrite(wheel2, LOW);
  digitalWrite(wheel3, HIGH);
}

void oopsiesWheel() {
  digitalWrite(wheel0, LOW); // backwards
  digitalWrite(wheel1, HIGH);
  digitalWrite(wheel2, LOW);
  digitalWrite(wheel3, HIGH);
}

void checkStateWheel(int instr){
  if (instr == '0') {
    stopWheel();
    Serial1.println("STOP"); 
  }
  else if (instr == '1') {
    forwardWheel();
    Serial1.println("Forward ");
  }
  else if (instr == '2') {
    leftTurnWheel();
    Serial1.println("Left");
  }
  else if (instr == '3') {
    rightTurnWheel();
    Serial1.println("Right Turn");
  }
  else if (instr == '4') {
    oopsiesWheel();
    Serial1.println("backward");
  }
  instr = 0;
}

void setup() {
  pinMode(wheel0, OUTPUT);//Define wheel as output
  pinMode(wheel1, OUTPUT);//Define  as output
  pinMode(wheel2, OUTPUT);//Define  as output
  pinMode(wheel3, OUTPUT);//Define  as output
  digitalWrite(wheel0, LOW);
  digitalWrite(wheel1, LOW);
  digitalWrite(wheel2, LOW);
  digitalWrite(wheel3, LOW);
  Serial1.begin(9600); // Default communication rate of the Bluetooth module
}





void loop() {
  if (Serial1.available() > 0) { // Checks whether data is coming from the serial port
    state = Serial1.read(); // Reads the data from the serial port
  }
  
  checkStateWheel(state); //determine if wheels need spin
 delay(100);
}

