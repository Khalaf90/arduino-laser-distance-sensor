
#define PulseTime 10  //ms
#define PulseHighTime 1  //ms  PulseLowTime = PulseTime - PulseHighTime

const byte pulsePin = 2;
const byte ledBoard = 13;
const byte Switch = 7;
// const byte analogPin = A0;   // Laser_Distanzsensor


void setup() {
//   Serial.begin(9600);
   pinMode(pulsePin, OUTPUT); 
   pinMode(ledBoard, OUTPUT);
   pinMode(Switch, INPUT_PULLUP);   

   digitalWrite(pulsePin, LOW);   
   digitalWrite(ledBoard, LOW); 
  
   if((PulseTime - PulseHighTime) <= 0) error_SOS();      
}


void loop() {

   while (!digitalRead (Switch)){
    pulse ();
    
   }

}

void pulse (){
  digitalWrite(pulsePin, HIGH);
  digitalWrite(ledBoard, HIGH);  
  delay(PulseHighTime); 
  digitalWrite(pulsePin, LOW); 
  digitalWrite(ledBoard, LOW);   
  delay(PulseTime - PulseHighTime);

  
}

void error_SOS(){
  for(;;){
    for(int i = 0; i<6; i++){
        digitalWrite(ledBoard, HIGH); 
        delay(100); 
        digitalWrite(ledBoard, LOW);
        delay(100);       
    }
    for(int i = 0; i<3; i++){
        digitalWrite(ledBoard, HIGH); 
        delay(500); 
        digitalWrite(ledBoard, LOW);      
        delay(500); 
    }
  }
}

