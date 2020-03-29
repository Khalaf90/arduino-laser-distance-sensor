
#define durationTime  250 //ms
#define sampleTime    1  //ms  fix

const float A = 119.94;
const float B = - 121.61;

float distance ;

const float voltPow = 5.04;
float voltAdjust,voltage;
long valAverage;


const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  9;      // the number of the LED pin
const int ledBoardPin =  13;
const int laserPin = A0;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
 
void setup() {

  //

  Serial.begin(9600);


 //

 voltAdjust=voltPow/1023.0;

  // initialize the LED pin as an output:

  pinMode(ledPin, OUTPUT);
  pinMode(ledBoardPin, OUTPUT);
  digitalWrite(ledBoardPin, HIGH);
    
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
}
 
void loop() {
  // read the state of the pushbutton value:
  buttonState = !digitalRead(buttonPin);
  
/*  Serial.print(buttonState);  Serial.print("    LASER:  ");
  Serial.println(analogRead(laserPin));
*/
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledBoardPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledBoardPin, LOW);
  }
//

 valAverage=0;

  for(int i = 0; i < durationTime; i++){

  valAverage+=analogRead(laserPin);
  delay(sampleTime);
  }

  voltage=valAverage*voltAdjust/durationTime;

  distance=(A*voltage)+ B;
  
  Serial.print(voltage,3);  Serial.print("    LASER:  ");
  Serial.println(distance);
  
}

  

