// 5 Cameras Version
// Date: 07.08.2018
// Trigger 5 Cameras (Oversampling and Minimum Detect Time, Gab Timeout and Exposure Time)
// MK

//============================================================================================
//!!! Which Sensor Tigger Input is to compile: #define SWITCH  or  LASER  or  LASER_AND_SWITCH
#define LASER
//============================================================================================

#include <TimerOne.h>
#include <MsTimer2.h>
#include <Oversample.h>
 
Oversample * sampler;
Oversample * exposure;

//=== USER DEFINE VALUES ====================================================================
#define FramesPerSecond               60    // FrameTime = 1/FramesPerSecond

#define LKW_DistanceTriggerThreshold  80    // cm
#define WaitTimeForMatlap             5000  // ms  Wait: do nothing for Matlab

/*-------------------------------------------------------------------------------------------
LaserDetect_LKW_Min_Time:  Nach dem ersten "DETECT" wird trotz "LKW-Laser-LÜCKEN" "DETECT-LKW"
                           auf "TRUE" gesetzt, garantierte Aufnahmedauer
LaserGap_LKW_TimeOut    :  Nach LaserDetect_LKW_Min_Time wird nach einer "LKW-Laser-Lücke oder
                           Schmutz am LKW oder das Ende des LKWs" für weitere
                           LaserGap_LKW_TimeOut "DETECT-LKW" auf "TRUE" gesetzt
-------------------------------------------------------------------------------------------*/
#define WaitAfterDistanceTriggerBefore    0  //ms

#define LaserDetect_LKW_Min_Time          200   //ms 
#define LaserGap_LKW_TimeOut              WaitAfterDistanceTriggerBefore + 500   //ms
#define Delay_LKW_End_Millis              100   //ms  Scan time End LKW = Delay_LKW_End_Millis + 25ms
#define SendDataTimeRate                  10    //ms  must be >= 4 ms  (4ms ==>  250 Samples/Second)
#define EOD                               -777  // End Of Data


//============================================================================================

//--- laser calibration coefficients 
const float OffsetLaserFrontPlatte = 22;    //cm
const float A = 119.94;               // distance = A * Voltage + B  [cm]
const float B = - 120.0 + OffsetLaserFrontPlatte;

 
//---  Adjust Onboard Power Supply for Analog-to-digital converter (ADC), for each board different 
const float voltPow = 4.996;
float voltAdjust;

//--- define Pins -----------------------------------------------------------------------------
const byte detectPin   = 8;      
const byte duoLedGreen = 13;     // Duo LED
const byte duoLedRed   = 8;      // detect pin with one LONG PULSE, not frames pulse
const byte swTest      = 12;     // test with switch
const byte laserPin    = A0;     // Laser Distance Sensor: Voltage-Signal Input
const byte exposurePin = A1;     // Poti Exposure
const byte timer1Pin   = 9;      // PWM to trigger camera frames with Exposure pulse wide

// --- Laser Resolution, don't change this values! --------------------------------------------
#define LaserResolution 13  // don't change this value!   14bit is to slow!
#define OversampleScale 8  // LaserResolution - 10bit ADC = 3  ==> 2 Exp 3 = 8


//--- Timer 2 --------------------------
volatile boolean isTimerTick = false;
volatile unsigned long timerTick = 0UL;

unsigned long frameLenghtTimeMicros;
unsigned long exposureTime;
unsigned long scaledOversample;      //  Digits from ADC
unsigned long scaledExposure;        // Digits from ADC
float LKW_DistanceOversample;        // cm   Distance

unsigned long LKW_Start_Millis = 0UL;
unsigned long LKW_Stop_Millis = 0UL;
unsigned long LKW_MeasureTime_Millis;

boolean firstDetect = false;
boolean isDetect = false;
boolean gabDetect = false;
unsigned long laserGabTimeStart = 0UL;
unsigned long laserGabTimeStop = 0UL;
unsigned long laserGabCount = 0UL;
unsigned long laserScanCount = 0UL;


unsigned long startMicros = 0UL;       //### only test !
unsigned long stopMicros  = 0UL;       //### only test !


// === Setup =======================================================================
void setup() {
   Serial.begin(115200);         // baud rate
   pinMode(timer1Pin, OUTPUT);
   pinMode(detectPin, OUTPUT); 
   pinMode(duoLedGreen, OUTPUT);
   pinMode(duoLedRed, OUTPUT);   
   pinMode(swTest,INPUT);       // with external pullup resistor
   
   digitalWrite(duoLedGreen,HIGH);  // ready
   digitalWrite(duoLedRed, LOW); 

   frameLenghtTimeMicros = 1000000/ (unsigned long) FramesPerSecond;  // us
   scaledExposure = exposure->readDecimated() / OversampleScale;      // values from 0 . . 1023
  
   Timer1.initialize(frameLenghtTimeMicros);  
   Timer1.pwm(timer1Pin, analogRead(A1));  // setPwmDuty
   Timer1.stop();

   voltAdjust=voltPow / 1023.0;
   sampler = new Oversample(laserPin, LaserResolution);
   exposure =  new Oversample(exposurePin, LaserResolution);   

   MsTimer2::set(SendDataTimeRate, ticksSendData); 
   MsTimer2::stop();   
 
   scanEndLKW(); 
}
// === End of Setup ==========================================================

//--- Timer1 Interrupt Function, will be called every SendDataTimeRate ---
void ticksSendData(){
  isTimerTick = true;
  ++timerTick;
}

//========== Scan end of LKW =================================================
void scanEndLKW(){
  unsigned long scanStartTime = millis();  
  digitalWrite(duoLedGreen, HIGH);
  digitalWrite(duoLedRed, HIGH);

#ifdef SWITCH 
  digitalWrite(duoLedGreen, HIGH);
  digitalWrite(duoLedRed, LOW);
  return;
#endif

  while( scanStartTime < (scanStartTime + Delay_LKW_End_Millis)){        
      digitalWrite(duoLedGreen, !digitalRead(duoLedGreen));
      digitalWrite(duoLedRed, !digitalRead(duoLedRed));
      int  single =  analogRead(laserPin);             
      float voltageTrigger = single * voltAdjust;
      float LKW_DistanceTrigger = ( A * voltageTrigger )+ B;
          
      if (LKW_DistanceTrigger > (LKW_DistanceTriggerThreshold)){
           if( millis() > (scanStartTime + Delay_LKW_End_Millis) ){
                  break;  // exit while in the hard way  ;-) 
         }
      } // end if  
      else{
          scanStartTime = millis();  
      } // end else

    delay(25);
  } // end while

  digitalWrite(duoLedGreen, HIGH);
  digitalWrite(duoLedRed, LOW);
}


//=========== Test Switch ======================================
boolean detectTestSwitch(){ 
  boolean sw = !digitalRead(swTest); //or: digitalRead(swTest) ^1
  laserScanCount += 1;
  return sw;
}

//=========== Detect LKW ========================================
boolean laserDetectLKW(){
  scaledOversample =  sampler->readDecimated() / OversampleScale;
  float voltageOversample = scaledOversample * voltAdjust;
  LKW_DistanceOversample = ( A * voltageOversample )+ B;

  if (LKW_DistanceOversample < LKW_DistanceTriggerThreshold ){
      isDetect = true;
      gabDetect = false;
      laserScanCount += 1;
  }
  else{ // Not detect, reasons: End of LKW, Gab in LKW, bad LKW reflection on surface, ...
      isDetect = false;
      unsigned long minTimeMillis = LKW_Start_Millis + LaserDetect_LKW_Min_Time; 
     
      if( firstDetect && (millis() < minTimeMillis) ){   // check minimal detect time is set
         isDetect = true;
         return isDetect;  // ===> return : LaserDetect_LKW_Min_Time not reached
      } // end if
      else {  // Laser-GAB after LaserDetect_LKW_Min_Time
          if(firstDetect && !gabDetect){
             laserGabTimeStart = millis();
             gabDetect = true; 
             laserGabCount +=1;
             laserGabTimeStop = laserGabTimeStart + LaserGap_LKW_TimeOut;
          } // end if
  
          if(firstDetect && gabDetect && (millis() < laserGabTimeStop)){
             isDetect = true;
          } // end if
          else{  // laser gab timeout
              isDetect = false;
              gabDetect = false;
              firstDetect = false;            
          }  // end else laser gab timeout    
    }// end else Laser-GAB after LaserDetect_LKW_Min_Time
             
  } // end else Not detect

  return isDetect;
}

//========== detect LKW =====================================
boolean detectLKW(){
   isDetect = false;
 
  #ifdef SWITCH
      isDetect = detectTestSwitch();
  #endif
      
  #ifdef LASER
      isDetect = laserDetectLKW();
  #endif
       
  #ifdef LASER_AND_SWITCH
      if(detectTestSwitch() || laserDetectLKW()) isDetect = true;
  #endif
  
  //digitalWrite(duoLedGreen, !isDetect);
  //digitalWrite(duoLedRed, isDetect);

  return isDetect; 
}


// === Main =========================================================================
void loop(){
  unsigned long currMillis;
  
//  scaledExposure = exposure->readDecimated() / OversampleScale;
       
  if ( detectLKW() ){   // first detect LKW
      LKW_Start_Millis =  millis();

      digitalWrite(duoLedRed, HIGH);

      Serial.print( -1 );
      Serial.print("\t"); 
      Serial.println( -1 );    
      delay(WaitAfterDistanceTriggerBefore);
      
      digitalWrite(duoLedGreen, LOW);
     
      Timer1.pwm(timer1Pin, scaledExposure);  // setPwmDuty
      Timer1.start();
      
      timerTick = 0UL;
      MsTimer2::start();

      //############# detectLKW(): duration time = 1,15 ms ###############
      //     startMicros = micros();
      //     detectLKW();
      //     stopMicros = micros();
      //#################################################################        
      
      firstDetect = true;
      laserScanCount = 0UL;
      laserGabCount =0UL;
      laserGabTimeStart = 0UL;
      laserGabTimeStop = 0UL;
          
      while( detectLKW() ){  // detectLKW
             if( isTimerTick ){
                 isTimerTick = false;  
                 //--- Send Data --------------------------------
                 Serial.print( (timerTick-1) * SendDataTimeRate );
                 Serial.print("\t"); 
                 Serial.println( LKW_DistanceOversample );                                                                                         
              } // end if
             
              scaledExposure = exposure->readDecimated() / OversampleScale;   // values from 0...1023 
              exposureTime = (frameLenghtTimeMicros*scaledExposure) /1023;

              //===> the minimum exposure time:  10  ==> 1 % form  frameLenghtTimeMicros             
              if( scaledExposure < 10) {
                  scaledExposure = 10;
              }

              //===> the maximum exposure time: 1002 ==> 98 % form  frameLenghtTimeMicros
              if (scaledExposure > 1002) {
                 scaledExposure = 1002;
              }
              
              Timer1.pwm(timer1Pin, scaledExposure);  // setPwmDuty
                                     
      }//end while detectLKW

      LKW_Stop_Millis = millis();    
      Timer1.stop();
      Timer1.pwm(timer1Pin, 0);   // set Level to Zero !!!  
      
      MsTimer2::stop();
    
      LKW_MeasureTime_Millis = LKW_Stop_Millis - LKW_Start_Millis - LaserGap_LKW_TimeOut;    
      firstDetect = false;

      digitalWrite(duoLedGreen, HIGH);
      digitalWrite(duoLedRed, LOW);
      
      //===============================================================================
      //===> The End: Send other Info and End Of Data 
      Serial.println(frameLenghtTimeMicros);          // one frame time 
      Serial.println(exposureTime);                  // exposure time in us
      
      Serial.println(LKW_MeasureTime_Millis);  
      Serial.println(EOD);     
      //==============================================================================
      
      delay(WaitTimeForMatlap);

      //!!! WAIT, do nothing for Matlab but scan Laser and exit only, 
      //!!! if Laser see "NOTHING" because a second LKW or more LKW can drive in this time      
      scanEndLKW();    // check if another LKW is driving in between time (WaitTimeForMatlab)
    } //end if first detect LKW   
   
}//end Main
// === End of Main ===================================================================






