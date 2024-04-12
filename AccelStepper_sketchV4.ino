/* Telsa Coil Winder sketch to control a stepper motor with 
   DRV8825 stepper motor driver, AccelStepper library, 
   Arduino Nano, and Nextion Display with Nextion Library.

   Developer:     

   Github:  https://

   More info about AccelStepper can be found here: https://www.makerguides.com 
*/

#include "AccelStepper.h"
#include <Nextion.h>

// Setup debug displays.  Change to 0 to output debug messages.  1 to surpress debug messages.
#define DEBUG 0
#if DEBUG == 1 
#define debug(...) Serial.print(__VA_ARGS__)
#define debugln(...) Serial.println(__VA_ARGS__)
#else
#define debug(...)
#define debugln(...)
#endif

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2
#define stepPin 3
#define dirPin2 12
#define stepPin2 13
#define motorInterfaceType 1
#define limitSwitch 6
#define sleepPin 5

//All the variables**********************************************************************************************************
int counter=0;
uint32_t awgStep = 0; 
uint32_t turnsTotal = 0;
uint32_t coilLength = 0;
uint32_t offset = 0;
uint32_t stepby = 25;   // default number of steps

// Flags
bool started = false;
bool home = false;
bool stop = false;
bool offSetPlus = false;
bool offSetMinus = false;
bool pause = false;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);  // Coil motor
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);  // Awg Wire Motor

// Defining all the buttons and inputs and outputs on the nextion display 
// Make sure that your page and the Id match whats in Nextion.  If not you will get very strange results.  (page, id, name)

// Page 0
NexPage page0    = NexPage(0, 0, "page0");

// Page 1
NexButton bHome=NexButton(1, 2, "bHome"); // define home button
NexButton bOffSetPlus=NexButton(1, 3, "bOffSetPlus"); // define plus offset button
NexButton bOffSetMinus=NexButton(1, 4, "bOffSetMinus"); // define minus offset button
NexDSButton btStepby=NexDSButton(1, 7, "btStepby"); // define number of steps to move 

// Pag 2
NexButton bPlus10=NexButton(2, 6,"bPlus10");   // Length plus 10 button
NexButton bMinus10=NexButton(2, 7,"bMinus10"); // Length Minus 10 button
NexButton bPlus1=NexButton(2, 8,"bPlus1");     // Length plus 1 button
NexButton bMinus1=NexButton(2, 9,"bMinus1");   // Length Minus 1 button

NexButton b26=NexButton(2, 10,"b26");   // AWG Wire Button
NexButton b28=NexButton(2, 11,"b28");   // AWG Wire Button
NexButton b30=NexButton(2, 12,"b30");   // AWG Wire Button
NexButton b32=NexButton(2, 13,"b32");   // AWG Wire Button
NexButton b34=NexButton(2, 14,"b34");   // AWG Wire Button
NexButton b36=NexButton(2, 15,"b36");   // AWG Wire Button

// Page 3
NexNumber nCoil = NexNumber(3, 2, "nCoil");  // Number of coil turns 
NexNumber nAwg = NexNumber(3, 5, "nAwg");  // Awg wire size 
NexNumber nTotalTurns = NexNumber(3, 7, "nTotalTurns");   // total turns -- not used
NexNumber nCount = NexNumber(3, 9, "nCount");  // Number of active turns 
NexButton bBack=NexButton(3, 10,"bBack");   // Back Button
NexButton bStart=NexButton(3, 11, "bStart"); // define the start button on page 3 
NexText tStatus = NexText(3, 12, "tStatus");   // Status bar
NexDSButton btPause=NexDSButton(3, 13, "btPause"); // pause button -- dual state

// setup listen events
NexTouch *nex_listen_list[] = { 
    &bHome,
    &bOffSetPlus,
    &bOffSetMinus,
    &btStepby,
    &bPlus10,
    &bMinus10,
    &bPlus1,
    &bMinus1,
    &b26,
    &b28,
    &b30,
    &b32,
    &b34,
    &b36,
    &bBack,
    &bStart,
    &btPause,
    NULL
   };

// Setup all buttons
// Start button  ****************************************************************************
void bHomePopCallback(void *ptr) { 
  home=true;
  stop=false;
  digitalWrite(sleepPin, HIGH); //turning on the motors
}

// Offset buttons  ************************************************************************************
void bOffSetPlusPopCallback(void *ptr) { 
  offSetPlus = true;
  debugln("bOffSetPlus");

}
void bOffSetMinusPopCallback(void *ptr) { 
  offSetMinus = true;
  debugln("bOffSetMinus");
}

void btStepbyPopCallback(void *ptr){
  uint32_t dual_state = 0;
  btStepby.getValue(&dual_state);
  if(dual_state==1){
    stepby = 100;
  }
  else if(dual_state==0){
     stepby = 25;
  }
}

// Length Buttons   *******************************************************************************
void bPlus10PopCallback(void *ptr) { 
  debugln("+10");
  coilLength = coilLength+10; 
  debugln(coilLength,DEC);
}
void bMinus10PopCallback(void *ptr) { 
  debugln("-10");
  coilLength = coilLength-10; 
  debugln(coilLength,DEC);
}
void bPlus1PopCallback(void *ptr) { 
  debugln("+1");
  coilLength = coilLength+1; 
  debugln(coilLength,DEC);
}
void bMinus1PopCallback(void *ptr) { 
  debugln("-1");
  coilLength = coilLength-1;
  debugln(coilLength,DEC); 
}

// AWG Wire buttons   *************************************************************************
void b26PopCallback(void *ptr) { 
  debugln("26");
  awgStep = 50; 
}
void b28PopCallback(void *ptr) { 
  debugln("28");
  awgStep = 40; 
}
void b30PopCallback(void *ptr) { 
  debugln("30");
  awgStep = 29; 
}
void b32PopCallback(void *ptr) { 
  debugln("32");
  awgStep = 20;  
}
void b34PopCallback(void *ptr) { 
  debugln("34");
  awgStep = 15; 
}
void b36PopCallback(void *ptr) { 
  debugln("36");
  awgStep = 10; 
}

// Start button  ****************************************************************************
void bStartPopCallback(void *ptr) { 

  if (coilLength != 0 && awgStep !=0){
    debugln("Start Button pressed");
    turnsTotal=coilLength*1000/awgStep;

    debugln(turnsTotal,DEC);
    // did not work for some reason.
    //nTotalTurns.setValue(turnsTotal);
    started = true;

    //  Wake Motors up
    digitalWrite(sleepPin, HIGH);
  }else{
    tStatus.setText("Please enter coil and Awg");
  }
} 

// Back Button  ********************************************************************************
void bBackPopCallback(void *ptr) { 

  debugln("bBack");
  counter=0;
  coilLength=0;
  awgStep=0;
  turnsTotal=0;
  nCoil.setValue(0);
  nCount.setValue(0);
  nAwg.setValue(0);
  nTotalTurns.setValue(0);
}

// Pause Button  ********************************************************************************
void btPausePopCallback(void *ptr){
	
  uint32_t dual_state = 0;
  btPause.getValue(&dual_state);
  if(dual_state==1){
    digitalWrite(sleepPin, LOW);
    tStatus.setText("Paused");
    pause=true;
  }
  else if(dual_state==0){
    digitalWrite(sleepPin, HIGH);
    tStatus.setText("Runnning");
    pause=false;
  }
}

// Setup   *******************************************************************************************************
void setup() {
  //Serial.begin(9600);
  Serial.begin(115200);

  // setup nexion display
  nexInit();

  // setup call back functions  ****************************************************
  bHome.attachPop(bHomePopCallback, &bHome);   // Home button
  bOffSetPlus.attachPop(bOffSetPlusPopCallback, &bOffSetPlus);     // Offset from endstop buttons
  bOffSetMinus.attachPop(bOffSetMinusPopCallback, &bOffSetMinus);  // Offset from endstop buttons
  btStepby.attachPop(btStepbyPopCallback, &btStepby);  // Step by from the endstop 
  bPlus10.attachPop(bPlus10PopCallback, &bPlus10);    // Length plus 10 button
  bMinus10.attachPop(bMinus10PopCallback, &bMinus10); // Length Minus 10 button
  bPlus1.attachPop(bPlus1PopCallback, &bPlus1);       // Length plus 1 button
  bMinus1.attachPop(bMinus1PopCallback, &bMinus1);    // Length Minus 1 button
  b26.attachPop(b26PopCallback, &b26);  // AWG Wire Buttons
  b28.attachPop(b28PopCallback, &b28);  // AWG Wire Buttons
  b30.attachPop(b30PopCallback, &b30);  // AWG Wire Buttons
  b32.attachPop(b32PopCallback, &b32);  // AWG Wire Buttons
  b34.attachPop(b34PopCallback, &b34);  // AWG Wire Buttons
  b36.attachPop(b36PopCallback, &b36);  // AWG Wire Buttons
  bBack.attachPop(bBackPopCallback, &bBack);   // Back button
  bStart.attachPop(bStartPopCallback, &bStart);   // Start button
  btPause.attachPop(btPausePopCallback, &btPause);   // Pause button

  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);

  // Must have a PULLUP on the limitswitch  INPUT will not work with a V-156-1C25 Micro Switch - Normally Opened (NO)
  pinMode(limitSwitch, INPUT_PULLUP);

  // Put all motors to sleep   -- Not used at this time.
  pinMode(sleepPin, OUTPUT);

  digitalWrite(sleepPin, LOW);

}
// Main processing loop.  *****************************************************************************************************
void loop() { 

  // nextion event loop
  nexLoop(nex_listen_list);

  // Limit switch loop.   Move motor until it hits the limit switch.  After contact back off limit switch.
  // Limit swich is set to high as default
  if(home == true && stop == false){
    // Set the current position to 0:
    stepper2.setCurrentPosition(0);
    while(stepper2.currentPosition() != -800){
      if(digitalRead(limitSwitch) == LOW){
        debugln("Found Low");
        stepper.setCurrentPosition(0);
        while(stepper2.currentPosition() != 100){
          stepper2.setSpeed(200);
          stepper2.runSpeed();
        }
        debugln("started");
        //started=true;
        stop=true;
        delay(1000);
        break;
      }else{ 
        // Limit switch is high until presssed
        //debugln("Found High");
        stepper2.setSpeed(-200);
        stepper2.runSpeed();
      }
    }
  }

  // set start location.
  if(offSetPlus == true && home == true && stop == true){
    stepper2.setCurrentPosition(0);
    while(stepper2.currentPosition() != stepby){
      stepper2.setSpeed(200);
      stepper2.runSpeed();
    }
    offSetPlus = false;
  }

  // set start location
  if(offSetMinus == true && home == true && stop == true){
    stepper2.setCurrentPosition(0);
    while(stepper2.currentPosition() != -stepby){
      stepper2.setSpeed(-200);
      stepper2.runSpeed();
    }
    offSetMinus = false;
  }

  // Main servo motor loop.
  if(started == true && pause == false){
    if(turnsTotal > counter) { 
      
      // Set the current position to 0:
      stepper.setCurrentPosition(0);

      // Run the motor forward at 200 steps/second until the motor reaches 600 steps (1 revolution - 3:1 ratio):
      while(stepper.currentPosition() != 600)
      {
        stepper.setSpeed(200);
        stepper.runSpeed();
      }
      counter++;
      nCount.setValue(counter);
      delay(200);
      // Reset the position to 0:
      stepper2.setCurrentPosition(0);

      // Run the motor forwards at awgStep steps/second until the motor reaches awgStep steps:
      while(stepper2.currentPosition() != awgStep) 
      {
        stepper2.setSpeed(200);
        stepper2.runSpeed();
      }
      delay(200);
    }else{
      tStatus.setText("Complete");
      delay(10000);

      // reset all variable to their initial state.  Needed for the next run if not powered off.
      started=false;
      stop=false;
      home=false;
      counter=0;
      coilLength=0;
      awgStep=0;
      turnsTotal=0;
      nCoil.setValue(0);
      nCount.setValue(0);
      nAwg.setValue(0);
      nTotalTurns.setValue(0);

      // Put Motors to sleep
      digitalWrite(sleepPin, LOW);

      // Reset page to page 0 for the next run
      tStatus.setText("Ready");
      page0.show();
    }
  }
}
