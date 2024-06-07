/* Telsa Coil Winder sketch to control a stepper motor with 
   DRV8825 stepper motor driver, AccelStepper library, 
   Arduino Nano, and Nextion Display with Nextion Library.

   Developer:     Jense Duart

   Github:  https://github.com/rduart/TelsaCoilWinder.git


   More info about AccelStepper can be found here: https://www.makerguides.com  

   Wire Diameter:  https://www.solaris-shop.com/content/American%20Wire%20Gauge%20Conductor%20Size%20Table.pdf
*/

/* This Telas Coil winder was created to match the output from JavaCT.   For the most part, the winder creates 1 extra coil then what JavaTC
   calculates.  For example:  Spec 140mm coil with AWG 34 =  JavaCT= 873, Winder=874.    This is due to rounding.  The winder is always 
   rounds up to the next whole number.

   The winder carriage is set to 1/4 step currently.   It works find with wire gauages we are currently using.   For wire under 36 gauage 
   consider changing 1/8 to smaller steps.   The step resolution may not be small enough otherwise.

   8mm / 800 steps = .01 mm/step     -- Lead Screw has 4 starts

   After much testing. I discovered I am not able to wind the coil based on the wire size alone.  There is always very small gaps, so I added 
   an difference offset to account for the extra spaces the exist between the wire coils.

   I determined this by measuring the coil after it was created.   For example: AWG 34 wire, 100MM coil created a 114MM coil. 114mm / totalTurns = offset | offset - wire dia = diffoffset

   The coil winder is set to full step.  200 steps per revolution.

   The nextion libray is has several issue.  Once in a while you need to either get or set the same atttribute twice or more to either update the 
   nextion display or get the value from Nextion display.   I am not sure why.  
*/

#include "AccelStepper.h"
#include <Nextion.h>
#include <math.h>

// Setup debug displays.  Change to 0 to output debug messages.  1 to surpress debug messages.
#define DEBUG 1
#if DEBUG == 0 
#define debug(...) Serial.print(__VA_ARGS__)
#define debugln(...) Serial.println(__VA_ARGS__)
#else
#define debug(...)
#define debugln(...)
#endif

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2               // Winder motor
#define stepPin 3              // Winder motor
#define dirPin2 12             // Wire motpr
#define stepPin2 13            // Wire motor
#define motorInterfaceType 1
#define limitSwitch 6          // Limit switch on wire motor
#define sleepPin 5             // used to put both motors to sleep.  Not be used with current motor controllers- No sleep pin availabe.

//All the variables**********************************************************************************************************
int counter = 0;
int countLead = 0;
uint32_t awgStep = 0;       // Number of steps needed of each wire diameter.
uint32_t turnsTotal = 0;    // Total calculated number of turns.
uint32_t coilLength = 0;    // Overall coil length.
uint32_t stepby = 25;       // default number of steps
float motorSpeed = 200;       // default number of steps
float wiredia = 0;          // Diameter of the wire.  
float estConstant = 17.42;  // Used to calculate estimated time.  If you change the motor speed this value needs to be reset. 
uint32_t estTime = 0;       // Estimated time to complete the winding of the coil.
uint32_t lead = 0;          // How many coil to make before starting wire carriage -- determines how tight the coils by delaying the carriage.

// Flags
bool started = false;        //  Flag that is set by the start button.
bool home = false;           // Flag the is set by the home button.
bool stop = false;           // Flag set at end to stop the winding from winding more coils
bool offSetPlus = false;     //  Flag used to trigger a plus offset from home
bool offSetMinus = false;    //  Flag used to trigger a minus offset from home
bool pause = false;          // Flag that Causes the main loop to pause during winding the coil
bool varnish = false;        // flag the causes the winding motor to turn until button state is changed

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);  // Coil motor
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);  // Awg Wire Motor

// Defining all the buttons and inputs and outputs on the nextion display 
// Make sure that your page and the Id match whats in Nextion.  If not you will get very strange results.  (page, id, name)

// Page 0
NexPage page0    = NexPage(0, 0, "page0");   // Application start page.

// Page 1
NexButton bHome=NexButton(1, 2, "bHome"); // define home button
NexButton bOffSetPlus=NexButton(1, 3, "bOffSetPlus"); // define plus offset button
NexButton bOffSetMinus=NexButton(1, 4, "bOffSetMinus"); // define minus offset button
NexDSButton btStepby=NexDSButton(1, 7, "btStepby"); // define number of steps to move 
NexDSButton btVarnish=NexDSButton(1, 8, "btVarnish"); // varnish button -- dual state

// Page 2
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
NexButton b2Next=NexButton(2, 17,"b2Next");   // Next page button

// Page 3
NexNumber nCoil = NexNumber(3, 2, "nCoil");  // Number of coil turns 
NexNumber nAwg = NexNumber(3, 5, "nAwg");  // Awg wire size 
//NexNumber nTotalTurns = NexNumber(3, 7, "nTotalTurns");   // total turns -- not used
NexNumber nCount = NexNumber(3, 8, "nCount");  // Number of active turns 
NexButton bBack=NexButton(3, 9,"bBack");   // Back Button
NexButton bStart=NexButton(3, 10, "bStart"); // define the start button on page 3 
NexText tStatus = NexText(3, 11, "tStatus");   // Status bar
NexDSButton btPause=NexDSButton(3, 12, "btPause"); // pause button -- dual state
NexText tnTotalTurns = NexText(3, 13, "tnTotalTurns");   // total turns
NexText tnEstTime = NexText(3, 15, "tnEstTime");  // estimated time in whole hours:minutes.

// setup listen events
NexTouch *nex_listen_list[] = { 
    &bHome,
    &bOffSetPlus,
    &bOffSetMinus,
    &btStepby,
    &btVarnish,
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
    &b2Next,
    &bBack,
    &bStart,
    &btPause,
    NULL
   };

// Setup all buttons
//Page 0
// Start button  ****************************************************************************
void bHomePopCallback(void *ptr) { 
  debugln("bHome");
  home=true;
  stop=false;
  digitalWrite(sleepPin, HIGH); //turning on the motors
}

// Page 1
// Offset buttons  ************************************************************************************
void bOffSetPlusPopCallback(void *ptr) { 
  offSetPlus = true;
  debugln("bOffSetPlus");
}
void bOffSetMinusPopCallback(void *ptr) { 
  offSetMinus = true;
  debugln("bOffSetMinus");
}

// Step by button
void btStepbyPopCallback(void *ptr){
  uint32_t dual_state = 0;
  // Need to get it twice.  Why I do not know. Bug or timing issue.  I am not sure.
  btStepby.getValue(&dual_state);
  btStepby.getValue(&dual_state);
  if(dual_state==1){
    stepby = 500;
  }
  else if(dual_state==0){
     stepby = 25;
  }
}

// varnish button
void btVarnishPopCallback(void *ptr){
  debugln("bVarnish");

  digitalWrite(sleepPin, HIGH);

  uint32_t dual_state = 0;
  // Need to get it twice.  Why I do not know. Bug or timing issue.  I am not sure.
  btVarnish.getValue(&dual_state);
  btVarnish.getValue(&dual_state);
  debugln(dual_state,DEC);
  if(dual_state==1){
    varnish = true;
  }
  else if(dual_state==0){
     varnish = false;
  }
}

// Page 2
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
// 8mm / 800 steps = .01 mm/step     -- Lead Screw has 4 starts   --  enamel Wire coating is like .0285 mm
// After much testing. I discovered I am not able to wind the coil based on the wire size alone.  There is always very small gaps, so I added an offet to account for the extra space.
// I determined this simple by measuring the coil after it was created.   For example: AWG 34 wire, 100MM coil created a 114MM coil. 114mm / totalTurns = offset | offset - wire dia = diffoffset

//                                   Wire Size calculation     Actual from measurement        Difference - Offset
void b26PopCallback(void *ptr) {  // .40386 / .01 = 40.386  |  
  debugln("26");
  //awgStep = 50;  // orig
  //awgStep = 47;
  awgStep = 43;
  lead = 3;   
  wiredia=.4334;  // Wire Diameter  .4334 .4049 + .0285 = .4334  // Thanks to carl1961
}
//                                   Wire Size calculation     Actual from measurement        Difference - Offset
void b28PopCallback(void *ptr) { // .32004 / .01 = 32.004   |  actual: .3429 / .01 = 34.29  | .3429 - .32004 = .0229
  debugln("28");
  //awgStep = 40;  // orig
  //awgStep = 32;
  awgStep = 34;
  lead = 5;  
  wiredia=.32004 + .0229;  // Wire Diameter
}
//                                   Wire Size calculation     Actual from measurement        Difference - Offset
void b30PopCallback(void *ptr) { // .254 / .01 = 25.4       |        
  debugln("30");
  //awgStep = 29;  // orig
  //awgStep = 27;
  awgStep = 25;
  lead = 5;
  wiredia=.254 + .0285;  // Wire Diameter
}
//                                   Wire Size calculation       Actual from measurement        Difference - Offset
void b32PopCallback(void *ptr) {  // Wire: .2032 / .01 = 20.32 | actual: .2236 / .01 = 22.36 | .2236 - .2032 = .0204
  debugln("32");
  //awgStep = 20;  // Wire Size  - 20.32
  awgStep = 22;   // Actual measured.  - 22.36
  lead = 10;      // Number of turn before carriage start moving.
  wiredia=.2032 + .0204;  // Wire Diameter + diff offset
}
//                                   Wire Size calculation          Actual from measurement        Difference - Offset
void b34PopCallback(void *ptr) {  // wire: .16002 / .01 = 16.002  | actual: .1829 / .01 = 18.29  | .1826 - .16002 = .02288
  debugln("34");
  //awgStep = 15;   // orig
  //awgStep = 16;   // Wire Size - 16.002
  awgStep = 18;     // Actual measured  - 18.29
  lead = 10;        // Number of turn before carriage start moving.
  wiredia=.16002 + .02288;  // Wire Diameter + diff offset
}
//                                   Wire Size calculation    Actual from measurement        Difference - Offset  
void b36PopCallback(void *ptr) {  // .127 / .01 = 12.7      | actual .14104 / .01 = 14.10   | .14104 - .127 = .01404
  debugln("36");
  //awgStep = 10; // orig
  //awgStep = 13; // wire size  12.7
  awgStep = 14;   // Actual measured  - 14.10
  lead = 12;      // Number of turn before carriage start moving.
  wiredia=.127 + .01404;  // Wire Diameter + offset
}
// Next button - navigate to page 3
void b2NextPopCallback(void *ptr) { 
  debugln("b2Next");
  if (coilLength != 0 && awgStep !=0){
    turnsTotal=(int((coilLength/wiredia) +.5)); // int() cast truncates (aka "rounds down") + .5 make sure 10.5 is a 11 while 10.3 is still 10.
    debugln(turnsTotal,DEC);
  
    // Having trouble sending numbers back to Nextion.  Converting to text appears to help.
    char numberArray[20];
    itoa(turnsTotal,numberArray,10);
    // Need to set twice.   
    tnTotalTurns.setText(numberArray);
    tnTotalTurns.setText(numberArray);

    // Cacluate the estimated time in minutes.  Seconds are being ignored, so it could be 1 to 59 second off from the actual time.
    estTime = turnsTotal / estConstant;     // Calculate the number of whole minutes -  Should be within a minute.
    debugln(estTime,DEC);
    
    String stringOne;
    char timeArray[10];

    // Convert minutes in estTime to hours and minutes.  Seconds will alaways be 00.  Its an estimate does not need to be exact.
    if (estTime > 60){
      float aTime = estTime/60.0;
      debugln(aTime,DEC);
      uint32_t bTime =(int(estTime/60));
      debugln(bTime,DEC);
      float cTime = aTime-bTime;
      debugln(cTime,DEC);
      //float dTime = cTime*60;
      uint32_t dTime =(int(cTime*60));
      debugln(dTime,DEC);

      // Format for Nextion display  -- Larger then 60 minutes
      stringOne.concat(bTime);
      stringOne.concat(":");
      if (dTime < 10){
        stringOne.concat("0");
      }
      stringOne.concat(dTime);
      debugln(stringOne);
      stringOne.concat(":00");
    }else{
      // Format for Nextion display  -- Less then 60 minutes
      stringOne.concat("00:");
      stringOne.concat(estTime);
      stringOne.concat(":00");
    }
    stringOne.toCharArray(timeArray, 10);
    tnEstTime.setText(timeArray);
    tnEstTime.setText(timeArray);
  }else{
    tStatus.setText("Please enter coil size MM and Awg");
  }
}

// Page 3
// Start button  ****************************************************************************
void bStartPopCallback(void *ptr) { 
  if (coilLength != 0 && awgStep !=0){
    debugln("Start Button pressed");
    started = true;

    //  Wake Motors up
    digitalWrite(sleepPin, HIGH);
  }else{
    tStatus.setText("Please enter coil size MM and Awg");
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
  tnEstTime.setText("00:00");
  tnTotalTurns.setText("0");
}
// Pause Button  ********************************************************************************
void btPausePopCallback(void *ptr){
  debugln("btPause");
  uint32_t dual_state = 0;
  // need to do get twice
  btPause.getValue(&dual_state);
  btPause.getValue(&dual_state);
  debugln(dual_state,DEC);
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

  // Discovered that you need to send the values back to Nextion more then once to get them to display on Nextion.
  // As such, doing first sets in setup.
  tnTotalTurns.setText("0");
  nCount.setValue(0);

  // setup call back functions  ****************************************************
  bHome.attachPop(bHomePopCallback, &bHome);   // Home button
  bOffSetPlus.attachPop(bOffSetPlusPopCallback, &bOffSetPlus);     // Offset from endstop buttons
  bOffSetMinus.attachPop(bOffSetMinusPopCallback, &bOffSetMinus);  // Offset from endstop buttons
  btStepby.attachPop(btStepbyPopCallback, &btStepby);  // Step by from the endstop 
  btVarnish.attachPop(btVarnishPopCallback, &btVarnish);  // Varnish Button
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
  b2Next.attachPop(b2NextPopCallback, &b2Next);  // Next Button
  bBack.attachPop(bBackPopCallback, &bBack);   // Back button
  bStart.attachPop(bStartPopCallback, &bStart);   // Start button
  btPause.attachPop(btPausePopCallback, &btPause);   // Pause button

  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);

  // Must have a PULLUP on the limitswitch  Will not work without INPUT_PULLUP for a V-156-1C25 Micro Switch - Normally Opened (NO)
  pinMode(limitSwitch, INPUT_PULLUP);

  // Put all motors to sleep   -- Not used at this time.
  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, LOW);

}
// Main processing loop.  *****************************************************************************************************
void loop() { 

  /*
   * Nextion event loop
   * When a pop or push event occured every time,
   * the corresponding component[right page id and component id] in touch event list will be asked.
   */
  nexLoop(nex_listen_list);

  // Limit switch loop.   Move motor until it hits the limit switch.  After contact back off limit switch.
  // Limit swich is set to high by default
  if(home == true && stop == false){
    // Set the current position to 0:
    stepper2.setCurrentPosition(0);
    while(stepper2.currentPosition() != -800){  // full length of the track is about 40,000 steps.
      if(digitalRead(limitSwitch) == LOW){
        debugln("Found Low");
        stepper2.setCurrentPosition(0);
        while(stepper2.currentPosition() != 75){
          stepper2.setSpeed(400);
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
        stepper2.setSpeed(-400);
        stepper2.runSpeed();
      }
    }
  }

  // set start location.  jogs the wire motor to the right away from the limit switch
  if(offSetPlus == true && home == true && stop == true){
    stepper2.setCurrentPosition(0);
    while(stepper2.currentPosition() != stepby){
      stepper2.setSpeed(400);
      stepper2.runSpeed();
    }
    offSetPlus = false;
  }

  // set start location.  jogs the wire motor to the left in the direction of the limit switch
  if(offSetMinus == true && home == true && stop == true){
    stepper2.setCurrentPosition(0);
    while(stepper2.currentPosition() != -stepby){
      stepper2.setSpeed(-400);
      stepper2.runSpeed();
    }
    offSetMinus = false;
  }

  // varnish function turns stepper motor until the dual state button is set varnish to false.
  if(varnish == true && home == true && stop == true){
    stepper.setCurrentPosition(0);
    while(stepper.currentPosition() != -600){
      stepper.setSpeed(-400);
      stepper.runSpeed();
    }
  }

  // Main servo motor loop.   This section is responsibile for the winding of the coil
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
      // need to set twice
      nCount.setValue(counter);
      nCount.setValue(counter);
      delay(200);
      if(lead == countLead) {   // make some turns prior to the carriage starting.  Make a tighter coil.
        // Reset the position to 0:
        stepper2.setCurrentPosition(0);

        // Run the motor forwards at 200 steps/second until the motor reaches awgStep steps:
        while(stepper2.currentPosition() != awgStep) 
        {
          stepper2.setSpeed(200);
          stepper2.runSpeed();
        }
      }else{
        countLead++;
      }
      delay(200);
    }else{
      // Job is complete
      tStatus.setText("Complete");
      delay(10000);

      // reset all variables to their initial state.  Needed for the next run if not powered off.
      started=false;
      stop=false;
      home=false;
      counter=0;
      countLead=0;
      coilLength=0;
      awgStep=0;
      turnsTotal=0;
      nCoil.setValue(0);
      nCount.setValue(0);
      nAwg.setValue(0);
      tnTotalTurns.setText("0");
      tnEstTime.setText("00:00");

      // Put Motors to sleep
      digitalWrite(sleepPin, LOW);

      // Reset page to page 0 for the next run
      tStatus.setText("Ready");
      page0.show();
    }
  }
}
