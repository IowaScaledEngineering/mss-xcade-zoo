/*************************************************************************
Title:    MSS-XCADE CP Anders for Rick Simpson
Authors:  Nathan D. Holmes <maverick@drgw.net>
          Iowa Scaled Engineering
File:     mss-xcade-cp-anders.ino
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2025 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/
#include "Wire.h"
#include "mss-xcade.h"

WireMux wireMux;
XCade xcade;
XCade xcadeExpander1;

SignalMast signalMast1A;
SignalMast signalMast1B;
SignalMast signalMast1C;
SignalMast signalMast1D;
SignalMast signalMast2A;
SignalMast signalMast2B;

#define LOOP_UPDATE_TIME_MS 50
#define DEBUG_UPDATE_TIME_MS 1000
void setup() 
{
  Serial.begin(115200);
  Serial.println("Startup");

  // Start the I2C/Wire interface.  This is how the host processor talks to everything on the xcade board.
  //  Also must be started before setting up the WireMux object - described below - because the mux is also
  //  configured over I2C
  Wire.setPins(XCADE_I2C_SDA, XCADE_I2C_SCL);
  Wire.setClock(100000);
  Wire.begin();

  // The master MSS-XCADE has an I2C/Wire switch on it to allow it to access up to three
  //  additional expansion boards.  Because this exists in sort of a "no man's land" between
  //  the host processor and the xcade peripherals that exist on each and every board, it
  //  becomes it's own object.  By passing that to the xcade, it can then switch the mux to talk
  //  to the correct board

  wireMux.begin(&Wire);

  // Start up the xcade board.  
  //  This is the master board, on the same PCB as the host processor.  If you had additional xcade boards, you would
  //  start them up afterwards using additional xcade objects and their port number, like xcade_expansion1.begin(&wireMux, 1);

  xcade.begin(&wireMux);
  xcadeExpander1.begin(&wireMux, 1);

  // Configure the signal heads.  If you're already writing your own code, you probably already know what type of
  //  signals you plan to connect.  This is really only needed if you plan to do something other than use the default
  //  setting for 3-light heads and using the board common anode / cathode switch to drive the head polarity.

  // Create "signal masts" - this allows us to put multiple heads together and then feed in an indication, and let the internal
  //  signal rules drive the aspects based on a 1, 2, or 3-headed signal.  The rules can be overriden if you want different 
  //  aspects shown for a given condition.  However, that's largely an advanced use case, so we'll leave that alone for now.

  // Signal A & D will be single-headed signals, since there's only one path through the interlocking for those
  // Signal B & C will be double-headed signals that can display both a normal or a diverging route
  signalMast1A.addSignalHeads(&xcade.signals.A1, &xcade.signals.A2);
  signalMast1B.addSignalHeads(&xcade.signals.B1, &xcade.signals.B2);
  signalMast1C.addSignalHeads(&xcade.signals.C1, &xcade.signals.C2);
  signalMast1D.addSignalHeads(&xcade.signals.D1, &xcade.signals.D2);

  signalMast2A.addSignalHeads(&xcadeExpander1.signals.A1, &xcadeExpander1.signals.A2);
  signalMast2B.addSignalHeads(&xcadeExpander1.signals.B1, &xcadeExpander1.signals.B2);

  enableLoopWDT();
}

void loop() 
{
  uint32_t currentTime = millis();
  static uint32_t lastReadTime = 0;
  static uint32_t debugPrintfTime = 0;

	// Because debouncing needs some time between samples, don't go for a hideous update rate
  // 50mS or so between samples does nicely.  That gives a 200mS buffer for changes, which is more
  // than enough for propagation delay
	if (!(((uint32_t)currentTime - lastReadTime) > LOOP_UPDATE_TIME_MS))
    return;


  // Update the last time we ran through the loop to the current time
  lastReadTime = currentTime;


  // Just blink the RGB LED once a second in a nice dim of blue, so that we know the board is alive
  rgbLedWrite(XCADE_RGB_LED, 0, 0, ((currentTime % 1000) > 500)?16:0);

  // First, read the input state from the hardware
  xcade.updateInputs();
  xcadeExpander1.updateInputs();

  // I'm not going to ASCII-ize the track diagram - just see the png
  // Read sensors
  bool turnout1APThrown = xcade.gpio.digitalRead(1);
  bool turnout1BPXoverThrown = xcade.gpio.digitalRead(2);
  bool turnout1CPXoverThrown = xcade.gpio.digitalRead(3);
  bool turnout1D2BThrown = xcade.gpio.digitalRead(4);
  bool turnoutP2AXoverThrown = xcade.gpio.digitalRead(5);
  bool turnoutP2BXoverThrown = xcade.gpio.digitalRead(6);

  bool block1AOccupancy = xcade.gpio.digitalRead(SENSOR_1_PIN);
  bool block1BOccupancy = xcade.gpio.digitalRead(SENSOR_3_PIN);
  bool block1COccupancy = xcade.gpio.digitalRead(SENSOR_5_PIN);
  bool block1DOccupancy = xcade.gpio.digitalRead(SENSOR_6_PIN);
  bool blockPOccupancy = xcade.gpio.digitalRead(SENSOR_9_PIN);
  bool block2AOccupancy = xcadeExpander1.gpio.digitalRead(SENSOR_1_PIN);
  bool block2BOccupancy = xcadeExpander1.gpio.digitalRead(SENSOR_3_PIN);

  bool irPCOccupancy = xcade.gpio.digitalRead(SENSOR_2_PIN);
  bool ir1COccupancy = xcade.gpio.digitalRead(SENSOR_4_PIN);
  bool ir2AOccupancy = xcadeExpander1.gpio.digitalRead(SENSOR_2_PIN);
  bool ir2BOccupancy = xcadeExpander1.gpio.digitalRead(SENSOR_4_PIN);

  // Start with all signals and ports at stop.  All routes not valid are invalid
  signalMast1A.setIndication(INDICATION_STOP);
  signalMast1B.setIndication(INDICATION_STOP);
  signalMast1C.setIndication(INDICATION_STOP);
  signalMast1D.setIndication(INDICATION_STOP);
  signalMast2A.setIndication(INDICATION_STOP);
  signalMast2B.setIndication(INDICATION_STOP);

  xcade.mssPortA.cascadeFromIndication(INDICATION_STOP);  
  xcade.mssPortB.cascadeFromIndication(INDICATION_STOP);  
  xcade.mssPortC.cascadeFromIndication(INDICATION_STOP);  
  xcade.mssPortD.cascadeFromIndication(INDICATION_STOP);  
  xcadeExpander1.mssPortA.cascadeFromIndication(INDICATION_STOP);  
  xcadeExpander1.mssPortB.cascadeFromIndication(INDICATION_STOP);  

  // Since what's below only sets occupancy for workable routes, we still need to provide occupancy for 
  //  the non-viable ones as well.  So we need to at least set the occupancy for each port based on 
  //  detectors we know are associated with it.

  xcade.mssPortA.setLocalOccupancy(block1AOccupancy);
  xcade.mssPortB.setLocalOccupancy(block1BOccupancy);
  xcade.mssPortC.setLocalOccupancy(block1COccupancy || ir1COccupancy);
  xcade.mssPortD.setLocalOccupancy(block1DOccupancy);

  xcadeExpander1.mssPortA.setLocalOccupancy(block2AOccupancy || ir2AOccupancy);
  xcadeExpander1.mssPortB.setLocalOccupancy(block2BOccupancy || ir2BOccupancy);

  // Now, work through all tracks on the left, one by one, and connect up all valid routes
  // In there, if a route is valid, it should set it in both directions.  

  // Port 1A Valid Routes
  //  - 1A->2A
  //  - 1A->2B

  if (turnout1APThrown && !turnout1BPXoverThrown && !turnoutP2AXoverThrown)
  {
    // Route is 1A to 2A
    xcade.mssPortA.setLocalOccupancy(block1AOccupancy || blockPOccupancy || irPCOccupancy || ir2AOccupancy);      
    xcadeExpander1.mssPortA.setLocalOccupancy(blockPOccupancy || irPCOccupancy || ir2AOccupancy || block2AOccupancy);

    xcade.mssPortA.cascadeFromPort(xcadeExpander1.mssPortA, false);
    xcadeExpander1.mssPortA.cascadeFromPort(xcade.mssPortA, true);

    signalMast1A.setIndication(xcadeExpander1.mssPortA);
    signalMast2A.setIndication(xcade.mssPortA, DIVERGING_SLOW_SPEED);
  }
  else if (turnout1APThrown && !turnout1BPXoverThrown && turnoutP2AXoverThrown && turnoutP2BXoverThrown)
  {
    // Route is 1A to 2B
    xcade.mssPortA.setLocalOccupancy(block1AOccupancy || blockPOccupancy || irPCOccupancy || ir2BOccupancy);      
    xcadeExpander1.mssPortB.setLocalOccupancy(blockPOccupancy || irPCOccupancy || ir2BOccupancy || block2BOccupancy);

    xcade.mssPortA.cascadeFromPort(xcadeExpander1.mssPortB, true);
    xcadeExpander1.mssPortB.cascadeFromPort(xcade.mssPortA, true);

    signalMast1A.setIndication(xcadeExpander1.mssPortB, DIVERGING_MEDIUM_SPEED);
    signalMast2B.setIndication(xcade.mssPortA, DIVERGING_SLOW_SPEED);
  } 

  // Port 1B Valid Routes
  //  - 1B->2A
  //  - 1B->2B

  if (!turnout1APThrown && !turnout1BPXoverThrown && !turnoutP2AXoverThrown)
  {
    // Route is 1B to 2A
    xcade.mssPortB.setLocalOccupancy(block1BOccupancy || blockPOccupancy || irPCOccupancy || ir2AOccupancy);      
    xcadeExpander1.mssPortA.setLocalOccupancy(blockPOccupancy || irPCOccupancy || ir2AOccupancy || block2AOccupancy);

    xcade.mssPortB.cascadeFromPort(xcadeExpander1.mssPortA, false);
    xcadeExpander1.mssPortA.cascadeFromPort(xcade.mssPortB, false);

    signalMast1B.setIndication(xcadeExpander1.mssPortA);
    signalMast2A.setIndication(xcade.mssPortB);
  }
  else if (!turnout1APThrown && !turnout1BPXoverThrown && turnoutP2AXoverThrown && turnoutP2BXoverThrown)
  {
    // Route is 1B to 2B
    xcade.mssPortB.setLocalOccupancy(block1BOccupancy || blockPOccupancy || irPCOccupancy || ir2BOccupancy);      
    xcadeExpander1.mssPortB.setLocalOccupancy(blockPOccupancy || irPCOccupancy || ir2BOccupancy || block2BOccupancy);

    xcade.mssPortB.cascadeFromPort(xcadeExpander1.mssPortB, true);
    xcadeExpander1.mssPortB.cascadeFromPort(xcade.mssPortB, true);

    signalMast1B.setIndication(xcadeExpander1.mssPortB, DIVERGING_FULL_SPEED);
    signalMast2B.setIndication(xcade.mssPortB, DIVERGING_FULL_SPEED);
  }

  // 1C valid routes:
  //  - 1C to 2A via the crossover
  //  - 1C to 2B via both crossover legs
  //  - 1C to 2B straight through
  if (turnout1CPXoverThrown && turnout1BPXoverThrown && !turnoutP2AXoverThrown)
  {
    // Route is 1C to 2A
      xcade.mssPortC.setLocalOccupancy(block1COccupancy || blockPOccupancy || ir2AOccupancy || ir1COccupancy);      
      xcadeExpander1.mssPortA.setLocalOccupancy(block2AOccupancy || blockPOccupancy || ir2AOccupancy || ir1COccupancy);

      xcade.mssPortC.cascadeFromPort(xcadeExpander1.mssPortA, true);
      xcadeExpander1.mssPortA.cascadeFromPort(xcade.mssPortC, true);

      signalMast1C.setIndication(xcadeExpander1.mssPortA, DIVERGING_FULL_SPEED);
      signalMast2A.setIndication(xcade.mssPortC, DIVERGING_FULL_SPEED);
  } 
  else if (turnout1CPXoverThrown && turnout1BPXoverThrown && turnoutP2AXoverThrown && turnoutP2BXoverThrown)
  {
    // Route is the dumb route - 1C to 2B via both crossover legs
      xcade.mssPortC.setLocalOccupancy(block1COccupancy || blockPOccupancy || ir2BOccupancy || ir1COccupancy);      
      xcadeExpander1.mssPortB.setLocalOccupancy(block2BOccupancy || blockPOccupancy || ir2BOccupancy || ir1COccupancy);

      xcade.mssPortC.cascadeFromPort(xcadeExpander1.mssPortB, true);
      xcadeExpander1.mssPortB.cascadeFromPort(xcade.mssPortC, true);

      signalMast1C.setIndication(xcadeExpander1.mssPortB, DIVERGING_MEDIUM_SPEED);
      signalMast2B.setIndication(xcade.mssPortC, DIVERGING_MEDIUM_SPEED);
  } 
  else if (!turnout1CPXoverThrown && !turnoutP2BXoverThrown && !turnout1D2BThrown)
  {
    // Route is straight through - 1C to 2B
      xcade.mssPortC.setLocalOccupancy(block1COccupancy || ir1COccupancy || ir2BOccupancy);      
      xcadeExpander1.mssPortB.setLocalOccupancy(block2BOccupancy || ir1COccupancy || ir2BOccupancy);

      xcade.mssPortC.cascadeFromPort(xcadeExpander1.mssPortB, false);
      xcadeExpander1.mssPortB.cascadeFromPort(xcade.mssPortC, false);

      signalMast1C.setIndication(xcadeExpander1.mssPortB);
      signalMast2B.setIndication(xcade.mssPortC);
  }

  // 1D valid routes:
  //  - 1D to 2B
  if (turnout1D2BThrown && !turnoutP2BXoverThrown)
  {
    // Route is 1C to 2A
      xcade.mssPortD.setLocalOccupancy(block1DOccupancy || ir2BOccupancy);      
      xcadeExpander1.mssPortB.setLocalOccupancy(block2BOccupancy || ir2BOccupancy);      

      xcade.mssPortD.cascadeFromPort(xcadeExpander1.mssPortB, false);
      xcadeExpander1.mssPortB.cascadeFromPort(xcade.mssPortD, true);

      signalMast1D.setIndication(xcadeExpander1.mssPortB);
      signalMast2B.setIndication(xcade.mssPortD, DIVERGING_SLOW_SPEED);
  }

  if ((((uint32_t)currentTime - debugPrintfTime) > DEBUG_UPDATE_TIME_MS))
  {
    Serial.printf("\n\nPort 1.A - ");
    xcade.mssPortA.printDebugStr();
    Serial.printf("\nPort 1.B - ");
    xcade.mssPortB.printDebugStr();
    Serial.printf("\nPort 1.C - ");
    xcade.mssPortC.printDebugStr();
    Serial.printf("\nPort 1.D - ");
    xcade.mssPortD.printDebugStr();
    Serial.printf("\nPort 2.A - ");
    xcadeExpander1.mssPortA.printDebugStr();
    Serial.printf("\nPort 2.B - ");
    xcadeExpander1.mssPortB.printDebugStr();
    debugPrintfTime = currentTime;

    Serial.printf("\nTO 1=[%c] 2=[%c] 3=[%c] 4=[%c] 5=[%c] 6=[%c]\n", turnout1APThrown?'R':'N', turnout1BPXoverThrown?'R':'N', turnout1CPXoverThrown?'R':'N', 
      turnout1D2BThrown?'R':'N', turnoutP2AXoverThrown?'R':'N', turnoutP2BXoverThrown?'R':'N');

    Serial.printf("BD 1A=[%c] 1B=[%c] 1C=[%c] 1D=[%c] P=[%c] 2A=[%c] 2B=[%c]\n", block1AOccupancy?'*':'-', block1BOccupancy?'*':'-', block1COccupancy?'*':'-', 
      block1DOccupancy?'*':'-', blockPOccupancy?'*':'-', block2AOccupancy?'*':'-', block2BOccupancy?'*':'-');

    Serial.printf("IR P=[%c] 1C=[%c] 2A=[%c] 2B=[%c]\n", irPCOccupancy?'*':'-', ir1COccupancy?'*':'-', ir2AOccupancy?'*':'-', ir2BOccupancy?'*':'-');

    Serial.printf("Sig 1A:[%s/%s] 1B:[%s/%s] 1C:[%s/%s] 1D:[%s/%s]\n", xcade.signals.A1.getAspectText(), xcade.signals.A2.getAspectText(),
      xcade.signals.B1.getAspectText(), xcade.signals.B2.getAspectText(),
      xcade.signals.C1.getAspectText(), xcade.signals.C2.getAspectText(),
      xcade.signals.D1.getAspectText(), xcade.signals.D2.getAspectText());

    Serial.printf("Sig 2A:[%s/%s] 2B:[%s/%s]\n", xcadeExpander1.signals.A1.getAspectText(), xcadeExpander1.signals.A2.getAspectText(),
      xcadeExpander1.signals.B1.getAspectText(), xcadeExpander1.signals.B2.getAspectText());
  }


  // Now that all state is computed, send the outputs to the hardware
  xcade.updateOutputs();
  xcadeExpander1.updateOutputs();
}
