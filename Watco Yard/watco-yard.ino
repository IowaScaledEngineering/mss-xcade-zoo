/*************************************************************************
Title:    Iowa Scaled Engineering MSS-XCADE Double Track 
             Crossover for Watco Yard modules
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     mss-xcade-watco.ino
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2026 Michael Petersen & Nathan Holmes

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
SignalMast signalMastA;
SignalMast signalMastB;
SignalMast signalMastC;
SignalMast signalMastD;

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

  wireMux.begin(&Wire, XCADE_I2C_MUX_RESET);

  // Start up the xcade board.  
  //  This is the master board, on the same PCB as the host processor.  If you had additional xcade boards, you would
  //  start them up afterwards using additional xcade objects and their port number, like xcade_expansion1.begin(&wireMux, 1);

  xcade.begin(&wireMux);

  // Configure the signal heads.  If you're already writing your own code, you probably already know what type of
  //  signals you plan to connect.  This is really only needed if you plan to do something other than use the default
  //  setting for 3-light heads and using the board common anode / cathode switch to drive the head polarity.

  // For setSignalHeadType(), the options are SIGNAL_HEAD_THREE_LIGHT (default) or SIGNAL_HEAD_SEARCHLIGHT
  // For setSignalHeadPolarity(), the options are SIGNAL_POL_BOARD_SENSING (default - obey the board switch) or 
  //  SIGNAL_POL_COMMON_ANODE or SIGNAL_POL_COMMON_CATHODE if you want to change any individual head.  Note that changing
  //  this define DOES NOT change the common pin on the signal output.

  // Example:
  // xcade.signals.B1.setSignalHeadType(SIGNAL_HEAD_THREE_LIGHT);
  // xcade.signals.B1.setSignalHeadPolarity(SIGNAL_POL_BOARD_SENSING);

  // Create "signal masts" - this allows us to put multiple heads together and then feed in an indication, and let the internal
  //  signal rules drive the aspects based on a 1, 2, or 3-headed signal.  The rules can be overriden if you want different 
  //  aspects shown for a given condition.  However, that's largely an advanced use case, so we'll leave that alone for now.

  // All four signals will be double-headed
  signalMastA.addSignalHeads(&xcade.signals.A1, &xcade.signals.A2);
  signalMastB.addSignalHeads(&xcade.signals.B1, &xcade.signals.B2);
  signalMastC.addSignalHeads(&xcade.signals.C1, &xcade.signals.C2);
  signalMastD.addSignalHeads(&xcade.signals.D1, &xcade.signals.D2);
}

void loop() 
{
  uint32_t currentTime = millis();
  static uint32_t lastReadTime = 0, debugStatusReportTime = 0;

	// Because debouncing needs some time between samples, don't go for a hideous update rate
  // 50mS or so between samples does nicely.  That gives a 200mS buffer for changes, which is more
  // than enough for propagation delay
  // Remember, prototype signal logic is slow and deliberate!
	if (!(((uint32_t)currentTime - lastReadTime) > LOOP_UPDATE_TIME_MS))
    return;

  // Update the last time we ran through the loop to the current time
  lastReadTime = currentTime;

  // Just blink the RGB LED once a second in a nice dim of blue, so that we know the board is alive
  rgbLedWrite(XCADE_RGB_LED, 0, 0, ((currentTime % 1000) > 500)?16:0);

  // This is our example track layout - a single crossover between double track

  //   A1/A2 |--OO                                OO--| C1/C2
  //	BD=S1         BD=S9      /         TO=GP2       BD=S5
  //	A ----|-----------=-----/-----------------------|------------- C
  //      IR=S2              /             \        IR=S6
  //                        /             --\--                        
  //      IR=S4  2TO=GP3   /                 \        BD=S7           
  //	B ----|-----------------------------------------|------------- D
  //	BD=S3      \   \    TO=GP1   BD=S10           IR=S8
  //  B1/B2 |--OO                                 OO--| D1/D2
  

  //  Sensor S1 - The DCC current detector for block A
  //  Sensor S2 - The IR detector at the block boundary between A and north plant track
  //  Sensor S3 - The DCC current detector for block B
  //  Sensor S4 - The IR detector at the block boundary between A and south plant track
  //  Sensor S5 - The DCC current detector for block C
  //  Sensor S6 - The IR detector at the block boundary between C and north plant track
  //  Sensor S7 - The DCC current detector for block D
  //  Sensor S8 - The IR detector at the block boundary between D and south plant track
  //  Sensor S9 - The DCC current detector for the north plant track (P1)
  //  Sensor S10 - The DCC current detector for the south plant track (P2) and crossing lead to yard

  //  GPIO1     - turnout from B across diamond into yard (open/high = normal, low = reverse)
  //  GPIO2     - turnout for A to D crossover  (open/high = normal, low = reverse)
  //  GPIO3     - two turnouts leading from south plant track to yard tracks (open/high = normal, low = reverse)


  // First, read the input state from the hardware
  xcade.updateInputs();

  // Read sensors
  bool turnoutBtoNorthYardNormal = xcade.gpio.digitalRead(1);
  bool turnoutCrossoverNormal = xcade.gpio.digitalRead(2);
  bool turnoutBtoSouthYardNormal = xcade.gpio.digitalRead(3);

  bool blockAOccupancy = xcade.gpio.digitalRead(SENSOR_1_PIN);
  bool blockBOccupancy = xcade.gpio.digitalRead(SENSOR_3_PIN);
  bool blockCOccupancy = xcade.gpio.digitalRead(SENSOR_5_PIN);
  bool blockDOccupancy = xcade.gpio.digitalRead(SENSOR_7_PIN);
  bool blockP1Occupancy = xcade.gpio.digitalRead(SENSOR_9_PIN);
  bool blockP2Occupancy = xcade.gpio.digitalRead(SENSOR_10_PIN);
  bool irAOccupancy = xcade.gpio.digitalRead(SENSOR_2_PIN);
  bool irBOccupancy = xcade.gpio.digitalRead(SENSOR_4_PIN);
  bool irCOccupancy = xcade.gpio.digitalRead(SENSOR_6_PIN);
  bool irDOccupancy = xcade.gpio.digitalRead(SENSOR_8_PIN);

  // Start with all signals and ports at stop.  All routes not valid are invalid
  signalMastA.setIndication(INDICATION_STOP);
  signalMastB.setIndication(INDICATION_STOP);
  signalMastC.setIndication(INDICATION_STOP);
  signalMastD.setIndication(INDICATION_STOP);

  xcade.mssPortA.cascadeFromIndication(INDICATION_STOP);  
  xcade.mssPortB.cascadeFromIndication(INDICATION_STOP);  
  xcade.mssPortC.cascadeFromIndication(INDICATION_STOP);  
  xcade.mssPortD.cascadeFromIndication(INDICATION_STOP);  

  // Set basic occupancy - even though a port at stop will send approach, we also may
  //  have something tripping the occupancy sensors, which would mean we need to send stop
  xcade.mssPortA.setLocalOccupancy(blockAOccupancy || irAOccupancy);
  xcade.mssPortB.setLocalOccupancy(blockBOccupancy || irBOccupancy);
  xcade.mssPortC.setLocalOccupancy(blockCOccupancy || irDOccupancy);
  xcade.mssPortD.setLocalOccupancy(blockDOccupancy || irDOccupancy);

  // Port A Valid Routes (BtoNorthYard turnout must be normal for any to be valid)
  //  - A<->C
  //  - A<->D

  if (turnoutBtoNorthYardNormal)
  {
    if (turnoutCrossoverNormal)
    {
      xcade.mssPortA.cascadeFromPort(xcade.mssPortC);
      xcade.mssPortA.setLocalOccupancy(blockAOccupancy || blockP1Occupancy || irAOccupancy || irCOccupancy);
      xcade.mssPortC.cascadeFromPort(xcade.mssPortA);
      xcade.mssPortC.setLocalOccupancy(blockCOccupancy || blockP1Occupancy || irAOccupancy || irCOccupancy);

      signalMastA.setIndication(xcade.mssPortC);
      signalMastC.setIndication(xcade.mssPortA);
    } else {
      xcade.mssPortA.cascadeFromPort(xcade.mssPortD, true);
      xcade.mssPortA.setLocalOccupancy(blockAOccupancy || blockP1Occupancy || irAOccupancy || irDOccupancy);
      xcade.mssPortD.cascadeFromPort(xcade.mssPortA, true);
      xcade.mssPortD.setLocalOccupancy(blockDOccupancy || blockP1Occupancy || irAOccupancy || irDOccupancy);

      signalMastA.setIndication(xcade.mssPortD, DIVERGING_FULL_SPEED);
      signalMastD.setIndication(xcade.mssPortA, DIVERGING_FULL_SPEED);
    }
  }

  // Port B Valid Routes
  //  - B<-->D if crossover normal
  //  - B<-->South Yard
  //  - B<-->North Yard
  
  if (turnoutBtoNorthYardNormal && turnoutCrossoverNormal && turnoutBtoSouthYardNormal)
  {
    xcade.mssPortB.cascadeFromPort(xcade.mssPortD);
    xcade.mssPortB.setLocalOccupancy(blockBOccupancy || blockP2Occupancy || irBOccupancy || irDOccupancy);
    xcade.mssPortD.cascadeFromPort(xcade.mssPortB);
    xcade.mssPortD.setLocalOccupancy(blockDOccupancy || blockP2Occupancy || irBOccupancy || irDOccupancy);

    signalMastB.setIndication(xcade.mssPortD);
    signalMastD.setIndication(xcade.mssPortB);
  } else if (!turnoutBtoNorthYardNormal) {
    // Priority to south yard switches, since they're first
    xcade.mssPortB.cascadeFromIndication(INDICATION_RESTRICTING, true); // Actual sensor values don't matter, we're going to send restricting/stop
    signalMastB.setIndication(INDICATION_RESTRICTING, DIVERGING_SLOW_SPEED);

  } else if (!turnoutBtoSouthYardNormal) {
    // Second would be the case where we're lined across the north track into the north yard
    xcade.mssPortB.cascadeFromIndication(INDICATION_RESTRICTING, true); // Actual sensor values don't matter, we're going to send restricting/stop
    signalMastB.setIndication(INDICATION_RESTRICTING, DIVERGING_SLOW_SPEED);
  }

  // Port C Valid Routes
  //  C<-->A - already handled by A
  //  C<-->B - already handled by B

  // Port D Valid Routes
  //  D<-->A - already handled by A
  //  D<-->B - already handled by B

  // Now that all state is computed, send the outputs to the hardware
  xcade.updateOutputs();

  if ((((uint32_t)currentTime - debugStatusReportTime) > DEBUG_UPDATE_TIME_MS))
  {
    debugStatusReportTime = currentTime;
    Serial.printf("\n\nPort A - ");
    xcade.mssPortA.printDebugStr();
    Serial.printf("\nPort B - ");
    xcade.mssPortB.printDebugStr();
    Serial.printf("\nPort C - ");
    xcade.mssPortC.printDebugStr();
    Serial.printf("\nPort D - ");
    xcade.mssPortD.printDebugStr();

    Serial.printf("\nTO BtoNY=[%c] BtoSY=[%c] Xover=[%c]\n", turnoutBtoNorthYardNormal?'N':'R', turnoutBtoSouthYardNormal?'N':'R', turnoutCrossoverNormal?'N':'R');

    Serial.printf("BD A=[%c] B=[%c] C=[%c] D=[%c] P1=[%c] P2=[%c]\n", blockAOccupancy?'*':'-', blockBOccupancy?'*':'-', blockCOccupancy?'*':'-', 
      blockDOccupancy?'*':'-', blockP1Occupancy?'*':'-', blockP2Occupancy?'*':'-');

    Serial.printf("IR A=[%c] B=[%c] C=[%c] D=[%c]\n", irAOccupancy?'*':'-', irBOccupancy?'*':'-', irCOccupancy?'*':'-', irDOccupancy?'*':'-');

    Serial.printf("Sig A:[%s/%s] B:[%s/%s] C:[%s/%s] D:[%s/%s]\n", xcade.signals.A1.getAspectText(), xcade.signals.A2.getAspectText(),
      xcade.signals.B1.getAspectText(), xcade.signals.B2.getAspectText(),
      xcade.signals.C1.getAspectText(), xcade.signals.C2.getAspectText(),
      xcade.signals.D1.getAspectText(), xcade.signals.D2.getAspectText());
  }
}

