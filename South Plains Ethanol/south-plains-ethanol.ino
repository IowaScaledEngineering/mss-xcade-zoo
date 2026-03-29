/*************************************************************************
Title:    MSS-XCADE Jeff Ritter's South Plains Ethanol Module
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
          Iowa Scaled Engineering
File:     mss-xcade-south-plains.ino
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

// New signal rules to add a restricting diverging aspect for cases entering yard track

const static IndicationRule_t doubleHead4IndicationWesternUSRestricting[] = 
{
	{ INDICATION_STOP,                  SignalMast::DIVMASK_NOT_DIVERGING, ASPECT_RED,       ASPECT_RED,       ASPECT_OFF       },
	{ INDICATION_APPROACH,              SignalMast::DIVMASK_NOT_DIVERGING, ASPECT_YELLOW,    ASPECT_RED,       ASPECT_OFF       },
	{ INDICATION_APPROACH_DIVERGING,    SignalMast::DIVMASK_NOT_DIVERGING, ASPECT_YELLOW,    ASPECT_YELLOW,    ASPECT_OFF       },
	{ INDICATION_ADVANCE_APPROACH,      SignalMast::DIVMASK_NOT_DIVERGING, ASPECT_FL_YELLOW, ASPECT_RED,       ASPECT_OFF       },
	{ INDICATION_APPROACH_DIVERGING_AA, SignalMast::DIVMASK_NOT_DIVERGING, ASPECT_FL_YELLOW, ASPECT_RED,       ASPECT_OFF       },
	{ INDICATION_CLEAR,                 SignalMast::DIVMASK_NOT_DIVERGING, ASPECT_GREEN,     ASPECT_RED,       ASPECT_OFF       },

	{ INDICATION_STOP,                  SignalMast::DIVMASK_ALL_DIVERGING, ASPECT_RED,       ASPECT_RED,       ASPECT_OFF       },

	{ INDICATION_APPROACH,              SignalMast::DIVMASK_DIV_FULL_SPD,  ASPECT_RED,       ASPECT_YELLOW,    ASPECT_OFF,      },
	{ INDICATION_APPROACH,              SignalMast::DIVMASK_DIV_NO_FULL ,  ASPECT_RED,       ASPECT_FL_RED,    ASPECT_OFF,      },

	{ INDICATION_APPROACH_DIVERGING,    SignalMast::DIVMASK_DIV_FULL_SPD,  ASPECT_RED,       ASPECT_YELLOW,    ASPECT_OFF,      },
	{ INDICATION_APPROACH_DIVERGING,    SignalMast::DIVMASK_DIV_NO_FULL ,  ASPECT_RED,       ASPECT_FL_RED,    ASPECT_OFF,      },

	{ INDICATION_ADVANCE_APPROACH,      SignalMast::DIVMASK_DIV_FULL_SPD,  ASPECT_RED,       ASPECT_FL_YELLOW, ASPECT_OFF,      },
	{ INDICATION_ADVANCE_APPROACH,      SignalMast::DIVMASK_DIV_SLOW_SPD,  ASPECT_RED,       ASPECT_FL_RED,    ASPECT_OFF,      },
	{ INDICATION_ADVANCE_APPROACH,      SignalMast::DIVMASK_DIV_NO_FULL,   ASPECT_RED,       ASPECT_YELLOW,    ASPECT_OFF,      },

	{ INDICATION_APPROACH_DIVERGING_AA, SignalMast::DIVMASK_DIV_FULL_SPD,  ASPECT_RED,       ASPECT_FL_YELLOW, ASPECT_OFF,      },
	{ INDICATION_APPROACH_DIVERGING_AA, SignalMast::DIVMASK_DIV_SLOW_SPD,  ASPECT_RED,       ASPECT_FL_RED,    ASPECT_OFF,      },
	{ INDICATION_APPROACH_DIVERGING_AA, SignalMast::DIVMASK_DIV_NO_FULL ,  ASPECT_RED,       ASPECT_YELLOW,    ASPECT_OFF,      },

	{ INDICATION_CLEAR,                 SignalMast::DIVMASK_DIV_FULL_SPD,  ASPECT_RED,       ASPECT_GREEN,     ASPECT_OFF,      },
	{ INDICATION_CLEAR,                 SignalMast::DIVMASK_DIV_SLOW_SPD,  ASPECT_RED,       ASPECT_FL_RED,    ASPECT_OFF,      },
	{ INDICATION_CLEAR,                 SignalMast::DIVMASK_DIV_NO_FULL ,  ASPECT_RED,       ASPECT_YELLOW,    ASPECT_OFF,      }
};

WireMux wireMux;
XCade xcade;

SignalMast signalMastA;
SignalMast signalMastB;
SignalMast signalMastC;
SignalMast signalMastD;
SignalMast signalMastEntrance;

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

  // Configure the signal heads.  If you're already writing your own code, you probably already know what type of
  //  signals you plan to connect.  This is really only needed if you plan to do something other than use the default
  //  setting for 3-light heads and using the board common anode / cathode switch to drive the head polarity.

  // Create "signal masts" - this allows us to put multiple heads together and then feed in an indication, and let the internal
  //  signal rules drive the aspects based on a 1, 2, or 3-headed signal.  The rules can be overriden if you want different 
  //  aspects shown for a given condition.  However, that's largely an advanced use case, so we'll leave that alone for now.

  // Signal A & D will be single-headed signals, since there's only one path through the interlocking for those
  // Signal B & C will be double-headed signals that can display both a normal or a diverging route
  // Signal "Entrance" will be the control signal on the ethanol yard lead
  signalMastA.addSignalHeads(&xcade.signals.A1);
  signalMastB.addSignalHeads(&xcade.signals.B1, &xcade.signals.B2);
  signalMastC.addSignalHeads(&xcade.signals.C1, &xcade.signals.C2);
  signalMastD.addSignalHeads(&xcade.signals.D1);
  signalMastEntrance.addSignalHeads(&xcade.signals.A2);

  // B is the only one that needs non-default signal rules because I need a diverging restricted aspect
  signalMastB.setDoubleHeadRules(doubleHead4IndicationWesternUSRestricting, sizeof(doubleHead4IndicationWesternUSRestricting)/sizeof(doubleHead4IndicationWesternUSRestricting[0]));

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

  // I'm not going to ASCII-ize the track diagram - just see the png
  // Read sensors
  bool turnout1XoverThrown = xcade.gpio.digitalRead(1);
  bool turnout2XoverThrown = xcade.gpio.digitalRead(2);
  bool turnout3YardLeadThrown = xcade.gpio.digitalRead(3);

  bool blockAOccupancy = xcade.gpio.digitalRead(SENSOR_1_PIN);
  bool irCOccupancy = xcade.gpio.digitalRead(SENSOR_2_PIN);
  bool blockCOccupancy = xcade.gpio.digitalRead(SENSOR_3_PIN);

  bool blockBOccupancy = xcade.gpio.digitalRead(SENSOR_4_PIN);
  bool irBOccupancy = xcade.gpio.digitalRead(SENSOR_5_PIN);
  bool blockDOccupancy = xcade.gpio.digitalRead(SENSOR_6_PIN);
  
  // Start with all signals and ports at stop.  All routes not valid are invalid
  signalMastA.setIndication(INDICATION_STOP);
  signalMastB.setIndication(INDICATION_STOP);
  signalMastC.setIndication(INDICATION_STOP);
  signalMastD.setIndication(INDICATION_STOP);
  signalMastEntrance.setIndication(INDICATION_STOP);

  xcade.mssPortA.cascadeFromIndication(INDICATION_STOP);  
  xcade.mssPortB.cascadeFromIndication(INDICATION_STOP);  
  xcade.mssPortC.cascadeFromIndication(INDICATION_STOP);  
  xcade.mssPortD.cascadeFromIndication(INDICATION_STOP);  

  // Since what's below only sets occupancy for workable routes, we still need to provide occupancy for 
  //  the non-viable ones as well.  So we need to at least set the occupancy for each port based on 
  //  detectors we know are associated with it.

  xcade.mssPortA.setLocalOccupancy(blockAOccupancy);
  xcade.mssPortB.setLocalOccupancy(blockBOccupancy || irBOccupancy);
  xcade.mssPortC.setLocalOccupancy(blockCOccupancy || irCOccupancy);
  xcade.mssPortD.setLocalOccupancy(blockDOccupancy);

  // Now, work through all tracks on the right, one by one, and connect up all valid routes
  // In there, if a route is valid, it should set it in both directions.  

  // Port A Valid Routes
  //  - A->C
  if (!turnout1XoverThrown)
  {
    // Route is A to C
    xcade.mssPortA.setLocalOccupancy(blockAOccupancy || irCOccupancy);
    xcade.mssPortC.setLocalOccupancy(blockCOccupancy || irCOccupancy);

    xcade.mssPortA.cascadeFromPort(xcade.mssPortC, false);
    xcade.mssPortC.cascadeFromPort(xcade.mssPortA, false);

    signalMastA.setIndication(xcade.mssPortC);
    signalMastC.setIndication(xcade.mssPortA);
  }

  // Port B Valid Routes
  //  - B->D
  //  - B->C (crossover)
  //  - B->Yard (diverging restricted)

  if (!turnout1XoverThrown && !turnout2XoverThrown && !turnout3YardLeadThrown)
  {
    // Route is B->D
    xcade.mssPortB.setLocalOccupancy(blockBOccupancy || irBOccupancy);
    xcade.mssPortD.setLocalOccupancy(blockDOccupancy || irBOccupancy);

    xcade.mssPortB.cascadeFromPort(xcade.mssPortD);
    xcade.mssPortD.cascadeFromPort(xcade.mssPortB);

    signalMastB.setIndication(xcade.mssPortD);
    signalMastD.setIndication(xcade.mssPortB);
  }
  else if (turnout1XoverThrown && turnout2XoverThrown && !turnout3YardLeadThrown)
  {
    // Route is B->C
    // Route is B->D
    xcade.mssPortB.setLocalOccupancy(blockBOccupancy || irBOccupancy || irCOccupancy);
    xcade.mssPortC.setLocalOccupancy(blockCOccupancy || irBOccupancy || irCOccupancy);

    xcade.mssPortB.cascadeFromPort(xcade.mssPortC, true);
    xcade.mssPortC.cascadeFromPort(xcade.mssPortB, true);

    signalMastB.setIndication(xcade.mssPortC, DIVERGING_FULL_SPEED);
    signalMastC.setIndication(xcade.mssPortB, DIVERGING_FULL_SPEED);
  }
  else if (turnout3YardLeadThrown)
  {
    // Route is B->yard
    xcade.mssPortB.setLocalOccupancy(blockBOccupancy || irBOccupancy);

    // Cascade from a stop indication, since restricting is effectively stop
    //  and we want to project approach signals up the line
    xcade.mssPortB.cascadeFromIndication(INDICATION_STOP, true);

    signalMastB.setIndication(INDICATION_CLEAR, DIVERGING_SLOW_SPEED);
    signalMastEntrance.setIndication(xcade.mssPortB);
  }

  if ((((uint32_t)currentTime - debugPrintfTime) > DEBUG_UPDATE_TIME_MS))
  {
    debugPrintfTime = currentTime;

    Serial.printf("\n\nPort A - ");
    xcade.mssPortA.printDebugStr();
    Serial.printf("\nPort B - ");
    xcade.mssPortB.printDebugStr();
    Serial.printf("\nPort C - ");
    xcade.mssPortC.printDebugStr();
    Serial.printf("\nPort D - ");
    xcade.mssPortD.printDebugStr();

    Serial.printf("\nTO 1=[%c] 2=[%c] 3=[%c]\n", turnout1XoverThrown?'R':'N', turnout2XoverThrown?'R':'N', turnout3YardLeadThrown?'R':'N');
    Serial.printf("BD A=[%c] B=[%c] C=[%c] D=[%c]\n", blockAOccupancy?'*':'-', blockBOccupancy?'*':'-', blockCOccupancy?'*':'-', blockDOccupancy?'*':'-');
    Serial.printf("IR S2=[%c] S5=[%c]\n", irBOccupancy?'*':'-', irCOccupancy?'*':'-');

    Serial.printf("Sig A:[%s] B:[%s/%s] C:[%s/%s] D:[%s] Ent:[%s]\n", 
      xcade.signals.A1.getAspectText(),
      xcade.signals.B1.getAspectText(), xcade.signals.B2.getAspectText(),
      xcade.signals.C1.getAspectText(), xcade.signals.C2.getAspectText(),
      xcade.signals.D1.getAspectText(), 
      xcade.signals.A2.getAspectText());

  }


  // Now that all state is computed, send the outputs to the hardware
  xcade.updateOutputs();
}
