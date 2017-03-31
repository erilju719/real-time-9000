/*
-- Copyright (C) 2005 IMEC                                                  -
   --                                                                          -
   -- Redistribution and use in source and binary forms, with or without       -
   -- modification, are permitted provided that the following conditions       -
   -- are met:                                                                 -
   --                                                                          -
   -- 1. Redistributions of source code must retain the above copyright        -
   --    notice, this list of conditions and the following disclaimer.         -
   --                                                                          -
   -- 2. Redistributions in binary form must reproduce the above               -
   --    copyright notice, this list of conditions and the following           -
   --    disclaimer in the documentation and/or other materials provided       -
   --    with the distribution.                                                -
   --                                                                          -
   -- 3. Neither the name of the author nor the names of contributors          -
   --    may be used to endorse or promote products derived from this          -
   --    software without specific prior written permission.                   -
   --                                                                          -
   -- THIS CODE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS''           -
   -- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED        -
   -- TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A          -
   -- PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR       -
   -- CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,             -
   -- SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         -
   -- LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF         -
   -- USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND      -
   -- ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,       -
   -- OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT       -
   -- OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF       -
   -- SUCH DAMAGE.                                                             -
   --                                                                          -
   -----------------------------------------------------------------------------
   -----------------------------------------------------------------------------
   -- File           : threads_RR.c
   -----------------------------------------------------------------------------
   -- Description    : C code
   -- --------------------------------------------------------------------------
   -- Author         : Kristof Loots
   -- Date           : 14/09/2006
   -- Version        : V1.0
   -- Change history :
   -----------------------------------------------------------------------------
 */

#include "ball.h"

void* thread_ball () {

	double x = 200, y = 300, angle = M_PI/2+1, speed = 5;
	int radius = 7;
	double newx, newy;
	struct ball_msg ball;
	struct response_msg response;

	while(1) {

		//New position
		newx = x + sin(angle)*speed;
		newy = y - cos(angle)*speed;

		/* Check hits and give approved coordinates to display. If display approves, update coordinates.
		 * If display detects hit, check new coordinates. Loop until display approves.*/
		do{

			int hit;

			/* Checking hits. If coordinates are updated, check new coordinates.
			 Stop when new coordinates are approved*/
			do{
				hit=0;

				//Left wall
				if (newx < 60 + radius) {
					newx = 2*(60 + radius) - (newx);
					angle = -angle;
					hit=1;
				}

				//Right wall
				else if (newx > 60 + BRICK_ZONE_COLUMNS - radius) {
					newx = 2*(60 +  BRICK_ZONE_COLUMNS - radius) - newx;
					angle = -angle;
					hit=1;
				}

				//Roof
				else if (newy < 60 + radius) {
					newy = 2*(60 + radius) - (newy);
					hit=1;
					if(angle > 0) {
						angle = M_PI - angle;
					}
					else {
						angle = -M_PI - angle;
					}
				}

				//Floor
				else if (newy > 60 + BRICK_ZONE_ROWS - radius) {
					newy = 2*(60 + BRICK_ZONE_ROWS - radius) - newy;
					hit=1;
					if (angle > 0) {
						angle = M_PI - angle;
					}
					else {
						angle = -M_PI - angle;
					}
				}
			}while(hit==1);//Inner do

			//Create message to display
			ball.x=(int)round(newx);
			ball.y=(int)round(newy);
			ball.angle=angle;
			ball.speed=speed;

			//Send message to display
			XMbox_WriteBlocking(&Mbox,&ball,20);
			XMbox_ReadBlocking(&Mbox,&response,24);
			//Receive response from display

			newx=response.x;
			newy=response.y;
			speed=response.speed;
			angle=response.angle;

		} while(response.unchanged == 0);//Outer do

		//Update to new coordinates
		x=newx;
		y=newy;
	}
}

int main (void) {

  print("-- Entering main() --\r\n");
  //Initialize Xilkernel
  xilkernel_init();

  //Add main_prog as the static thread that will be invoked by Xilkernel
  xmk_add_static_thread(main_prog, 0);

  //Start Xilkernel
  xilkernel_start();

  //Control does not reach here
  return 0;
}

void* main_prog(void *arg) {// This thread is statically created and has priority 0 (This is the highest possible)

  print("-- Entering main_prog() --\r\n");

  // CONFIGURE THE MAILBOX HERE
  	XMbox_Config* ConfigPtr;
  	int Status;

  	ConfigPtr = XMbox_LookupConfig(MBOX_DEVICE_ID );
  	if (ConfigPtr == (XMbox_Config *)NULL) {
  		print("-- Error configuring Mbox uB1 --\r\n");
  		return XST_FAILURE;
  	}
  	Status = XMbox_CfgInitialize(&Mbox, ConfigPtr, ConfigPtr->BaseAddress);
  	if (Status != XST_SUCCESS) {
  		print("-- Error initializing Mbox uB1 --\r\n");
  		return XST_FAILURE;
  	}

  int ret;
  //Start thread_ball
  ret = pthread_create (&tid1, NULL, (void*)thread_ball, NULL);
  if (ret != 0) {
    xil_printf ("-- ERROR (%d) launching thread_ball...\r\n", ret);
  }
  else {
    xil_printf ("Thread ball launched with ID %d \r\n",tid1);
  }

  return 0;
}
