#ifndef SRC_BALL_H_
#define SRC_BALL_H_

#include "xparameters.h"
#include "xmk.h"
#include "sys/init.h"
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "xmbox.h"
#include <errno.h>

#define DISPLAY_COLUMNS  640
#define DISPLAY_ROWS     480
#define BRICK_ZONE_COLUMNS 455
#define BRICK_ZONE_ROWS 360

// Mailbox declaration
#define MY_CPU_ID XPAR_CPU_ID
#define MBOX_DEVICE_ID		XPAR_MBOX_0_DEVICE_ID
static XMbox Mbox;	/* Instance of the Mailbox driver */

struct ball_msg {
	int x,y,speed;
	double angle;
};

struct response_msg{
	int x,y,speed,unchanged;
	double angle;
};

void* thread_ball ();
void* main_prog(void *arg);
// Variables //

pthread_t tid1;

#endif /* SRC_BALL_H_ */
