/*
 * lab1_draw.h
 *
 *  Created on: 14 Feb 2017
 *      Author: erikl
 */

#ifndef SRC_LAB1_DRAW_H_
#define SRC_LAB1_DRAW_H_

#include "xtft.h"
#include "xparameters.h"
#include "xmk.h"
#include "sys/init.h"
#include "xmbox.h"
#include "xmutex.h"
#include "xgpio.h"
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <sys/timer.h> //for using sleep. need to set config_time to true
#include <sys/intr.h> //xilkernel api for interrupts
#include <sys/msg.h>
#include <sys/ipc.h>

// Mailbox declaration
//#define MY_CPU_ID 1
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

struct response_msg response;
unsigned int columns_check=0;
unsigned int columns_done=0;
unsigned int bricks_left = 80;
unsigned int bricks[8][10];// brick[i][j] row i column j.
unsigned int columns_alive[10]={1,1,1,1,1,1,1,1,1,1};// brick[i][j] row i column j.

pthread_mutex_t swmutex;

#define TFT_DEVICE_ID    XPAR_TFT_0_DEVICE_ID
#define DDR_HIGH_ADDR    XPAR_PS7_DDR_0_S_AXI_HIGHADDR

#ifndef DDR_HIGH_ADDR
#warning "CHECK FOR THE VALID DDR ADDRESS IN XPARAMETERS.H"
#endif

#define DISPLAY_COLUMNS  640
#define DISPLAY_ROWS     480
#define BRICK_ZONE_COLUMNS 455
#define BRICK_ZONE_ROWS 360

//Brick structure
#define BRICK_WIDTH 40
#define BRICK_HEIGHT 15
#define D 5 //Distance
#define R 7 //Radius

/**
 * User has to specify a 2MB memory space for filling the frame data.
 * This constant has to be updated based on the memory map of the
 * system.
 */
#define TFT_FRAME_ADDR        0x10000000

//Function Prototypes //

void* thread_display ();
void* thread_brick_column_1 ();
void* thread_brick_column_2 ();
void* thread_brick_column_3 ();
void* thread_brick_column_4 ();
void* thread_brick_column_5 ();
void* thread_brick_column_6 ();
void* thread_brick_column_7 ();
void* thread_brick_column_8 ();
void* thread_brick_column_9 ();
void* thread_brick_column_10 ();

void* main_prog(void *arg);

int TftInit(u32 TftDeviceId);
int XTft_DrawCircle(XTft *Tft, int x0, int y0, int radius, int col);
void SetPixel(XTft *InstancePtr, int ColVal, int RowVal, u32 PixelVal);
int XTft_DrawSolidBox(XTft *Tft, int x1, int y1, int x2, int y2, unsigned int col);
void init_brick_zone();
void init_score_zone();
void value_display();
void check_column(int j);
void increase_score(int j);
void update_special_columns();
void update_columns();
void remove_brick(int i,int j);
int brick_alive(int i, int j);
void check_column_status(int j);
int nr_columns_alive();
// Variables //

pthread_t tid1, tidbc1, tidbc2, tidbc3, tidbc4, tidbc5, tidbc6, tidbc7, tidbc8, tidbc9, tidbc10;
static XTft TftInstance;

//INIT BAR
#define	Y_BAR 410 //The y coordinate is fixed
#define WIDTH_BAR 80
#define HEIGHT_BAR 5
int x_bar = 248;
int newx_bar = 248;

//Variables for interrupt
int sp[] = {0,0,0,0,0,0,0,0,0,0};
unsigned int last_interrupt_time; //Variable to keep track of time between interrupts (used for debouncing)
unsigned int btn_pressed_time;
unsigned int latest_move_time;
unsigned int holding_button = 0;
XGpio gpPB; //PB device instance.
int left_btn_pressed = 0;
int right_btn_pressed = 0;


int rand1, rand2;

//Score counter
unsigned int score = 0;

#endif /* SRC_LAB1_DRAW_H_ */
