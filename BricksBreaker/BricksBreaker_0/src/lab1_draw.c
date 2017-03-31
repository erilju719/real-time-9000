/*
   -----------------------------------------------------------------------------
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

#include "lab1_draw.h"

//The interrupt routine
static void gpPBIntHandler(void *arg) //Should be very short (in time). In a practical program, don't print etc.
{
	unsigned char val;
	//clear the interrupt flag. if this is not done, gpio will keep interrupting the microblaze.--
	// --Possible to use (XGpio*)arg instead of &gpPB
	XGpio_InterruptClear(&gpPB,1);

	//Read the state of the push buttons.
			val = XGpio_DiscreteRead(&gpPB, 1);

			left_btn_pressed = 0;
			right_btn_pressed = 0;

	// Ignore interrupts within 5*10 = 50 ms after an accepted interrupt (debouncing)
	if (xget_clock_ticks()- last_interrupt_time > 5)
	{

		// Time when the interrupt was accepted
		last_interrupt_time = xget_clock_ticks();

		// These are used for continuous bar movement
		if (val == 2) {
			left_btn_pressed = 1;
			right_btn_pressed = 0;
			btn_pressed_time = xget_clock_ticks();
		}
		else if (val == 1) {
			left_btn_pressed = 0;
			right_btn_pressed = 1;
			btn_pressed_time = xget_clock_ticks();
		}

		// These are used for discrete bar movement
		if (val == 2 && newx_bar -25 >= 60) {
			newx_bar = newx_bar - 25;
		}
		else if(val == 2 && newx_bar -25 < 60){
			newx_bar = 60;
		}
		else if (val == 1 && ((newx_bar + WIDTH_BAR + 25) <= (60 + BRICK_ZONE_COLUMNS))) {
			newx_bar = newx_bar + 25;
		}
		else if (val == 1 && ((newx_bar + WIDTH_BAR + 25) > (60 + BRICK_ZONE_COLUMNS))) {
			newx_bar = 60 + BRICK_ZONE_COLUMNS - WIDTH_BAR;
		}
	}
}

void* thread_display() {

	//INIT BALL
	int x_ball = 200, y_ball = 300;
	int radius = 7;
	int newx_ball, newy_ball, speed_ball;
	double angle_ball;
	struct ball_msg ball;
	int unchanged; // Flag to keep track of whether ball position is unchanged

	//Time counter
	unsigned int time_counter = xget_clock_ticks();
	//FPS value
	unsigned int fps;

	while(1) {

		// Calculate FPS
		fps = 100/(xget_clock_ticks() - time_counter);
		time_counter = xget_clock_ticks();
		// Display FPS
		value_display(fps, 60 + BRICK_ZONE_COLUMNS + 65, 60 + 5*BRICK_ZONE_ROWS/6);

		// Display the elapsed time (in seconds)
		value_display(time_counter/100, 60 + BRICK_ZONE_COLUMNS + 65, 60 + BRICK_ZONE_ROWS/3);

		// Display current score
		value_display(score, 60 + BRICK_ZONE_COLUMNS + 65, 60 + BRICK_ZONE_ROWS/6);

		// Display the number of bricks left
		value_display(bricks_left, 60 + BRICK_ZONE_COLUMNS + 65, 60 + 2*BRICK_ZONE_ROWS/3);


		//BALL
		//Get data from ball
		XMbox_ReadBlocking(&Mbox,&ball,20);
		newx_ball=ball.x;
		newy_ball=ball.y;
		angle_ball=ball.angle;
		speed_ball=ball.speed;

		// Display current ball speed (in pixels per second)
		value_display(fps*speed_ball, 60 + BRICK_ZONE_COLUMNS + 65, 60 + BRICK_ZONE_ROWS/2);

		//Checking bar

		unchanged = 1;

		if ((newy_ball > Y_BAR - radius) && ((newx_ball >= newx_bar )&& (newx_ball <= newx_bar + WIDTH_BAR ))) {
			newy_ball = 2*(Y_BAR - radius) - newy_ball;
			unchanged=0;
			if (angle_ball > 0) {
				angle_ball = M_PI - angle_ball;
			}
			else {
				angle_ball = -M_PI - angle_ball;
			}
		}

		response.x=newx_ball;
		response.y=newy_ball;
		response.angle=angle_ball;
		response.speed=speed_ball;
		response.unchanged=unchanged;

		//If hit bar, return new coords since it's impossible to hit columns afterwards.
		//Else check the columns
		//if(response.unchanged){

			columns_check=1;
			sleep(1);
			columns_check=0;
		//}

		XMbox_WriteBlocking(&Mbox,&response,24);

		// Check if button pressed for more than 250 ms
		if ((left_btn_pressed == 1) && (xget_clock_ticks() - btn_pressed_time > 25)){

			if(xget_clock_ticks() >= latest_move_time + 2){
				if (newx_bar -4 >= 60) {
					newx_bar = newx_bar - 4;
				}
				else if(newx_bar -4 < 60){
					newx_bar = 60;
				}
				//Next move in 2*10 = 20 ms -> move the bar 200 pixels per second
				latest_move_time = latest_move_time + 2;
			}

		if(holding_button == 0){
			holding_button = 1;
			latest_move_time = xget_clock_ticks();
		}
	}
	else if ((right_btn_pressed == 1) && (xget_clock_ticks() - btn_pressed_time > 25)) {

		if(xget_clock_ticks() >= latest_move_time + 2){

			if ((newx_bar + WIDTH_BAR + 4) <= (60 + BRICK_ZONE_COLUMNS)) {
				newx_bar = newx_bar + 4;
			}
			else if ((newx_bar + WIDTH_BAR + 4) > (60 + BRICK_ZONE_COLUMNS)) {
				newx_bar = 60 + BRICK_ZONE_COLUMNS - WIDTH_BAR;
			}
			//Next move in 2*10 = 20 ms -> move the bar 200 pixels per second
			latest_move_time = latest_move_time + 2;
		}

		if(holding_button == 0){
			holding_button = 1;
			latest_move_time = xget_clock_ticks();
		}
	}
	else{
		holding_button = 0;
	}


		//Plotting
		if(response.unchanged){
			//Erase the old circle (draw green circle at old position)
			XTft_DrawCircle(&TftInstance, x_ball, y_ball, radius, 0x0000ff00);
			//Draw blue circle with radius 7 pixels at new position
			XTft_DrawCircle(&TftInstance, newx_ball, newy_ball, radius, 0x000000ff);

			x_ball=newx_ball;
			y_ball=newy_ball;

			//BAR
			//Erase the old bar(draw green bar at old position)
			XTft_DrawSolidBox(&TftInstance, x_bar, Y_BAR,x_bar + WIDTH_BAR,Y_BAR + HEIGHT_BAR,0x0000ff00);
			//Draw blue bar at new position
			x_bar=newx_bar;
			XTft_DrawSolidBox(&TftInstance, x_bar, Y_BAR,x_bar + WIDTH_BAR,Y_BAR + HEIGHT_BAR,0x000000ff);
		}
		else{

			update_columns();
		}
	}
}

void* thread_brick_column_1() {
	int j=1; //Column index
	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

void* thread_brick_column_2() {
	int j=2; //Column index
	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

void* thread_brick_column_3() {
	int j=3; //Column index
	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

void* thread_brick_column_4() {
	int j=4; //Column index
	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

void* thread_brick_column_5() {
	int j=5; //Column index

	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

void* thread_brick_column_6() {
	int j=6; //Column index
	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

void* thread_brick_column_7() {
	int j=7; //Column index
	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

void* thread_brick_column_8() {
	int j=8; //Column index
	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

void* thread_brick_column_9() {
	int j=9; //Column index
	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

void* thread_brick_column_10() {
	int j=10; //Column index
	while(1){
		sleep(1);
		while(!columns_check){ //Wait for new ball message
			sleep(1);
		}
		check_column(j);
	}
}

int main (void) {

	print("-- Entering main() --\r\n");
	//Initialize Xilkernel
	xilkernel_init();

	//Initialize TFT
	TftInit(TFT_DEVICE_ID);

	//Add main_prog as the static thread that will be invoked by Xilkernel
	xmk_add_static_thread(main_prog, 0);

	//Start Xilkernel
	xilkernel_start();

	//Control does not reach here
	return 0;
}

void* main_prog(void *arg) {// This thread is statically created and has priority 0 (This is the highest possible)

	//INIT INTERRUPT HANDLING
	int Status;
	last_interrupt_time = xget_clock_ticks(); //Initialize tick counter

	xil_printf("Initializing PB\r\n");
	// Initialise the PB instance
	Status = XGpio_Initialize(&gpPB, XPAR_GPIO_0_DEVICE_ID);
	// set PB gpio direction to input.
	XGpio_SetDataDirection(&gpPB, 1, 0x000000FF);

	xil_printf("Enabling PB interrupts\r\n");
	//global enable
	XGpio_InterruptGlobalEnable(&gpPB);
	// interrupt enable. both global enable and this function should be called to enable gpio interrupts.
	XGpio_InterruptEnable(&gpPB,1);
	//register the handler with xilkernel
	register_int_handler(XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR, gpPBIntHandler, &gpPB);
	//enable the interrupt in xilkernel
	enable_interrupt(XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_0_IP2INTC_IRPT_INTR);

	// CONFIGURE THE MAILBOX HERE
	XMbox_Config* ConfigPtr;
	int Status_Mbox;

	ConfigPtr = XMbox_LookupConfig(MBOX_DEVICE_ID);
	if (ConfigPtr == (XMbox_Config *)NULL) {
		print("-- Error configuring Mbox --\r\n");
		return XST_FAILURE;
	}
	Status_Mbox = XMbox_CfgInitialize(&Mbox, ConfigPtr, ConfigPtr->BaseAddress);
	if (Status_Mbox != XST_SUCCESS) {
		print("-- Error initializing Mbox --\r\n");
		return XST_FAILURE;
	}

	//SOFTWARE MUTEX
	int ret = pthread_mutex_init (&swmutex, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) init swmutex...\r\n", ret);
	}

	//Blue text and white background
	XTft_SetColor(&TftInstance, 0x000000ff, 0x00ffffff);
	XTft_ClearScreen(&TftInstance);

	//Draw backgrounds for the brick and score zones
	init_brick_zone();
	init_score_zone();

	//Start thread_display
	ret = pthread_create (&tid1, NULL, (void*)thread_display, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_display...\r\n", ret);
	}
	else {
		xil_printf ("Thread display launched with ID %d \r\n",tid1);
	}

	//START BRICK_COLUMN THREADS

	ret = pthread_create (&tidbc1, NULL, (void*)thread_brick_column_1, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_1...\r\n", ret);
	}
	else {
		xil_printf ("Thread display launched with ID %d \r\n",tidbc1);
	}

	ret = pthread_create (&tidbc2, NULL, (void*)thread_brick_column_2, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_2...\r\n", ret);
	}
	else {
		xil_printf ("Thread display launched with ID %d \r\n",tidbc2);
	}

	ret = pthread_create (&tidbc3, NULL, (void*)thread_brick_column_3, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_3...\r\n", ret);
	}
	else {
		xil_printf ("Thread display launched with ID %d \r\n",tidbc3);
	}

	ret = pthread_create (&tidbc4, NULL, (void*)thread_brick_column_4, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_4...\r\n", ret);
	}
	else {
		xil_printf ("Thread display launched with ID %d \r\n",tidbc4);
	}

	ret = pthread_create (&tidbc5, NULL, (void*)thread_brick_column_5, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_5...\r\n", ret);
	}
	else {
		xil_printf ("Thread tidbc5 launched with ID %d \r\n",tidbc5);
	}

	ret = pthread_create (&tidbc6, NULL, (void*)thread_brick_column_6, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_6...\r\n", ret);
	}
	else {
		xil_printf ("Thread tidbc6 launched with ID %d \r\n",tidbc6);
	}

	ret = pthread_create (&tidbc7, NULL, (void*)thread_brick_column_7, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_7...\r\n", ret);
	}
	else {
		xil_printf ("Thread display launched with ID %d \r\n",tidbc7);
	}

	ret = pthread_create (&tidbc8, NULL, (void*)thread_brick_column_8, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_8...\r\n", ret);
	}
	else {
		xil_printf ("Thread display launched with ID %d \r\n",tidbc8);
	}

	ret = pthread_create (&tidbc9, NULL, (void*)thread_brick_column_9, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_9...\r\n", ret);
	}
	else {
		xil_printf ("Thread display launched with ID %d \r\n",tidbc9);
	}

	ret = pthread_create (&tidbc10, NULL, (void*)thread_brick_column_10, NULL);
	if (ret != 0) {
		xil_printf ("-- ERROR (%d) launching thread_brick_column_10...\r\n", ret);
	}
	else {
		xil_printf ("Thread display launched with ID %d \r\n",tidbc10);
	}

	return 0;
}

int TftInit(u32 TftDeviceId)
{
	int Status;
	XTft_Config *TftConfigPtr;


	//Get address of the XTft_Config structure for the given device id.
	TftConfigPtr = XTft_LookupConfig(TftDeviceId);
	if (TftConfigPtr == (XTft_Config *)NULL) {
		return XST_FAILURE;
	}


	 /* Initialize all the TftInstance members and fills the screen with
	 * default background color.*/
	Status = XTft_CfgInitialize(&TftInstance, TftConfigPtr,
			TftConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	 /* Wait till Vsync(Video address latch) status bit is set before writing
	 * the frame address into the Address Register. This ensures that the
	 * current frame has been displayed and we can display a new frame of
	 * data. Checking the Vsync state ensures that there is no data flicker
	 * when displaying frames in real time though there is some delay due to
	 * polling. */
	while (XTft_GetVsyncStatus(&TftInstance) !=
			XTFT_IESR_VADDRLATCH_STATUS_MASK);


	 /* Change the Video Memory Base Address from default value to
	 * a valid Memory Address and clear the screen.*/
	XTft_SetFrameBaseAddr(&TftInstance, TFT_FRAME_ADDR);

	print("Finish initializing TFT\n\r");

	return 0;
}

int XTft_DrawCircle(XTft *Tft, int x0, int y0, int radius,int col)
{
	for(int dy=-radius; dy<=radius; dy++) {
		for(int dx=-radius; dx<=radius; dx++) {
			if(dx*dx+dy*dy <= radius*radius + radius*0.8f) {
				SetPixel(Tft, x0+dx, y0+dy, col);
			}
		}
	}
	return 0;
}

// Modified version of XTft_SetPixel without the assertions
void SetPixel(XTft *InstancePtr, int ColVal, int RowVal, u32 PixelVal)
{
	if ((ColVal < 0) || (RowVal < 0))
	{
		return;
	}

	//Set the pixel at the given position with the color value.
	Xil_Out32(InstancePtr->TftConfig.VideoMemBaseAddr +
			(4 * ((RowVal) * XTFT_DISPLAY_BUFFER_WIDTH + ColVal)),
			PixelVal);
	return;
}

int XTft_DrawSolidBox(XTft *Tft, int x1, int y1, int x2, int y2, unsigned int col)
{

	int xmin,xmax,ymin,ymax,i,j;

	if (x2 < x1) {
		xmin = x2;
		xmax = x1;
	}
	else {
		xmin = x1;
		xmax = x2;
	}
	if (y2 < y1) {
		ymin = y2;
		ymax = y1;
	}
	else {
		ymin = y1;
		ymax = y2;
	}

	for (i=xmin; i<=xmax; i++) {
		for (j=ymin; j<=ymax; j++) {
			SetPixel(Tft, i, j, col);
		}
	}
	return 0;

}

void init_brick_zone(){

	srand(10000); //Initialize seed for randomizing numbers

	// Background for brick zone
	XTft_DrawSolidBox(&TftInstance, 60, 60, 60 + BRICK_ZONE_COLUMNS,
			60 + BRICK_ZONE_ROWS, 0x0000ff00);

	// Columns
	rand1=(int)floor(9.999*(double)rand() / (double)RAND_MAX); // from 0 to 9
	do{
		rand2=(int)floor(9.999*(double)rand() / (double)RAND_MAX); // from 0 to 9
	}while(rand2 == rand1);
	sp[rand1]=1;
	sp[rand2]=1;

	for(int f=1; f<=10; f++){
		for(int e=1; e<=8; e++){

			if(sp[f-1] == 1){
				XTft_DrawSolidBox(&TftInstance, 60+D+(f-1)*(D+BRICK_WIDTH), 60+(e-1)*(BRICK_HEIGHT+D)+D, 60+D + BRICK_WIDTH+(f-1)*(D+BRICK_WIDTH),
						60+(e-1)*(BRICK_HEIGHT+D)+BRICK_HEIGHT+D, 0x00ffff00);
			}

			else{
				XTft_DrawSolidBox(&TftInstance, 60+D+(f-1)*(D+BRICK_WIDTH), 60+(e-1)*(BRICK_HEIGHT+D)+D, 60+D + BRICK_WIDTH+(f-1)*(D+BRICK_WIDTH),
						60+(e-1)*(BRICK_HEIGHT+D)+BRICK_HEIGHT+D, 0x00ffffff);
			}
		}
	}

	//All bricks alive initially
	for(int i=0; i<8; i=i+1){
		for(int j=0; j<10; j=j+1){
			bricks[i][j]=1; //Set alive
		}
	}

	return;
}

void init_score_zone(){
	//Backgrounds for score zone
		XTft_DrawSolidBox(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 10, 60 + BRICK_ZONE_ROWS/6 - 12,
					      60 + BRICK_ZONE_COLUMNS + 90, 60 + BRICK_ZONE_ROWS/6 + 20, 0x0000ffff);
		XTft_DrawSolidBox(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 10, 60 + BRICK_ZONE_ROWS/3 - 12,
						  60 + BRICK_ZONE_COLUMNS + 90, 60 + BRICK_ZONE_ROWS/3 + 20, 0x0000ffff);
		XTft_DrawSolidBox(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 10, 60 + BRICK_ZONE_ROWS/2 - 12,
							  60 + BRICK_ZONE_COLUMNS + 90, 60 + BRICK_ZONE_ROWS/2 + 20, 0x0000ffff);
		XTft_DrawSolidBox(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 10, 60 + 2*BRICK_ZONE_ROWS/3 - 12,
								  60 + BRICK_ZONE_COLUMNS + 90, 60 + 2*BRICK_ZONE_ROWS/3 + 20, 0x0000ffff);
		XTft_DrawSolidBox(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 10, 60 + 5*BRICK_ZONE_ROWS/6 - 12,
								  60 + BRICK_ZONE_COLUMNS + 90, 60 + 5*BRICK_ZONE_ROWS/6 + 20, 0x0000ffff);

		// Text for score zone
		XTft_SetPos(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 15, 60 + BRICK_ZONE_ROWS/6);
		XTft_SetColor(&TftInstance, 0x00000000, 0x0000ffff);
		XTft_Write(&TftInstance,'S');
		XTft_Write(&TftInstance,'c');
		XTft_Write(&TftInstance,'o');
		XTft_Write(&TftInstance,'r');
		XTft_Write(&TftInstance,'e');
		XTft_Write(&TftInstance,':');
		XTft_SetPos(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 15, 60 + BRICK_ZONE_ROWS/3);
		XTft_Write(&TftInstance,'T');
		XTft_Write(&TftInstance,'i');
		XTft_Write(&TftInstance,'m');
		XTft_Write(&TftInstance,'e');
		XTft_Write(&TftInstance,':');
		XTft_SetPos(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 15, 60 + BRICK_ZONE_ROWS/2);
		XTft_Write(&TftInstance,'S');
		XTft_Write(&TftInstance,'p');
		XTft_Write(&TftInstance,'e');
		XTft_Write(&TftInstance,'e');
		XTft_Write(&TftInstance,'d');
		XTft_Write(&TftInstance,':');
		XTft_SetPos(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 15, 60 + 2*BRICK_ZONE_ROWS/3);
		XTft_Write(&TftInstance,'B');
		XTft_Write(&TftInstance,'r');
		XTft_Write(&TftInstance,'i');
		XTft_Write(&TftInstance,'x');
		XTft_Write(&TftInstance,':');
		XTft_SetPos(&TftInstance, 60 + BRICK_ZONE_COLUMNS + 15, 60 + 5*BRICK_ZONE_ROWS/6);
		XTft_Write(&TftInstance,'F');
		XTft_Write(&TftInstance,'P');
		XTft_Write(&TftInstance,'S');
		XTft_Write(&TftInstance,':');

		return;
}

void value_display(unsigned int value, unsigned int x_pos, unsigned int y_pos) {
	unsigned int value_1 = value/100;
	unsigned int value_2 = (value - value_1*100)/10;
	unsigned int value_3 = (value - value_1*100)%10;

	XTft_SetPos(&TftInstance, x_pos, y_pos);
	XTft_SetColor(&TftInstance, 0x00000000, 0x0000ffff);

	XTft_Write(&TftInstance,(char)((int)('0') + value_1));
	XTft_Write(&TftInstance,(char)((int)('0') + value_2));
	XTft_Write(&TftInstance,(char)((int)('0') + value_3));

	return;
}

void check_column(int j){

	pthread_mutex_lock(&swmutex);
	//Check hit with bricks
	//If not inside column space. Return no hit

	if((response.x < 60+1 + j*(BRICK_WIDTH+D)+R)&&
			(response.x > 60-1 + D+(j-1)*(BRICK_WIDTH+D)-R)&&
			(response.y < 60+1 + 8*(BRICK_HEIGHT+D)+ R)){

		//Else check all bricks
		for(int i = 1; i <= 8; i++){
			if(brick_alive(i,j)){
				if((response.y > 60-1+(i-1)*BRICK_HEIGHT+i*D-R) &&
						(response.y<60+1+i*(BRICK_HEIGHT+D)+R)){ //Above && Below

					//From below
					if((response.y>60+1+i*(BRICK_HEIGHT+D))&&
							(response.x<60+1+j*(BRICK_WIDTH+D)+R )&&
							(response.x > 60-1+D+(j-1)*(BRICK_WIDTH+D)-R)){// Kommer underifrån
						response.y = 2*(60+1+ i*(BRICK_HEIGHT+D) + R) - response.y;
						response.unchanged=0;
						increase_score(j);
						remove_brick(i,j);
						if(response.angle > 0) {
							response.angle = M_PI - response.angle;
						}
						else {
							response.angle = -M_PI - response.angle;
						}
						break;
					}

					//From above
					else if((response.y<60-1 + i*(BRICK_HEIGHT+D) - BRICK_HEIGHT -R)&&
							(response.x<60+1+j*(BRICK_WIDTH+D)+R )&&
							(response.x >60-1+ D+(j-1)*(BRICK_WIDTH+D)-R)){
						response.y = 2*(60-1 + i*(BRICK_HEIGHT+D) - BRICK_HEIGHT - R) - response.y;
						response.unchanged=0;
						increase_score(j);
						remove_brick(i,j);
						if (response.angle > 0) {
							response.angle = M_PI - response.angle;
						}
						else {
							response.angle = -M_PI - response.angle;
						}
						break;
					}

					//From Right
					else if((response.x<60+1+j*(BRICK_WIDTH+D)+R )&&
							(response.x>60-1+j*(BRICK_WIDTH+D))){
						response.x = 2*(60+1+j*(BRICK_WIDTH+D) + R) -response.x;
						response.angle = -response.angle;
						response.unchanged=0;
						increase_score(j);
						remove_brick(i,j);
						break;
					}

					//From Left
					else if((response.x >60-1+ D+(j-1)*(BRICK_WIDTH+D)-R)&&
							(response.x < 60+1+D+(j-1)*(BRICK_WIDTH+D))){
						response.x = 2*(60-1+j*(BRICK_WIDTH+D)-BRICK_WIDTH-R) - response.x;
						response.angle = -response.angle;
						response.unchanged=0;
						increase_score(j);
						remove_brick(i,j);
						break;
					}
				}
			}
		}
	}
	pthread_mutex_unlock(&swmutex);
	return;
}

void increase_score(int j){
	if(sp[j-1]==1){
		score=score+2;//Double points
		if(score%10 == 0 || score%10 == 1 ){
			update_special_columns();
		}
	}
	else{
		score=score+1;//Single point
		if(score%10 == 0){
			update_special_columns();
		}
	}
	return;
}

void update_special_columns(){

	//Old
	if(columns_alive[rand1]){
		sp[rand1]=-1;
	}
	else{
		sp[rand1]=0; //For efficiency, don't condisder when updating golden bars in the future
	}
	if(columns_alive[rand2]){
		sp[rand2]=-1;
	}
	else{
		sp[rand2]=0; //For efficiency, don't condisder when updating golden bars in the future
	}

	//New
	if(nr_columns_alive()>0){ //Not necessairly later
		do{
			rand1=(int)floor(9.999*(double)rand() / (double)RAND_MAX); // from 0 to 9
		}while(columns_alive[rand1] == 0); //New rand1 while column is dead
		sp[rand1]=1;
	}

	if(nr_columns_alive()>1){
		do{
			rand2=(int)floor(9.999*(double)rand() / (double)RAND_MAX); // from 0 to 9
		}while(rand2 == rand1 || columns_alive[rand2] == 0); //Do while same as rand1 or column is dead
		sp[rand2]=1;
	}

	return;
}

// Only updates the column that have become yellow or turned from yellow to white
void update_columns(){

	for(int f=1; f<=10; f++){ //All columns
		if(sp[f-1]==1){//Yellow
			for(int e=1; e<=8; e++){ //All rows
				if(brick_alive(e,f)){
					XTft_DrawSolidBox(&TftInstance, 60+D+(f-1)*(D+BRICK_WIDTH), 60+(e-1)*(BRICK_HEIGHT+D)+D, 60+D + BRICK_WIDTH+(f-1)*(D+BRICK_WIDTH),
							60+(e-1)*(BRICK_HEIGHT+D)+BRICK_HEIGHT+D, 0x00ffff00);
				}
			}
		}
		else if(sp[f-1]==-1){//White
			for(int e=1; e<=8; e++){ //All rows
				if(brick_alive(e,f)){
					XTft_DrawSolidBox(&TftInstance, 60+D+(f-1)*(D+BRICK_WIDTH), 60+(e-1)*(BRICK_HEIGHT+D)+D, 60+D + BRICK_WIDTH+(f-1)*(D+BRICK_WIDTH),
							60+(e-1)*(BRICK_HEIGHT+D)+BRICK_HEIGHT+D, 0x00ffffff);
					// Indicate that column has been painted white again
					sp[f-1] = 0;
				}
			}
		}
	}
	return;
}

void remove_brick(int i,int j){
	bricks[i-1][j-1]=0;
	bricks_left--;


	//Draw green
	XTft_DrawSolidBox(&TftInstance, 60+D+(j-1)*(D+BRICK_WIDTH), 60+(i-1)*(BRICK_HEIGHT+D)+D, 60+D + BRICK_WIDTH+(j-1)*(D+BRICK_WIDTH),
	60+(i-1)*(BRICK_HEIGHT+D)+BRICK_HEIGHT+D, 0x0000ff00);

	check_column_status(j-1);

	return;
}

int brick_alive(int i, int j){
	if(bricks[i-1][j-1]==1){
		return 1;
	}
	else{
		return 0;
	}
}

//Checks status of column after hit
void check_column_status(int j){ //j from 0 to 9
	int dead=1;

	for(int i=0; i<8; i++){
		if(bricks[i][j]==1){
			dead=0;
			break;	//Breaks if not dead
		}
	}

	if(dead){ //Is dead, kill
		columns_alive[j]=0;
		//KILL THREAD
		//pthread_exit(NULL);
	}
	return;
}

//Return number of columns alive
int nr_columns_alive(){

int nr_columns_alive=0;
	for(int index=0; index < 10; index ++){
		if(columns_alive[index]){
			nr_columns_alive=nr_columns_alive+1;;
		}
	}
return nr_columns_alive;
}
