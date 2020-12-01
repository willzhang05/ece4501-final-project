// Main.c
// Runs on LM4F120/TM4C123
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// Jonathan W. Valvano 2/20/17, valvano@mail.utexas.edu
// Modified by Sile Shu 10/4/17, ss5de@virginia.edu
// Modified by Mustafa Hotaki 7/29/18, mkh3cf@virginia.edu

#include <stdint.h>
#include "OS.h"
#include "tm4c123gh6pm.h"
#include "LCD.h"
#include <string.h> 
#include "UART.h"
#include "FIFO.h"
#include "joystick.h"
#include "PORTE.h"

// Constants
#define BGCOLOR     					LCD_BLACK
#define CROSSSIZE            	5
#define PERIOD               	4000000   // DAS 20Hz sampling period in system time units
#define PSEUDOPERIOD         	8000000
#define LIFETIME             	1000
#define RUNLENGTH            	600 // 30 seconds run length

extern Sema4Type LCDFree;
uint16_t origin[2]; 	// The original ADC value of x,y if the joystick is not touched, used as reference
int16_t x = 63;  			// horizontal position of the crosshair, initially 63
int16_t y = 63;  			// vertical position of the crosshair, initially 63
int16_t prevx, prevy;	// Previous x and y values of the crosshair
uint8_t select;  			// joystick push
uint8_t area[2];

#define HORIZONAL_NUM_BLOCKS 6
#define VERTICAL_NUM_BLOCKS 6
#define NUM_CUBES 4

// #define DEBUG

enum Direction {
	UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3
};

struct Cube {
	uint8_t x;
	uint8_t y;
	enum Direction dir;
	uint8_t dead;
	uint16_t color;
	uint16_t life;
};

struct Cube cubes[NUM_CUBES];

uint8_t used_blocks[VERTICAL_NUM_BLOCKS][HORIZONAL_NUM_BLOCKS];

unsigned long NumCreated;   		// Number of foreground threads created
unsigned long UpdateWork;   		// Incremented every update on position values
unsigned long Calculation;  		// Incremented every cube number calculation
unsigned long DisplayCount; 		// Incremented every time the Display thread prints on LCD 
unsigned long ConsumerCount;		// Incremented every time the Consumer thread prints on LCD
unsigned long Button1RespTime; 	// Latency for Task 2 = Time between button1 push and response on LCD 
unsigned long Button2RespTime; 	// Latency for Task 7 = Time between button2 push and response on LCD
unsigned long Button1PushTime; 	// Time stamp for when button 1 was pushed
unsigned long Button2PushTime; 	// Time stamp for when button 2 was pushed

//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};
unsigned long TotalWithI1;
unsigned short MaxWithI1;

unsigned long Score;
unsigned long Life;

static uint32_t rand = 21269;

uint32_t get_rand() {
	rand = rand * rand + rand/2;
	return rand;
}

enum Direction get_random_direction() {
	return (enum Direction)(get_rand() % 4);
}

Sema4Type NeedCubeRedraw;
Sema4Type CubeLock;

// Must have CubeLock when calling
int get_movable_directions(struct Cube *cube, int8_t *dirs) {
	int total = 0;
	if (cube->x > 0 && !used_blocks[cube->y][cube->x - 1]) {
		total += 1;
		dirs[LEFT] = 1;
	} else {
		dirs[LEFT] = 0;
	}
	if (cube->x < HORIZONAL_NUM_BLOCKS - 1 && !used_blocks[cube->y][cube->x + 1]) {
		total += 1;
		dirs[RIGHT] = 1;
	} else {
		dirs[RIGHT] = 0;
	}
	if (cube->y > 0 && !used_blocks[cube->y - 1][cube->x]) {
		total += 1;
		dirs[UP] = 1;
	} else {
		dirs[UP] = 0;
	}
	if (cube->y < VERTICAL_NUM_BLOCKS - 1 && !used_blocks[cube->y + 1][cube->x]) {
		total += 1;
		dirs[DOWN] = 1;
	} else {
		dirs[DOWN] = 0;
	}
	return total;
}

void Fatal(char *msg, char *msg2) {
	BSP_LCD_DrawString(0,0,"FATAL ERROR:", LCD_RED);
	BSP_LCD_DrawString(0,1, msg, LCD_RED);
	BSP_LCD_DrawString(0,2, msg2, LCD_RED);
	while (1);
}

// Must have CubeLock when calling
void MoveCube(struct Cube *cube) {
	int8_t valid_directions[4];
	int dir_to_move_num;
	int i;
	int total_valid_dirs = get_movable_directions(cube, valid_directions);
	if (!total_valid_dirs) {
		cube->color = LCD_YELLOW;
		return;
	}
	
	if (valid_directions[cube->dir]) {
		cube->color = LCD_WHITE;
	} else {
		cube->color = LCD_RED;
		dir_to_move_num = get_rand() % total_valid_dirs;
		
		for (i = 0; i < 4; ++i) {
			if (!valid_directions[i]) continue;
			if (--dir_to_move_num < 0) {
				cube->dir = (enum Direction)i;
				break;
			}
		}
		if (i == 4) {
			Fatal("Couldn't find dir", "");
		}
	}
	
	used_blocks[cube->y][cube->x] = 0;
	
	switch(cube->dir) {
		case UP:
			cube->y -= 1;
			break;
		case DOWN:
			cube->y += 1;
			break;
		case LEFT:
			cube->x -= 1;
			break;
		case RIGHT:
			cube->x += 1;
			break;
	}
	
	used_blocks[cube->y][cube->x] = 1;
}

#define MAX_CUBE_LIFETIME 10

void InitCubes(int num_cubes) {
	int y, x, i;
	// initialize data structures
	for (y = 0; y < VERTICAL_NUM_BLOCKS; ++y) {
		for (x = 0; x < HORIZONAL_NUM_BLOCKS; ++x) {
			used_blocks[y][x] = 0; // free
		}
	}
	#ifdef DEBUG
	BSP_LCD_Message(0, 2, 0, "Making cubes: ", num_cubes);
	#endif
  for (i = 0; i < num_cubes; ++i) {
		uint8_t x = 0;
		uint8_t y = 0;
		uint8_t attempt = 0;
		do {
			x = get_rand() % HORIZONAL_NUM_BLOCKS;
			y = get_rand() % VERTICAL_NUM_BLOCKS;
			#ifdef DEBUG
			BSP_LCD_Message(0, 4, 0, "attempt: ", attempt);
			BSP_LCD_Message(0, 5, 0, "x: ", x);
			BSP_LCD_Message(0, 6, 0, "y: ", y);
			OS_Sleep(50);
			#endif
			if (attempt++ > 5) { Fatal("Ran out of attempts", "RNG is broken"); }
		}
		while (used_blocks[y][x]); // keep picking new coordinates until the position is free
		used_blocks[y][x] = 1;
		cubes[i].x = x;
		cubes[i].y = y;
		cubes[i].dead = 0;
		cubes[i].dir = get_random_direction();
		cubes[i].color = LCD_WHITE;
		cubes[i].life = 1 + (get_rand() % (MAX_CUBE_LIFETIME - 1));
	}
}

static const uint16_t block_width = 18;
static const uint16_t block_height = 18;


// Must hold CubeLock
void ClearBlocksLCD(void){
	int i;
	OS_bWait(&LCDFree);
	for (i = 0; i < NUM_CUBES; ++i) {
		int16_t px, py, w, h;
		if (cubes[i].dead) continue;
		px = cubes[i].x * block_width;
		py = cubes[i].y * block_height;
		w = block_width;
		h = block_height;
		BSP_LCD_FillRect(px, py, w, h, LCD_BLACK);
	}
	OS_bSignal(&LCDFree);
}

void InitAndMoveBlocks(void){
	int i;
	OS_InitSemaphore(&CubeLock, 0);
	InitCubes(NUM_CUBES);
	while (1) {
		int num_alive = 0;
		for (i = 0; i < NUM_CUBES; ++i) {
			if (cubes[i].dead) continue;
			MoveCube(&cubes[i]);
		}
		OS_bSignal(&CubeLock);
		OS_bSignal(&NeedCubeRedraw);
		OS_Sleep(1000);
		OS_bWait(&CubeLock);
		for (i = 0; i < NUM_CUBES; ++i) {
			if (cubes[i].dead) continue;
			cubes[i].life--;
			num_alive++;
		}
		ClearBlocksLCD();
		if (!num_alive) {
			InitCubes(1 + (get_rand() % 5));
		}
		for (i = 0; i < NUM_CUBES; ++i) {
			if (cubes[i].dead) continue;
			if (cubes[i].life == 0) {
				cubes[i].dead = 1;
			}
		}
	}
  OS_Kill();  // done
}


void DrawBlocks(void){
	while (1) {
		int i;
		OS_bWait(&NeedCubeRedraw);
		OS_bWait(&LCDFree);
		OS_bWait(&CubeLock);
		// BSP_LCD_FillRect(0, 0, block_width * HORIZONAL_NUM_BLOCKS, block_height * VERTICAL_NUM_BLOCKS, LCD_BLACK);
		for (i = 0; i < NUM_CUBES; ++i) {
			int16_t px, py, w, h;
			if (cubes[i].dead) continue;
			px = cubes[i].x * block_width;
			py = cubes[i].y * block_height;
			w = block_width;
			h = block_height;
			BSP_LCD_FillRect(px, py, w, h, cubes[i].color);
		}
		OS_bSignal(&CubeLock);
		OS_bSignal(&LCDFree);
		OS_Suspend();
	}
  OS_Kill();  // done
}

void Device_Init(void){
	UART_Init();
	BSP_LCD_OutputInit();
	BSP_Joystick_Init();
}
//------------------Task 1--------------------------------
// background thread executed at 20 Hz
//******** Producer *************** 
int UpdatePosition(uint16_t rawx, uint16_t rawy, jsDataType* data){
	if (rawx > origin[0]){
		x = x + ((rawx - origin[0]) >> 9);
	}
	else{
		x = x - ((origin[0] - rawx) >> 9);
	}
	if (rawy < origin[1]){
		y = y + ((origin[1] - rawy) >> 9);
	}
	else{
		y = y - ((rawy - origin[1]) >> 9);
	}
	if (x > 127){
		x = 127;}
	if (x < 0){
		x = 0;}
	if (y > 112 - CROSSSIZE){
		y = 112 - CROSSSIZE;}
	if (y < 0){
		y = 0;}
	data->x = x; data->y = y;
	return 1;
}

void Producer(void){
	uint16_t rawX,rawY; // raw adc value
	uint8_t select;
	jsDataType data;
	unsigned static long LastTime;  // time at previous ADC sample
	unsigned long thisTime;         // time at current ADC sample
	long jitter;                    // time between measured and expected, in us
	BSP_Joystick_Input(&rawX,&rawY,&select);
	thisTime = OS_Time();       // current time, 12.5 ns
	UpdateWork += UpdatePosition(rawX,rawY,&data); // calculation work
	if(JsFifo_Put(data) == 0){ // send to consumer
		DataLost++;
	}
//calculate jitter
	if(UpdateWork > 1){    // ignore timing of first interrupt
		unsigned long diff = OS_TimeDifference(LastTime,thisTime);
		if(diff > PERIOD){
			jitter = (diff-PERIOD+4)/8;  // in 0.1 usec
		}
		else{
			jitter = (PERIOD-diff+4)/8;  // in 0.1 usec
		}
		if(jitter > MaxJitter){
			MaxJitter = jitter; // in usec
		}       // jitter should be 0
		if(jitter >= JitterSize){
			jitter = JITTERSIZE-1;
		}
		JitterHistogram[jitter]++; 
	}
	LastTime = thisTime;
	OS_Suspend();
}

//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
	uint32_t StartTime,CurrentTime,ElapsedTime;
	StartTime = OS_MsTime();
	ElapsedTime = 0;
	OS_bWait(&LCDFree);
	Button1RespTime = OS_MsTime() - Button1PushTime; // LCD Response here
	BSP_LCD_FillScreen(BGCOLOR);
	//Button1FuncTime = OS_MsTime() - Button1PushTime;
	//Button1PushTime = 0;
	while (ElapsedTime < LIFETIME){
		CurrentTime = OS_MsTime();
		ElapsedTime = CurrentTime - StartTime;
		BSP_LCD_Message(0,5,0,"Life Time:",LIFETIME);
		BSP_LCD_Message(1,0,0,"Horizontal Area:",area[0]);
		BSP_LCD_Message(1,1,0,"Vertical Area:",area[1]);
		BSP_LCD_Message(1,2,0,"Elapsed Time:",ElapsedTime);
		OS_Sleep(50);

	}
	BSP_LCD_FillScreen(BGCOLOR);
	OS_bSignal(&LCDFree);
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
  if(OS_MsTime() > 20 ){ // debounce
    if(OS_AddThread(&ButtonWork,128,4)){
			OS_ClearMsTime();
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
		Button1PushTime = OS_MsTime(); // Time stamp

  }
}

//--------------end of Task 2-----------------------------


//------------------Task 3--------------------------------

//******** Consumer *************** 
// foreground thread, accepts data from producer
// Display crosshair and its positions
// inputs:  none
// outputs: none
void Consumer(void){
	while (1) {
		jsDataType data;
		JsFifo_Get(&data);
		OS_bSignal(&NeedCubeRedraw);
		OS_bWait(&LCDFree);
			
		BSP_LCD_DrawCrosshair(prevx, prevy, LCD_BLACK); // Draw a black crosshair
		BSP_LCD_DrawCrosshair(data.x, data.y, LCD_RED); // Draw a red crosshair

		BSP_LCD_Message(1, 5, 3, "Score: ", Score);		
		BSP_LCD_Message(1, 5, 12, "Life: ", Life);
		ConsumerCount++;
		OS_bSignal(&LCDFree);
		prevx = data.x; 
		prevy = data.y;
		OS_Suspend();
	}
  OS_Kill();  // done
}

//------------------Task 7--------------------------------
// background thread executes with button2
// one foreground task created with button push
// ***********ButtonWork2*************
void Restart(void){
	uint32_t StartTime,CurrentTime,ElapsedTime;
	OS_Sleep(50); // wait
	StartTime = OS_MsTime();
	ElapsedTime = 0;
	OS_bWait(&LCDFree);
	Button2RespTime = OS_MsTime() - Button2PushTime; // Response on LCD here
	BSP_LCD_FillScreen(BGCOLOR);
	while (ElapsedTime < 500){
		CurrentTime = OS_MsTime();
		ElapsedTime = CurrentTime - StartTime;
		BSP_LCD_DrawString(5,6,"Restarting",LCD_WHITE);
	}
	BSP_LCD_FillScreen(BGCOLOR);
	
	// restart
	DataLost = 0;        // lost data between producer and consumer
  UpdateWork = 0;
	MaxJitter = 0;       // in 1us units
	Score = 0;
	Life = 0;
	x = 63; y = 63;
	
	OS_bSignal(&LCDFree);
	
	InitCubes(NUM_CUBES);
	
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW2Push*************
// Called when Button2 pushed
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  if(OS_MsTime() > 20 ){ // debounce
    if(OS_AddThread(&Restart,128,4)){
			OS_ClearMsTime();
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
		Button2PushTime = OS_MsTime(); // Time stamp
  }
}

//--------------end of Task 7-----------------------------

// Fill the screen with the background color
// Grab initial joystick position to bu used as a reference
void CrossHair_Init(void){
	BSP_LCD_FillScreen(BGCOLOR);
	BSP_Joystick_Input(&origin[0],&origin[1],&select);
}

//******************* Main Function**********
int main(void){ 
  OS_Init();           // initialize, disable interrupts
	Device_Init();
  CrossHair_Init();
  DataLost = 0;        // lost data between producer and consumer
  MaxJitter = 0;       // in 1us units
	Score = 0;
	Life = 0;

//********initialize communication channels
  JsFifo_Init();

//*******attach background tasks***********
  OS_AddSW1Task(&SW1Push, 4);
	OS_AddSW2Task(&SW2Push, 4);
  OS_AddPeriodicThread(&Producer, PERIOD, 3); // 2 kHz real time sampling of PD3
	
  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&Consumer, 128, 1); 
  NumCreated += OS_AddThread(&InitAndMoveBlocks, 128, 1); 
  NumCreated += OS_AddThread(&DrawBlocks, 128, 3); 
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
	return 0;            // this never executes
}
