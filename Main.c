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
#define BGCOLOR LCD_BLACK
#define CROSSSIZE 5
#define PERIOD 4000000  // DAS 20Hz sampling period in system time units
#define PSEUDOPERIOD 8000000
#define LIFETIME 1000
#define RUNLENGTH 600  // 30 seconds run length

extern Sema4Type LCDFree;
uint16_t
    origin[2];   // The original ADC value of x,y if the joystick is not touched, used as reference
int16_t x = 63;  // horizontal position of the crosshair, initially 63
int16_t y = 63;  // vertical position of the crosshair, initially 63
int16_t prevx, prevy;  // Previous x and y values of the crosshair
uint8_t select;        // joystick push
uint8_t area[2];

#define HORIZONAL_NUM_BLOCKS 6
#define VERTICAL_NUM_BLOCKS 6
#define NUM_CUBES 5
#define SLEEP_TIME 500
#define MAX_CUBE_LIFETIME 20
#define DEFAULT_LIFE 5
#define MAX_ATTEMPTS 50  // for cube placement

#define POLY_MASK_32 0xB4BCD35C
#define POLY_MASK_31 0x7A5BC2E3

// #define DEBUG_CUBE_COLOR

// #define DEBUG
// #define DEBUG_V

enum Direction { UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3 };
enum PowerUp { NONE = 0, LIFE = 1 };

struct Cube {
    uint8_t x;
    uint8_t y;
    enum Direction dir;
    uint8_t dead;
    uint16_t color;
    uint16_t life;
    Sema4Type sem;
	  enum PowerUp powerup;
};

struct Cube cubes[NUM_CUBES];

Sema4Type blocks[VERTICAL_NUM_BLOCKS][HORIZONAL_NUM_BLOCKS];

unsigned long NumCreated;     // Number of foreground threads created
unsigned long UpdateWork;     // Incremented every update on position values
unsigned long Calculation;    // Incremented every cube number calculation
unsigned long DisplayCount;   // Incremented every time the Display thread prints on LCD
unsigned long ConsumerCount;  // Incremented every time the Consumer thread prints on LCD
unsigned long
    Button1RespTime;  // Latency for Task 2 = Time between button1 push and response on LCD
unsigned long
    Button2RespTime;  // Latency for Task 7 = Time between button2 push and response on LCD
unsigned long Button1PushTime;  // Time stamp for when button 1 was pushed
unsigned long Button2PushTime;  // Time stamp for when button 2 was pushed

//---------------------User debugging-----------------------
unsigned long DataLost;  // data sent by Producer, but not received by Consumer
long MaxJitter;          // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize = JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE] = {
    0,
};
unsigned long TotalWithI1;
unsigned short MaxWithI1;

unsigned long Score;
unsigned long Life;

static uint32_t lfsr32;
static uint32_t lfsr31;

int shift_lfsr(uint32_t *lfsr, uint32_t poly_mask) {
    int feedback = *lfsr & 1;
    *lfsr >>= 1;
    if (feedback == 1) {
        *lfsr ^= poly_mask;
    }
    return *lfsr;
}

void init_lfsrs(uint32_t x, uint32_t y) {
    lfsr32 = x;
    lfsr31 = y;
}

// Joystick-based PRNG. Can grab bits 4-7 from rawX and rawY to cut BSP_Joystick_Input() calls by
// half if needed
uint32_t get_rand(void) {
    shift_lfsr(&lfsr32, POLY_MASK_32);
    return (shift_lfsr(&lfsr32, POLY_MASK_32) ^ shift_lfsr(&lfsr31, POLY_MASK_31)) & 0xFFFF;
}

enum Direction get_random_direction() { return (enum Direction)(get_rand() % 4); }

Sema4Type NeedCubeRedraw;
Sema4Type MoveCubesSem;
Sema4Type DoneMovingCubesSem;
Sema4Type ThrottleSem;
Sema4Type CubeDrawing;
Sema4Type InfoSem;
Sema4Type DoneSem;
Sema4Type MoveWaitSem;

int CheckLife(void) {
    int res;
    OS_bWait(&InfoSem);
    res = Life;
    OS_bSignal(&InfoSem);
    return res;
}
static uint32_t restarting = 0;
static uint32_t scoring = 0;
Sema4Type ResSem;

int CheckRestarting() {
    int res;
    OS_bWait(&ResSem);
    res = restarting;
    OS_bSignal(&ResSem);
    return res;
}
int CheckScoring() {
    int res;
    OS_bWait(&ResSem);
    res = scoring;
    OS_bSignal(&ResSem);
    return res;
}

// Must have CubeLock when calling
int get_movable_directions(struct Cube *cube, int8_t *dirs) {
    int total = 0;
    if (cube->x > 0 && blocks[cube->y][cube->x - 1].Value) {
        total += 1;
        dirs[LEFT] = 1;
    } else {
        dirs[LEFT] = 0;
    }
    if (cube->x < HORIZONAL_NUM_BLOCKS - 1 && blocks[cube->y][cube->x + 1].Value) {
        total += 1;
        dirs[RIGHT] = 1;
    } else {
        dirs[RIGHT] = 0;
    }
    if (cube->y > 0 && blocks[cube->y - 1][cube->x].Value) {
        total += 1;
        dirs[UP] = 1;
    } else {
        dirs[UP] = 0;
    }
    if (cube->y < VERTICAL_NUM_BLOCKS - 1 && blocks[cube->y + 1][cube->x].Value) {
        total += 1;
        dirs[DOWN] = 1;
    } else {
        dirs[DOWN] = 0;
    }
    return total;
}

void Fatal(char *msg, char *msg2) {
    BSP_LCD_DrawString(0, 0, "FATAL ERROR:", LCD_RED);
    BSP_LCD_DrawString(0, 1, msg, LCD_RED);
    BSP_LCD_DrawString(0, 2, msg2, LCD_RED);
    while (1)
        ;
}

static const uint16_t block_width = 18;
static const uint16_t block_height = 18;

void ClearBlockLCD(struct Cube *cube, char *msg) {
    int16_t px, py, w, h;
    OS_bWait(&LCDFree);
    if (cube->dead) Fatal("Called ClearBlockLCD", msg);
    px = cube->x * block_width;
    py = cube->y * block_height;
    w = block_width;
    h = block_height;
    BSP_LCD_FillRect(px, py, w, h, LCD_BLACK);
    OS_bSignal(&LCDFree);
}
void KillCube(struct Cube *cube) {
    cube->dead = 1;
    OS_bSignal(&blocks[cube->y][cube->x]);
}

Sema4Type reset_crosshair_sem;
void ResetCrosshairSize() {
}

void HandlePowerUp(struct Cube *cube) {
	switch(cube->powerup) {
		case NONE:
			break;
		case LIFE:
			Life += 1;
			break;
	}
}

int CheckBlockIntersection(struct Cube *cube) {
    int px, py;
    px = cube->x * block_width;
    py = cube->y * block_height;
    if (x + 4 >= px && x - 4 <= px + block_width) {
        if (y + 4 >= py && y - 4 <= py + block_height) {
            ClearBlockLCD(cube, "CheckInt");
            KillCube(cube);
            OS_bWait(&InfoSem);
            Score += 1;
					  HandlePowerUp(cube);
            OS_bSignal(&InfoSem);
            return 1;
        }
    }
    return 0;
}

void MoveCube(struct Cube *cube) {
    int8_t valid_directions[4];
    int dir_to_move_num;
    int new_x, new_y;
    int i;
    int total_valid_dirs;
    int32_t status;
    if (cube->dead) return;
    status = StartCritical();
    total_valid_dirs = get_movable_directions(cube, valid_directions);
    if (!total_valid_dirs) {
#ifdef DEBUG_CUBE_COLOR
        cube->color = LCD_YELLOW;
#endif
        EndCritical(status);
        return;
    }

    if (valid_directions[cube->dir]) {
#ifdef DEBUG_CUBE_COLOR
        cube->color = LCD_WHITE;
#endif
    } else {
#ifdef DEBUG_CUBE_COLOR
        cube->color = LCD_RED;
#endif
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

    new_x = cube->x;
    new_y = cube->y;

    switch (cube->dir) {
        case UP:
            new_y -= 1;
            break;
        case DOWN:
            new_y += 1;
            break;
        case LEFT:
            new_x -= 1;
            break;
        case RIGHT:
            new_x += 1;
            break;
    }
    OS_bWait(&blocks[new_y][new_x]);  // this should never block
    OS_bSignal(&blocks[cube->y][cube->x]);
    EndCritical(status);
    // ClearBlockLCD(cube);
    cube->y = new_y;
    cube->x = new_x;
}

Sema4Type CheckIntSem;
int CheckIntOk = 0;

int CanCheckIntersectionAndHold() {
    OS_bWait(&CheckIntSem);
    if (!CheckIntOk) {
        OS_bSignal(&CheckIntSem);
        return 0;
    }
    return CheckIntOk;
}

void DecLife() {
    OS_bWait(&InfoSem);
    if (Life) {
      Life--;
			if (!Life) {
				// Game over
        BSP_LCD_FillScreen(BGCOLOR);
        BSP_LCD_DrawString(6, 7, "Game over!", LCD_RED);
        BSP_LCD_DrawString(2, 8, "Press SW1 to save", LCD_WHITE);
        BSP_LCD_DrawString(1, 9, "Press SW2 to restart", LCD_WHITE);
			}
    }
    OS_bSignal(&InfoSem);
}

static int reinit = 0;

void MoveCubeThread(struct Cube *cube) {
    while (CheckLife() > 0 && !cube->dead && !CheckRestarting() && !reinit) {
        while (!cube->dead && !MoveCubesSem.Value && !CheckRestarting() && !reinit) {
            if (CanCheckIntersectionAndHold()) {
                int res;
                OS_bWait(&cube->sem);
                res = CheckBlockIntersection(cube);
                OS_bSignal(&cube->sem);
                OS_bSignal(&CheckIntSem);
                if (res) break;
            }
            OS_Suspend();
        }
        if (reinit || CheckRestarting()) break;
        OS_bWait(&cube->sem);
        if (cube->dead) {
            OS_bSignal(&cube->sem);
            break;
        }
        OS_bSignal(&cube->sem);
        OS_Wait(&MoveCubesSem);
        if (reinit || CheckRestarting()) break;
        OS_bWait(&cube->sem);
        if (!cube->dead) {
            MoveCube(cube);
            cube->life--;
            if (!cube->life) {
                KillCube(cube);
							  DecLife();
            }
        }
        OS_bSignal(&cube->sem);
        OS_Signal(&DoneMovingCubesSem);
        OS_Wait(&ThrottleSem);
    }
    OS_Signal(&MoveWaitSem);
}

void MoveCube0(void) {
    struct Cube *cube = &cubes[0];
    MoveCubeThread(cube);
    OS_Kill();
}
void MoveCube1(void) {
    struct Cube *cube = &cubes[1];
    MoveCubeThread(cube);
    OS_Kill();
}
void MoveCube2(void) {
    struct Cube *cube = &cubes[2];
    MoveCubeThread(cube);
    OS_Kill();
}
void MoveCube3(void) {
    struct Cube *cube = &cubes[3];
    MoveCubeThread(cube);
    OS_Kill();
}
void MoveCube4(void) {
    struct Cube *cube = &cubes[4];
    MoveCubeThread(cube);
    OS_Kill();
}

static int run_once = 0;
static int num_last_created = NUM_CUBES;
void InitCubes(int num_cubes) {
    int y, x, i;
    void (*move_cube[5])(void) = {&MoveCube0, &MoveCube1, &MoveCube2, &MoveCube3, &MoveCube4};
    num_last_created = num_cubes;
    OS_InitSemaphore(&MoveWaitSem, 0);
    OS_InitSemaphore(&CheckIntSem, 1);
    CheckIntOk = 0;
    // initialize data structures
    for (y = 0; y < VERTICAL_NUM_BLOCKS; ++y) {
        for (x = 0; x < HORIZONAL_NUM_BLOCKS; ++x) {
            // if (run_once && blocks[y][x].Value != 1) Fatal("Sema not 1", "");
            OS_InitSemaphore(&blocks[y][x], 1);
        }
    }
    run_once++;
#ifdef DEBUG
    BSP_LCD_Message(0, 2, 0, "Making cubes: ", num_cubes);
#endif
    for (i = 0; i < num_cubes; ++i) {
        uint8_t x = 0;
        uint8_t y = 0;
        uint8_t attempt = 0;
			  int powerup_rand;
        do {
            x = get_rand() % HORIZONAL_NUM_BLOCKS;
            y = get_rand() % VERTICAL_NUM_BLOCKS;
#ifdef DEBUG
            BSP_LCD_Message(0, 4, 0, "attempt: ", attempt);
            BSP_LCD_Message(0, 5, 0, "x: ", x);
            BSP_LCD_Message(0, 6, 0, "y: ", y);
            OS_Sleep(50);
#endif
            if (attempt++ > MAX_ATTEMPTS) {
                Fatal("Ran out of attempts", "RNG is broken");
            }
        } while (!blocks[y][x].Value);  // keep picking new coordinates until the position is free
        OS_bWait(&blocks[y][x]);
        cubes[i].x = x;
        cubes[i].y = y;
        cubes[i].dead = 0;
        cubes[i].dir = get_random_direction();
        cubes[i].life = 1 + (get_rand() % (MAX_CUBE_LIFETIME - 1));
				powerup_rand = get_rand() % 10;
				if (powerup_rand == 0) {
					cubes[i].color = LCD_RED;
					cubes[i].powerup = LIFE;
				} else {
					cubes[i].color = LCD_BLUE;
					cubes[i].powerup = NONE;
				}
        OS_InitSemaphore(&cubes[i].sem, 1);
        OS_AddThread(move_cube[i], 128, 3);
    }
}

void ClearLCDBlocks() {
    BSP_LCD_FillRect(0, 0, HORIZONAL_NUM_BLOCKS * block_width, VERTICAL_NUM_BLOCKS * block_height,
                     LCD_BLACK);
}

void InitAndSyncBlocks(void) {
    int i;
    int8_t live_or_dying_cubes[NUM_CUBES] = {0};
    InitCubes(NUM_CUBES);
    OS_bSignal(&CubeDrawing);
    while (CheckLife() > 0) {
        int num_alive = 0;
        OS_bWait(&CheckIntSem);
        CheckIntOk = 1;
        OS_bSignal(&CheckIntSem);
        OS_bSignal(&NeedCubeRedraw);
        OS_Sleep(SLEEP_TIME);

        OS_bWait(&CheckIntSem);
        CheckIntOk = 0;
        OS_bSignal(&CheckIntSem);

        if (CheckRestarting()) break;

        OS_bWait(&CubeDrawing);

        for (i = 0; i < NUM_CUBES; ++i) {
            OS_bWait(&cubes[i].sem);
            if (cubes[i].dead) {
                live_or_dying_cubes[i] = 0;
                OS_bSignal(&cubes[i].sem);
                continue;
            }
            ClearBlockLCD(&cubes[i], "Premove");
            live_or_dying_cubes[i] = 1;
            OS_bSignal(&cubes[i].sem);
        }

        for (i = 0; i < NUM_CUBES; ++i) {
            if (!live_or_dying_cubes[i]) continue;
            OS_Signal(&MoveCubesSem);
        }
        for (i = 0; i < NUM_CUBES; ++i) {
            if (!live_or_dying_cubes[i]) continue;
            OS_Wait(&DoneMovingCubesSem);
        }
        for (i = 0; i < NUM_CUBES; ++i) {
            if (!live_or_dying_cubes[i]) continue;
            OS_Signal(&ThrottleSem);
        }

        for (i = 0; i < NUM_CUBES; ++i) {
            if (cubes[i].dead) continue;
            num_alive++;
        }
        if (!num_alive) {
            OS_Sleep(500);
            OS_bWait(&ResSem);  // do not allow a restart right now
#ifdef DEBUG_V
            BSP_LCD_DrawString(0, 0, "About to reinitialize", LCD_WHITE);
            OS_Sleep(50);
#endif
            reinit = 1;
            for (i = 0; i < num_last_created; ++i) {
                OS_Signal(&MoveCubesSem);
#ifdef DEBUG_V
                BSP_LCD_DrawString(0, 3 + i, "Waiting... ", LCD_WHITE);
#endif
                OS_Wait(&MoveWaitSem);
#ifdef DEBUG_V
                BSP_LCD_DrawString(10, 3 + i, "Done", LCD_GREEN);
#endif
            }
            reinit = 0;
            // ClearLCDBlocks();
            OS_InitSemaphore(&MoveWaitSem, 0);
            OS_InitSemaphore(&MoveCubesSem, 0);
#ifdef DEBUG_V
            BSP_LCD_DrawString(0, 1, "Done waiting", LCD_WHITE);
            OS_Sleep(50);
#endif
            InitCubes(1 + (get_rand() % 4));
#ifdef DEBUG_V
            BSP_LCD_DrawString(0, 1, "Done reinit", LCD_WHITE);
            OS_Sleep(50);
#endif
            OS_bSignal(&ResSem);
        }
        OS_bSignal(&CubeDrawing);
    }
#ifdef DEBUG
    BSP_LCD_DrawString(0, 9, "MoveBlocks exiting", LCD_WHITE);
#endif
    OS_Signal(&DoneSem);
    OS_Kill();  // done
}

void DrawBlocks(void) {
    while (CheckLife() > 0) {
        int i;
        OS_bWait(&NeedCubeRedraw);
        OS_bWait(&CubeDrawing);
        OS_bWait(&LCDFree);
			  if (!CheckLife()) {
          OS_bSignal(&LCDFree);
          OS_bSignal(&CubeDrawing);
					break;
				}
        // BSP_LCD_FillRect(0, 0, block_width * HORIZONAL_NUM_BLOCKS, block_height *
        // VERTICAL_NUM_BLOCKS, LCD_BLACK);
        for (i = 0; i < NUM_CUBES; ++i) {
            int16_t px, py, w, h;
            if (cubes[i].dead) continue;
            px = cubes[i].x * block_width;
            py = cubes[i].y * block_height;
            w = block_width;
            h = block_height;
            BSP_LCD_FillRect(px, py, w, h, cubes[i].color);
        }
        OS_bSignal(&LCDFree);
        OS_bSignal(&CubeDrawing);
        OS_Suspend();
    }
#ifdef DEBUG
    BSP_LCD_DrawString(0, 10, "DrawBlocks exiting", LCD_WHITE);
#endif
    OS_Signal(&DoneSem);
    OS_Kill();  // done
}

void Device_Init(void) {
    UART_Init();
    BSP_LCD_OutputInit();
    BSP_Joystick_Init();
}
//------------------Task 1--------------------------------
// background thread executed at 20 Hz
//******** Producer ***************
int UpdatePosition(uint16_t rawx, uint16_t rawy, jsDataType *data) {
    if (rawx > origin[0]) {
        x = x + ((rawx - origin[0]) >> 9);
    } else {
        x = x - ((origin[0] - rawx) >> 9);
    }
    if (rawy < origin[1]) {
        y = y + ((origin[1] - rawy) >> 9);
    } else {
        y = y - ((rawy - origin[1]) >> 9);
    }
    if (x > 127) {
        x = 127;
    }
    if (x < 0) {
        x = 0;
    }
    if (y > 112 - CROSSSIZE) {
        y = 112 - CROSSSIZE;
    }
    if (y < 0) {
        y = 0;
    }
    data->x = x;
    data->y = y;
    return 1;
}

void Producer(void) {
    uint16_t rawX, rawY;  // raw adc value
    uint8_t select;
    jsDataType data;
    unsigned static long LastTime;  // time at previous ADC sample
    unsigned long thisTime;         // time at current ADC sample
    long jitter;                    // time between measured and expected, in us
    BSP_Joystick_Input(&rawX, &rawY, &select);
    thisTime = OS_Time();                             // current time, 12.5 ns
    UpdateWork += UpdatePosition(rawX, rawY, &data);  // calculation work
    if (JsFifo_Put(data) == 0) {                      // send to consumer
        DataLost++;
    }
    // calculate jitter
    if (UpdateWork > 1) {  // ignore timing of first interrupt
        unsigned long diff = OS_TimeDifference(LastTime, thisTime);
        if (diff > PERIOD) {
            jitter = (diff - PERIOD + 4) / 8;  // in 0.1 usec
        } else {
            jitter = (PERIOD - diff + 4) / 8;  // in 0.1 usec
        }
        if (jitter > MaxJitter) {
            MaxJitter = jitter;  // in usec
        }                        // jitter should be 0
        if (jitter >= JitterSize) {
            jitter = JITTERSIZE - 1;
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
#define CENTER 64
void HighScore(void) {
	  int let_idx = 0;
    jsDataType data2, data3;
	  char letters[3] = { 'A', 'A', 'A' };
    OS_bWait(&ResSem);
    if (restarting || scoring) {
			if (scoring == 1) scoring = 2;
      OS_bSignal(&ResSem);
      OS_Kill();
			return;
		}
		scoring = 1;
    OS_bSignal(&ResSem);
		OS_bWait(&LCDFree);
		x = CENTER;
		y = CENTER;
    JsFifo_Get(&data3);
    JsFifo_Get(&data2);
		BSP_LCD_FillScreen(BGCOLOR);
    BSP_LCD_DrawString(0, 10, "Press SW1 to save", LCD_WHITE);
		// While SW2 is not pressed a second time
		while (CheckScoring() != 2) {
			  int i;
		    x = CENTER;
		    y = CENTER;
        JsFifo_Get(&data3);
			  if (data3.x - CENTER > 0 && data2.x - CENTER <= 0) {
					if (let_idx < 2)
					  let_idx++;
				} else if (data3.x - CENTER < 0 && data2.x - CENTER >= 0) {
					if (let_idx > 0)
					  let_idx--;
				} else {
					// only update letter if we're not updating let_idx
					letters[let_idx] += (data3.y - CENTER)/3;
					if (letters[let_idx] < 'A') {
						letters[let_idx] = 'Z' - ('A' - letters[let_idx] - 1);
					} else if (letters[let_idx] > 'Z') {
						letters[let_idx] = 'A' + (letters[let_idx] - 'Z' - 1);
					}
				}
				
			// TODO: Update let_idx and letter
			  for (i = 0; i < 3; ++i) {
					int16_t color = LCD_WHITE;
					if (i == let_idx) {
						color = LCD_RED;
					}
					BSP_LCD_DrawChar(30 + i * 20, 25, letters[i], color, LCD_BLACK, 2);
				}
			  // data1 = data2;
			  data2 = data3;
			  OS_Sleep(50);
		}
    OS_bWait(&ResSem);
		scoring = 0;
    OS_bSignal(&ResSem);
		BSP_LCD_DrawString(0, 0, "DONE WITH HS", LCD_WHITE);
    OS_Kill();  // done, OS does not return from a Kill
}

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void) {
    if (OS_MsTime() > 20) {  // debounce
        if (OS_AddThread(&HighScore, 128, 4)) {
            OS_ClearMsTime();
            NumCreated++;
        }
        OS_ClearMsTime();               // at least 20ms between touches
        Button1PushTime = OS_MsTime();  // Time stamp
    }
}

//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------

//******** Consumer ***************
// foreground thread, accepts data from producer
// Display crosshair and its positions
// inputs:  none
// outputs: none
void Consumer(void) {
    while (CheckLife() > 0) {
        jsDataType data;
        JsFifo_Get(&data);
        OS_bSignal(&NeedCubeRedraw);
        OS_bWait(&LCDFree);

        BSP_LCD_DrawCrosshair(prevx, prevy, LCD_BLACK);  // Draw a black crosshair
        BSP_LCD_DrawCrosshair(data.x, data.y, LCD_RED);  // Draw a red crosshair

        OS_bWait(&InfoSem);
        BSP_LCD_Message(1, 5, 0, "Score:", Score);
        BSP_LCD_Message(1, 5, 11, "Life:", Life);
        OS_bSignal(&InfoSem);
        ConsumerCount++;
        OS_bSignal(&LCDFree);
        prevx = data.x;
        prevy = data.y;
        OS_Suspend();
    }
#ifdef DEBUG
    BSP_LCD_DrawString(0, 11, "Consumer exiting", LCD_WHITE);
#endif
    OS_Signal(&DoneSem);
    OS_Kill();  // done
}

//------------------Task 7--------------------------------
// background thread executes with button2
// one foreground task created with button push
// ***********ButtonWork2*************
void Restart(void) {
    uint32_t StartTime, CurrentTime, ElapsedTime, i;
    OS_bWait(&ResSem);
    if (restarting || scoring) {
        OS_bSignal(&ResSem);
        OS_Kill();
        return;
    }
    restarting = 1;
    OS_bSignal(&ResSem);
    OS_Sleep(50);  // wait
    OS_bWait(&InfoSem);
    Life = 0;  // Kill
    OS_bSignal(&InfoSem);
    OS_bSignal(&NeedCubeRedraw);
    StartTime = OS_MsTime();
    ElapsedTime = 0;
    OS_bWait(&LCDFree);
    Button2RespTime = OS_MsTime() - Button2PushTime;  // Response on LCD here
    BSP_LCD_FillScreen(BGCOLOR);
    while (ElapsedTime < 500) {
        CurrentTime = OS_MsTime();
        ElapsedTime = CurrentTime - StartTime;
        BSP_LCD_DrawString(5, 6, "Restarting", LCD_WHITE);
    }
    for (i = 0; i < NUM_CUBES; ++i) {
        OS_Signal(&MoveCubesSem);
        OS_Signal(&ThrottleSem);
    }
    OS_bSignal(&LCDFree);
#ifdef DEBUG
    for (i = 0; i < 3; ++i) {
        BSP_LCD_Message(0, i, 0, "Check", i);
        OS_Wait(&DoneSem);
        BSP_LCD_DrawString(10, i, "Done", LCD_GREEN);
    }
    for (i = 0; i < num_last_created; ++i) {
        OS_Wait(&MoveWaitSem);
    }
    BSP_LCD_DrawString(0, 0, "Restarting!!", LCD_RED);
#else
    for (i = 0; i < 3; ++i) {
        OS_Wait(&DoneSem);
    }
    for (i = 0; i < num_last_created; ++i) {
        OS_Wait(&MoveWaitSem);
    }
#endif
    OS_bWait(&LCDFree);
    BSP_LCD_FillScreen(BGCOLOR);

    // restart
    DataLost = 0;  // lost data between producer and consumer
    UpdateWork = 0;
    MaxJitter = 0;  // in 1us units
    Score = 0;
    Life = DEFAULT_LIFE;
    x = 63;
    y = 63;

    OS_bSignal(&LCDFree);

    OS_InitSemaphore(&MoveCubesSem, 0);
    OS_InitSemaphore(&DoneMovingCubesSem, 0);
    OS_InitSemaphore(&ThrottleSem, 0);
    OS_InitSemaphore(&CubeDrawing, 0);
    OS_InitSemaphore(&NeedCubeRedraw, 0);
    OS_InitSemaphore(&InfoSem, 1);
    OS_InitSemaphore(&reset_crosshair_sem, 1);

    OS_bWait(&ResSem);
    restarting = 0;
    OS_bSignal(&ResSem);

    OS_AddThread(&Consumer, 128, 1);
    OS_AddThread(&InitAndSyncBlocks, 128, 1);
    OS_AddThread(&DrawBlocks, 128, 3);

    OS_Kill();  // done, OS does not return from a Kill
}

//************SW2Push*************
// Called when Button2 pushed
// Adds another foreground task
// background threads execute once and return
void SW2Push(void) {
    if (OS_MsTime() > 20) {  // debounce
        if (OS_AddThread(&Restart, 128, 4)) {
            OS_ClearMsTime();
            NumCreated++;
        }
        OS_ClearMsTime();               // at least 20ms between touches
        Button2PushTime = OS_MsTime();  // Time stamp
    }
}

//--------------end of Task 7-----------------------------

// Fill the screen with the background color
// Grab initial joystick position to bu used as a reference
void CrossHair_Init(void) {
    BSP_LCD_FillScreen(BGCOLOR);
    BSP_Joystick_Input(&origin[0], &origin[1], &select);
}

void IdleThread(void) {
    while (1) OS_Suspend();
}

//******************* Main Function**********
int main(void) {
    uint16_t rawX, rawY;  // raw adc value
    uint32_t seedA, seedB;

    OS_Init();  // initialize, disable interrupts
    Device_Init();
    CrossHair_Init();
    DataLost = 0;   // lost data between producer and consumer
    MaxJitter = 0;  // in 1us units
    Score = 0;
    Life = DEFAULT_LIFE;

    // Grab readings from joystick
    BSP_Joystick_Input(&rawX, &rawY, &select);
    // Concatenate joystick readings
    seedA = (rawX << 16 | rawY);
    seedB = (rawY << 16 | rawX);
    init_lfsrs(seedA, seedB);
    //********initialize communication channels
    JsFifo_Init();

    //*******attach background tasks***********
    OS_AddSW1Task(&SW1Push, 4);
    OS_AddSW2Task(&SW2Push, 4);
    OS_AddPeriodicThread(&Producer, PERIOD, 3);  // 2 kHz real time sampling of PD3

    OS_InitSemaphore(&MoveCubesSem, 0);
    OS_InitSemaphore(&DoneMovingCubesSem, 0);
    OS_InitSemaphore(&ThrottleSem, 0);
    OS_InitSemaphore(&CubeDrawing, 0);
    OS_InitSemaphore(&NeedCubeRedraw, 0);
    OS_InitSemaphore(&InfoSem, 1);
    OS_InitSemaphore(&ResSem, 1);
    OS_InitSemaphore(&DoneSem, 0);
    OS_InitSemaphore(&reset_crosshair_sem, 1);

    NumCreated = 0;
    // create initial foreground threads
    NumCreated += OS_AddThread(&Consumer, 128, 1);
    NumCreated += OS_AddThread(&InitAndSyncBlocks, 128, 1);
    NumCreated += OS_AddThread(&DrawBlocks, 128, 3);
    NumCreated += OS_AddThread(&IdleThread, 128, 6);

    OS_Launch(TIME_2MS);  // doesn't return, interrupts enabled in here
    return 0;             // this never executes
}
