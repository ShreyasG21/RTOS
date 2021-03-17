// RTOS Framework - Spring 2021
// J Losh

// Student Name: Shreyas Vikas Gawali
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Add 03_ prefix to all files in your project
// 03_rtos.c
// 03_tm4c123gh6pm_startup_ccs.c
// 03_other files (except uart0.x and wait.x)
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Orange: PE2
// Yellow: PE3
// Green:  PE4
// PBs on these pins
// PB0:    PA7
// PB1:    PA6
// PB2:    PA5
// PB3:    PA4
// PB4:    PA3
// PB5:    PA2
// UART Interface:
// U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
// The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
// Configured to 115,200 baud, 8N1

///-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//------------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs

#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board orange LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board green LED

#define PB_0   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) // off-board push-button PB0
#define PB_1   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) // off-board push-button PB1
#define PB_2   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // off-board push-button PB2
#define PB_3   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board push-button PB3
#define PB_4   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board push-button PB4
#define PB_5   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board push-button PB5

#define BLUE_LED_MASK   4
#define RED_LED_MASK    2
#define ORANGE_LED_MASK 4
#define YELLOW_LED_MASK 8
#define GREEN_LED_MASK  16

#define PB_0_MASK 128
#define PB_1_MASK 64
#define PB_2_MASK 32
#define PB_3_MASK 16
#define PB_4_MASK 8
#define PB_5_MASK 4

#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)(void);

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5

typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[30];
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];
semaphore *sem;
#define keyPressed  0
#define keyReleased 1
#define flashReq    2
#define resource    3

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define STATE_KILLED     5

#define MAX_TASKS 12            // maximum number of valid tasks
#define MAX_CHARS 30

// global variables

char strg[MAX_CHARS + 1], out[30][30];
uint8_t pos[MAX_CHARS], argCount, argString, argNum, count;
uint8_t taskCurrent = 0;        // index of last dispatched task
uint8_t taskCount = 0;          // total number of valid tasks
uint32_t heap[MAX_TASKS][256];
uint32_t heapShell[1024];
uint8_t svcNum;
uint8_t preemption;
uint8_t prio = 1;
uint32_t tau;
uint32_t pn;
uint32_t startTime, stopTime;

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 7=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    uint32_t deltaTime;
    uint32_t percentCPUint;
    uint32_t percentCPUfra;
    uint32_t time;
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

// Function prototypes
// ASM function prototypes

void setPsp(uint32_t *p);
uint32_t *getPsp(void);
void pushRegs(void);
void popRegs(void);
uint32_t getR0(void);
uint32_t getR1(void);
uint32_t getR2(void);
uint32_t getR3(void);
uint32_t getSVC(void);

// C function prototypes

char *strcopy(char *dest, const char *src);
uint8_t strlength(const char *s);
int strcompare(const char* s1, const char* s2);
int atoi(char* str);
char* itoa(int n, char s[]);
char* reverse(char s[]);
char* decToHexa(long n, char res[100]);

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos(void)
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }

    NVIC_ST_CTRL_R     = 0;                                                                    // Clear Control bit for safe programming
    NVIC_ST_CURRENT_R  = 0;                                                                    // Start Value
    NVIC_ST_RELOAD_R   = 0x9C3F;                                                               // Set for 1Khz, (40000000/1000) - 1
    NVIC_ST_CTRL_R     = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;      // set for source as clock interrupt enable and enable the timer.

}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler(void)
{
    uint8_t v = 0;
    _Bool ok;
    static uint8_t task = 0xFF;
    static uint8_t c;
    ok = false;
    uint8_t k,i;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        if(prio == 1)
        {
            for(k=0;k<=7;k++)
            {
                for(i=0;i<=MAX_TASKS;i++)
                {
                    v = (i+c) % MAX_TASKS;
                    ok = (tcb[v].state == STATE_READY || tcb[v].state == STATE_UNRUN);
                    if(tcb[v].currentPriority == k)
                    {
                        if(ok==true)
                        {
                            task = v;
                            c = v+1;
                            return task;
                        }
                    }
                }
                k++;
            }
        }
        else
        {
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
    }
    return task;
}

_Bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    _Bool ok = false;
    uint8_t i = 0;
    _Bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            if(name == "Shell")
                tcb[i].sp = &heapShell[0];
            else
                tcb[i].sp = &heap[i][(stackBytes/4)-1];
            tcb[i].spInit = &heap[i][(stackBytes/4)-1];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            strcopy(tcb[i].name, name);
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    uint8_t i;
    for(i=0;i<MAX_TASKS;i++)
    {
        if(tcb[i].pid == fn)
        {
            if(tcb[i].state == STATE_INVALID)
            {
                tcb[i].sp = tcb[i].spInit;
                tcb[i].state = STATE_UNRUN;
            }
        }
    }
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
    uint8_t i,j,k;
    for(i=0;i<MAX_TASKS;i++)
    {
        if(tcb[i].pid == fn)
        {
            tcb[i].state = STATE_INVALID;
            taskCount--;
            if(tcb[i].semaphore != 0)
            {
                sem = tcb[i].semaphore;
                for(j=0;j<sem->queueSize;j++)
                {
                    if(sem->processQueue[j] == (uint32_t)fn)
                    {
                        sem->processQueue[j] = 0;
                        for(k=j;k<sem->queueSize;k++)
                        {
                            sem->processQueue[k] = sem->processQueue[k+1];
                        }
                        sem->queueSize--;
                    }
                }
            }
        }
    }
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t i;
    for(i=0;i<MAX_TASKS;i++)
    {
        if(tcb[i].pid == fn)
        {
            tcb[i].currentPriority = priority;
            break;
        }
    }
}

_Bool createSemaphore(uint8_t semaphore, uint8_t count)
{
    _Bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
        if(semaphore == 0)
        {
            strcopy(semaphores[semaphore].name,"keyPressed");
        }
        else if(semaphore == 1)
        {
            strcopy(semaphores[semaphore].name,"keyReleased");
        }
        else if(semaphore == 2)
        {
            strcopy(semaphores[semaphore].name,"flashReq");
        }
        else if(semaphore == 3)
        {
            strcopy(semaphores[semaphore].name,"resource");
        }
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos(void)
{
    _fn fn;
    taskCurrent = rtosScheduler();
    setPsp(tcb[taskCurrent].sp);
    fn = (_fn)tcb[taskCurrent].pid;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    (*fn)();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield(void)
{
    __asm("     SVC  #100");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm("     SVC  #101");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(int8_t semaphore)
{
    __asm("     SVC  #102");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    __asm("     SVC  #103");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr(void)
{
    /*uint8_t j;
    for(j=0;j<MAX_TASKS;j++)
    {
        tau += tcb[j].time;
    }
    pn += ((tcb[taskCurrent].time) * 10000)/tau;
    tcb[taskCurrent].percentCPUint = (pn/1000)%100;
    tcb[taskCurrent].percentCPUfra = pn%100;*/

    if(preemption == 1 && taskCurrent != 0)
    {
        NVIC_INT_CTRL_R = 0x10000000;
    }
    else
    {
        uint8_t i;
        for(i=0;i<MAX_TASKS;i++)
        {
            if(tcb[i].state == STATE_DELAYED)
            {
                tcb[i].ticks--;
                if(tcb[i].ticks == 0)
                {
                    tcb[i].state = STATE_READY;
                }
            }

        }
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr(void)
{
    pushRegs();
    tcb[taskCurrent].sp = getPsp();

    /*stopTime = TIMER1_TAV_R;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_TAV_R = 0;

    tcb[taskCurrent].deltaTime = stopTime - startTime;
    tcb[taskCurrent].time += tcb[taskCurrent].deltaTime;*/

    taskCurrent = rtosScheduler();

    /*TIMER1_CTL_R |= TIMER_CTL_TAEN;
    startTime = TIMER1_TAV_R;*/

    if(tcb[taskCurrent].state == STATE_UNRUN)
    {
        setPsp(tcb[taskCurrent].sp);
        uint32_t *ptr;
        ptr = getPsp();
        ptr--;
        *ptr = 0x01000000;
        ptr--;
        *ptr = (uint32_t)tcb[taskCurrent].pid;
        ptr --;
        ptr --;
        ptr --;
        ptr --;
        ptr --;
        ptr --;
        setPsp(ptr);
        tcb[taskCurrent].state = STATE_READY;
    }
    else if(tcb[taskCurrent].state == STATE_READY)
    {
        setPsp(tcb[taskCurrent].sp);
        popRegs();
    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr(void)
{
    uint32_t r0 = getR0();
    svcNum = getSVC();
    switch(svcNum)
    {
        case 100:       // yield
        {
            NVIC_INT_CTRL_R = 0x10000000;
            break;
        }

        case 101:       // sleep
        {
            tcb[taskCurrent].ticks = r0;
            tcb[taskCurrent].state = STATE_DELAYED;
            NVIC_INT_CTRL_R = 0x10000000;
            break;
        }

        case 102:       // wait
        {
            if(semaphores[r0].count > 0)
            {
                semaphores[r0].count--;
            }
            else
            {
                tcb[taskCurrent].state = STATE_BLOCKED;
                semaphores[r0].processQueue[semaphores[r0].queueSize] = (uint32_t)tcb[taskCurrent].pid;
                semaphores[r0].queueSize++;
                NVIC_INT_CTRL_R = 0x10000000;
            }
            break;
        }

        case 103:       // post
        {
            uint8_t i,j;
            semaphores[r0].count++;
            if(semaphores[r0].queueSize > 0)
            {
                semaphores[r0].count--;
                for(i=0;i<MAX_TASKS;i++)
                {
                    if(semaphores[r0].processQueue[0] == (uint32_t)tcb[i].pid)
                    {
                        semaphores[r0].processQueue[0] = 0;
                        tcb[i].state = STATE_READY;
                        for(j=0;j<semaphores[r0].queueSize;j++)
                        {
                            semaphores[r0].processQueue[j] = semaphores[r0].processQueue[j+1];
                            semaphores[r0].queueSize--;
                        }
                        break;
                    }
                }
            }
            NVIC_INT_CTRL_R = 0x10000000;
            break;
        }
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw(void)
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO ports
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD;

    // Configure LED pins

    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;  // enable on-board LED
    GPIO_PORTE_DIR_R |= RED_LED_MASK|ORANGE_LED_MASK|YELLOW_LED_MASK|GREEN_LED_MASK;  // make Port E an output port
    GPIO_PORTE_DR2R_R |= RED_LED_MASK|ORANGE_LED_MASK|YELLOW_LED_MASK|GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R |= RED_LED_MASK|ORANGE_LED_MASK|YELLOW_LED_MASK|GREEN_LED_MASK;  // enable digital for off-board LEDs

    // Configure push-buttons
    GPIO_PORTA_DIR_R &= ~PB_0_MASK|~PB_1_MASK|~PB_2_MASK|~PB_3_MASK|~PB_4_MASK|~PB_5_MASK;  // make PA7-PA2 inputs
    GPIO_PORTA_DEN_R |= PB_0_MASK|PB_1_MASK|PB_2_MASK|PB_3_MASK|PB_4_MASK|PB_5_MASK;        // enable digital on Port A
    GPIO_PORTA_PUR_R |= PB_0_MASK|PB_1_MASK|PB_2_MASK|PB_3_MASK|PB_4_MASK|PB_5_MASK;        // enable internal pull-ups for push-buttons

    // Configure Timer 1
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;                                      // Enable Systick timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                                                // Disable timer before configuring (safe programming)
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                                          // Configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;                     // Configure for periodic mode and Count Up timer
    TIMER1_TAV_R = 0;                                                               // set initial value to zero
    TIMER1_TAILR_R = 0xFFFFFFFF;                                                    // set load value to max value

}

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs(void)
{
    uint8_t value = 0;
    if(!PB_0)
    {
        value = 1;
    }
    else if(!PB_1)
    {
        value = 2;
    }
    else if(!PB_2)
    {
        value = 4;
    }
    else if(!PB_3)
    {
        value = 8;
    }
    else if(!PB_4)
    {
        value = 16;
    }
    else if(!PB_5)
    {
        value = 32;
    }
    return value;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// https://www.techiedelight.com/implement-strcpy-function-c/
// implementation of string copy function

char *strcopy(char *dest, const char *src)
{
    if(dest == NULL)
    {
        return NULL;
    }
    char *p = dest;
    while(*src!='\0')
    {
        *dest = *src;
        dest++;
        src++;
    }
    *dest = '\0';
    return p;
}

// https://aticleworld.com/strlen-function-in-c-cpp/
// implementation of string length function

uint8_t strlength(const char *s)
{
    uint8_t count = 0;
    while(*s!='\0')
    {
        count++;
        s++;
    }
    return count;
}

// https://stackoverflow.com/questions/34873209/implementation-of-strcmp
// implementation of string compare function

int strcompare(const char* s1, const char* s2)
{
    while(*s1 && (*s1 == *s2))
    {
        s1++;
        s2++;
    }
    return *(const unsigned char*)s1 - *(const unsigned char*)s2;
}

// https://www.geeksforgeeks.org/write-your-own-atoi/
// implementation of atoi function

int atoi(char* str)
{
    // Initialize result
    int res = 0;
    int i;
    // Iterate through all characters
    // of input string and update result
    // take ASCII character of corrosponding digit and
    // subtract the code from '0' to get numerical
    // value and multiply res by 10 to shuffle
    // digits left to update running total
    for (i = 0; str[i] != '\0'; ++i)
        res = res * 10 + str[i] - '0';

    // return result.
    return res;
}

// https://en.wikibooks.org/wiki/C_Programming/stdlib.h/itoa
// implementation of itoa function

char* itoa(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)
        n = -n;
    i = 0;
    do
    {
        s[i++] = n % 10 + '0';
    }
    while ((n /= 10) > 0);
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    return reverse(s);
}

// https://en.wikibooks.org/wiki/C_Programming/stdlib.h/itoa
// implementation of string reversal function

char* reverse(char s[])
{
    int i, j,l;
    char c;
    for (l = 0; s[l] != '\0'; ++l);
    ;
    for (i = 0, j = l - 1; i < j; i++, j--)
    {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
    return s;
}
// https://www.geeksforgeeks.org/program-decimal-hexadecimal-conversion/
// function to convert decimal to hexadecimal

char* decToHexa(long n, char res[100])
{
    int j,t;
    //char res[];
    // char array to store hexadecimal number
    char hexaDeciNum[100];

    // counter for hexadecimal number array
    int i = 0;
    while (n != 0) {
        // temporary variable to store remainder
        int temp = 0;

        // storing remainder in temp variable.
        temp = n % 16;

        // check if temp < 10
        if (temp < 10) {
            hexaDeciNum[i] = temp + 48;
            i++;
        }
        else {
            hexaDeciNum[i] = temp + 55;
            i++;
        }

        n = n / 16;
    }

    // printing hexadecimal number array in reverse order
    for (j = i - 1; j >= 0; j--)
    {
        for(t=0;t<j;t++)
        {
            res[t] = hexaDeciNum[j];
        }
        //res = hexaDeciNum[j];
    }
    return reverse(res);
}

// string parsing functions from EE5314 code

void getString(void)
{
    uint8_t count=0;
    char c;
    while(1)
    {
        label: c = getcUart0();
        putcUart0(c);
        if((c==8)||(c==127))
        {
            if(count>0)
            {
                count--;
                goto label;
            }
            else
            {
                goto label;
            }
        }
        else
        {
            if((c==10)||(c==13))
            {
                label1: strg[count] = 0;
                break;
            }
            else
            {
                if(c>=32)
                {
                    strg[count++] = c;
                    if(count == MAX_CHARS)
                    {
                        goto label1;
                    }
                    else
                    {
                        goto label;
                    }
                }
                else
                {
                    goto label;
                }
            }
        }
    }
}

_Bool alfNum(char a)
{
    if((a>=45 && a<=57)||(a>=65 && a<=90)||(a>=97 && a<=122))
    {
        return true;
    }
    else
    {
        return false;
    }
}

_Bool splCh(char b)
{
    if((b>=32 && b<=44)||(b==47)||(b>=58 && b<=64)||(b>=91 && b<=96)||(b>=123 && b<=126))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void parseString(void)
{
    uint8_t j=0,l,i=0,r=0,c=0,flag=0;
    l = strlength(strg);
    while(i<=l)
    {
        while(alfNum(strg[i]))
        {
            flag =1;
            if((i==0)||(strg[i-1]=='\0'))
            {
                pos[j]=i;
                j++;
            }
            out[j-1][c] = strg[i];
            c++;
            i++;
        }
        c=0;
        if(flag==1)
        {
            r++;
        }
        if(splCh(strg[i+1]) && flag==1)
        {
            r--;
        }
        strg[i]='\0';

        i++;
    }
argCount=r;
}

char *getargString(uint8_t argNum)
{
    if(argNum == argCount-1)
    {
        return &strg[pos[argNum]];
    }
    else
    {
        putsUart0("Error: Invalid number of arguments");
    }
    return 0;
}

uint32_t getargInt(uint8_t argNum)
{
    return atoi((getargString(argNum)));
}

_Bool isCommand(char *command, uint8_t argNum)
{
    uint16_t compareStr;
    compareStr = strcompare(&strg[pos[0]], command);
    if(argNum == argCount-1 && compareStr == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// power up flash
void powerUp(void)
{
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);
}

// reset
void reset(void)
{
    putsUart0("System reset initiated");
    putsUart0("\r\n");
    waitMicrosecond(250000);
    NVIC_APINT_R = 0x04 | (0x05FA << 16);
}
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose

void idle(void)
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

// REQUIRED: Add other tasks here after task switching is complete
//           These will be provided in class

void flash4Hz(void)
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot(void)
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn(void)
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn(void)
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys(void)
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce(void)
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative(void)
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant(void)
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important(void)
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell(void)
{
    while (true)
    {
        putsUart0("\r\n");
        putsUart0("Enter command : ");
        getString();
        parseString();
        putsUart0("\r\n");
        if(isCommand("help",0))
        {
            putsUart0("\r\n");
            putsUart0("LIST OF COMMANDS\r\n");
            putsUart0("\r\n");
            putsUart0("ps:                  Process Stats\r\n");
            putsUart0("ipcs:                Inter-Process Communication Status\r\n");
            putsUart0("reboot:              Restart the CPU \r\n");
            putsUart0("pidof thread_name:   Process ID of a thread\r\n");
            putsUart0("kill pid:            Kill a thread\r\n");
            putsUart0("fn_name &:           Restart a thread\r\n");
            putsUart0("sched PRIO/RR:       PRIO -> Priority Scheduler | RR -> Round-robin Scheduler  \r\n");
            putsUart0("preemption ON/OFF:   ON -> Preemeption ON | OFF -> Preemption OFF \r\n");
        }
        else if(isCommand("reboot",0))
        {
            reset();
        }
        else if(isCommand("pidof",1))
        {
            uint8_t i, c;
            char a[30];
            for(i=0;i<MAX_TASKS;i++)
            {
                c = strcompare(&strg[pos[1]],tcb[i].name);
                if(c==0)
                {
                    putsUart0("PID: ");
                    itoa((uint32_t)tcb[i].pid,a);
                    putsUart0(a);
                    putsUart0("\r\n");
                }
            }
        }
        else if(isCommand("kill",1))
        {
            uint8_t i;
            uint32_t pid;
            pid = atoi(&strg[pos[1]]);
            for(i=0;i<MAX_TASKS;i++)
            {
                if((uint32_t)tcb[i].pid == pid)
                {
                    if(strcompare(tcb[i].name,"Shell")==0||strcompare(tcb[i].name,"Idle")==0)
                    {
                        putsUart0("Error: Cannot kill Shell or Idle \r\n");
                    }
                    else
                    {
                        putsUart0("(x_x) \r\n");
                        destroyThread((_fn)tcb[i].pid);
                    }

                }
            }
        }
        else if(isCommand("Flash4Hz",1)||isCommand("LengthyFn",1)||isCommand("OneShot",1)||isCommand("ReadKeys",1)||
                isCommand("Debounce",1)||isCommand("Important",1)||isCommand("Uncoop",1)||isCommand("Errant",1))
        {
            uint8_t i;
            for(i=0;i<MAX_TASKS;i++)
            {
                if(strcompare(tcb[i].name,&strg[pos[0]]) == 0)
                {
                    putsUart0("(^_^) \r\n");
                    restartThread((_fn)tcb[i].pid);
                }
            }
        }
        else if(isCommand("preemption",1))
        {
            uint8_t c,v;
            c = strcompare(&strg[pos[1]],"ON");
            v = strcompare(&strg[pos[1]],"OFF");
            if(c == 0)
            {
                preemption = 1;
                putsUart0("Preemption enabled \r\n");
            }
            else if(v == 0)
            {
                preemption = 0;
                putsUart0("Preemption OFF \r\n");
            }
        }
        else if(isCommand("sched",1))
        {
            uint8_t c,v;
            c = strcompare(&strg[pos[1]],"PRIO");
            v = strcompare(&strg[pos[1]],"RR");
            if(c == 0 && prio == 1)
            {
                putsUart0("Priority scheduler already ON \r\n");
            }
            else if(c == 0)
            {
                prio = 1;
                putsUart0("Priority scheduler \r\n");
            }
            else if(v == 0 && prio == 1)
            {
                prio = 0;
                putsUart0("Round robin scheduler \r\n");
            }
        }
        else if(isCommand("ipcs",0))
        {
            putsUart0("\r\n");
            uint8_t i;
            char a[30],b[30];
            putsUart0("SEMAPHORE NAME\t");
            putsUart0("COUNT\t");
            putsUart0("QUEUE SIZE\t");
            putsUart0("\r\n");
            for(i=0;i<MAX_SEMAPHORES-1;i++)
            {
                putsUart0("\r\n");
                putsUart0(semaphores[i].name);
                putsUart0("\t  ");
                itoa(semaphores[i].count,a);
                putsUart0(a);
                putsUart0("\t  ");
                itoa(semaphores[i].queueSize,b);
                putsUart0(b);
            }
            putsUart0("\r\n");
        }
        else if(isCommand("ps",0))
        {
            putsUart0("\r\n");
            uint8_t i;
            char a[30],d[100];
            //char b[30],c[30];
            putsUart0("PID \t");
            putsUart0("STATE \t\t");
            //putsUart0("%CPU USE \t");
            putsUart0("&SP \t\t");
            putsUart0("THREAD NAME");
            putsUart0("\r\n");
            for(i=0;i<MAX_TASKS-2;i++)
            {
                putsUart0("\r\n");
                itoa((uint32_t)tcb[i].pid,a);
                putsUart0(a);
                putsUart0("\t");
                if(tcb[i].state == 0)
                {
                    putsUart0("INVALID");
                }
                else if(tcb[i].state == 1)
                {
                    putsUart0("UNRUN");
                }
                else if(tcb[i].state == 2)
                {
                    putsUart0("READY");
                }
                else if(tcb[i].state == 3)
                {
                    putsUart0("DELAYED");
                }
                else if(tcb[i].state == 4)
                {
                    putsUart0("BLOCKED");
                }
                putsUart0("\t\t");
                /*itoa(tcb[i].percentCPUint,b);
                itoa(tcb[i].percentCPUfra,c);
                putsUart0(b);
                putsUart0(".");
                putsUart0(c);
                putsUart0("%");
                putsUart0("\t\t");*/
                decToHexa((long)tcb[i].sp,d);
                putsUart0("0x");
                putsUart0(d);
                putsUart0("\t");
                putsUart0(tcb[i].name);
            }
            putsUart0("\r\n");
        }
        else
        {
            putsUart0("Invalid command enter 'help' for list of commands \r\n");
        }
        yield();
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    _Bool ok;

    // Initialize hardware
    initHw();
    initUart0();
    initRtos();

    // Setup UART0 baud rate
    //setUart0BaudRate(115200, 40e6);

    // Power-up flash
    powerUp();

    // Initialize semaphores
    createSemaphore(keyPressed, 1);
    createSemaphore(keyReleased, 0);
    createSemaphore(flashReq, 5);
    createSemaphore(resource, 1);

    // Add required idle process at lowest priority
    ok = createThread(idle, "Idle", 7, 1024);

    // Add other processes
    // REQUIRED: create threads for additional tasks here

    ok &= createThread(lengthyFn,     "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz,      "Flash4Hz",  4, 1024);
    ok &= createThread(oneshot,       "OneShot",   2, 1024);
    ok &= createThread(readKeys,      "ReadKeys",  6, 1024);
    ok &= createThread(debounce,      "Debounce",  6, 1024);
    ok &= createThread(important,     "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop",    6, 1024);
    ok &= createThread(errant,        "Errant",    6, 1024);
    ok &= createThread(shell,         "Shell",     6, 4096);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
