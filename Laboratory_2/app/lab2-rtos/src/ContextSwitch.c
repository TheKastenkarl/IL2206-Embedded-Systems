// File: TwoTasks.c 

#include <stdio.h>
#include "includes.h"
#include <string.h>
#include <altera_avalon_performance_counter.h>

#define DEBUG 1

/* Definition of Task Stacks */
/* Stack grows from HIGH to LOW memory */
#define   TASK_STACKSIZE       2048
OS_STK    task1_stk[TASK_STACKSIZE];
OS_STK    task2_stk[TASK_STACKSIZE];
OS_STK    stat_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define TASK1_PRIORITY      6  // highest priority
#define TASK2_PRIORITY      7
#define TASK_STAT_PRIORITY 12  // lowest priority 

OS_EVENT *ClientRequest; //my change
OS_EVENT *ServerAnswer; // my change

void printStackSize(char* name, INT8U prio) 
{
  INT8U err;
  OS_STK_DATA stk_data;
    
  err = OSTaskStkChk(prio, &stk_data);
  if (err == OS_NO_ERR) {
    if (DEBUG == 1)
      printf("%s (priority %d) - Used: %d; Free: %d\n", 
       name, prio, stk_data.OSUsed, stk_data.OSFree);
  }
  else
    {
      if (DEBUG == 1)
  printf("Stack Check Error!\n");    
    }
}

/* Prints a message and sleeps for given time interval */
void task1(void* pdata)
{
  while (1)
    { 
      INT8U err;
      char state0[] = "Task 0 - State 0\n";
      char state1[] = "Task 0 - State 1\n";
      int i;

      

      for (i = 0; i < strlen(state0); i++){
        putchar(state0[i]);
      }

      OSSemPost(ClientRequest); //my change

      
      PERF_RESET(PERFORMANCE_COUNTER_BASE);
      PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);

      OSSemPend(ServerAnswer, 0, &err);



      for (i = 0; i < strlen(state1); i++){
        putchar(state1[i]);
      }


      OSTimeDlyHMSM(0, 0, 0, 11); /* Context Switch to next task
           * Task will go to the ready state
           * after the specified delay
           */
    }
}

/* Prints a message and sleeps for given time interval */
void task2(void* pdata)
{
  while (1)
    { 
      INT8U err;
      char state0[] = "Task 1 - State 0\n";
      char state1[] = "Task 1 - State 1\n";
      int i;

      OSSemPend(ClientRequest, 0, &err);

      PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);

      alt_u64 context_switch_time = perf_get_total_time(PERFORMANCE_COUNTER_BASE);
      printf("context switch time: %u \n", context_switch_time);



      for (i = 0; i < strlen(state0); i++){
        putchar(state0[i]);
      }

      

      for (i = 0; i < strlen(state1); i++){
        putchar(state1[i]);
      }

      OSSemPost(ServerAnswer);

      OSTimeDlyHMSM(0, 0, 0, 4);
    }
}

/* Printing Statistics */
void statisticTask(void* pdata)
{
  while(1)
    {
      printStackSize("Task1", TASK1_PRIORITY);
      printStackSize("Task2", TASK2_PRIORITY);
      printStackSize("StatisticTask", TASK_STAT_PRIORITY);
    }
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{
  printf("Lab 3 - Two Tasks\n");

  ServerAnswer = OSSemCreate(0); //my change
  ClientRequest = OSSemCreate(0); //my change

  OSTaskCreateExt
    ( task1,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task1_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK1_PRIORITY,               // Desired Task priority
      TASK1_PRIORITY,               // Task ID
      &task1_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
      );
     
  OSTaskCreateExt
    ( task2,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task2_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK2_PRIORITY,               // Desired Task priority
      TASK2_PRIORITY,               // Task ID
      &task2_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                       
      );  

  if (DEBUG == 1)
    {
      OSTaskCreateExt
  ( statisticTask,                // Pointer to task code
    NULL,                         // Pointer to argument passed to task
    &stat_stk[TASK_STACKSIZE-1],  // Pointer to top of task stack
    TASK_STAT_PRIORITY,           // Desired Task priority
    TASK_STAT_PRIORITY,           // Task ID
    &stat_stk[0],                 // Pointer to bottom of task stack
    TASK_STACKSIZE,               // Stacksize
    NULL,                         // Pointer to user supplied memory (not needed)
    OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
    OS_TASK_OPT_STK_CLR           // Stack Cleared                              
    );
    }  

  OSStart();
  return 0;
}