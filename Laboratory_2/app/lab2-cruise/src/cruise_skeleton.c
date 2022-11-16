/* Cruise control skeleton for the IL 2206 embedded lab
 *
 * Maintainers:  Rodolfo Jordao (jordao@kth.se), George Ungereanu (ugeorge@kth.se)
 *
 * Description:
 *
 *   In this file you will find the "model" for the vehicle that is being simulated on top
 *   of the RTOS and also the stub for the control task that should ideally control its
 *   velocity whenever a cruise mode is activated.
 *
 *   The missing functions and implementations in this file are left as such for
 *   the students of the IL2206 course. The goal is that they get familiriazed with
 *   the real time concepts necessary for all implemented herein and also with Sw/Hw
 *   interactions that includes HAL calls and IO interactions.
 *
 *   If the prints prove themselves too heavy for the final code, they can
 *   be exchanged for alt_printf where hexadecimals are supported and also
 *   quite readable. This modification is easily motivated and accepted by the course
 *   staff.
 */
#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
//#include "altera_avalon_performance_counter.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */
#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02

/* Switch Patterns */
#define TOP_GEAR_FLAG       0x00000002 // SW1
#define ENGINE_FLAG         0x00000001 // SW0
#define SW2_FLAG            0x00000004
#define SW3_FLAG            0x00000008
#define SW4_FLAG            0x00000010
#define SW5_FLAG            0x00000020
#define SW6_FLAG            0x00000040
#define SW7_FLAG            0x00000080
#define SW8_FLAG            0x00000100
#define SW9_FLAG            0x00000200

/* LED Patterns */
#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear
#define LED_RED_2 0x00000004
#define LED_RED_3 0x00000008
#define LED_RED_4 0x00000010
#define LED_RED_5 0x00000020
#define LED_RED_6 0x00000040
#define LED_RED_7 0x00000080
#define LED_RED_8 0x00000100
#define LED_RED_9 0x00000200
#define LED_RED_10 0x00000400
#define LED_RED_11 0x00000800
#define LED_RED_12 0x00001000
#define LED_RED_13 0x00002000
#define LED_RED_14 0x00004000
#define LED_RED_15 0x00008000
#define LED_RED_16 0x00010000
#define LED_RED_17 0x00020000

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIO_Stack[TASK_STACKSIZE];
OS_STK SwitchIO_Stack[TASK_STACKSIZE];
OS_STK Watchdog_Stack[TASK_STACKSIZE];
OS_STK Overload_Stack[TASK_STACKSIZE];
OS_STK ExtraLoad_Stack[TASK_STACKSIZE];

// Task Priorities

#define STARTTASK_PRIO     5
#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  12
#define BUTTONIO_PRIO      7
#define SWITCHIO_PRIO      8
#define WATCHDOG_PRIO      1
#define OVERLOAD_PRIO      3
#define EXTRALOAD_PRIO     2

// Task Periods
#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define BUTTONS_PERIOD  300
#define SWITCHES_PERIOD 300
#define WATCHDOG_PERIOD 300
#define OVERLOAD_PERIOD 300
#define EXTRALOAD_PERIOD 300

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;
OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_Engine;

// Semaphores
OS_EVENT *Vehicle_Semaphore;
OS_EVENT *Control_Semaphore;
OS_EVENT *Buttons_Semaphore;
OS_EVENT *Switches_Semaphore;
OS_EVENT *Watchdog_Semaphore;
OS_EVENT *Overload_Semaphore;
OS_EVENT *OKSignal_Semaphore;
OS_EVENT *ExtraLoad_Semaphore;

// SW-Timer
OS_TMR *Vehicle_Timer;
OS_TMR *Control_Timer;
OS_TMR *Buttons_Timer;
OS_TMR *Switches_Timer;
OS_TMR *Watchdog_Timer;
OS_TMR *Overload_Timer;
OS_TMR *ExtraLoad_Timer;

/*
 * Types
 */
enum active {on = 2, off = 1};
enum active cruise_control = off; 
enum active gas_pedal = off;
enum active brake_pedal = off;
enum active top_gear = off;
enum active engine = off;


/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs
int desired_utilization = 0; // desired utilization of extra load


/*
 * Helper functions
 */
int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */

  return delay;
}

static int b2sLUT[] = {0x40, //0
  0x79, //1
  0x24, //2
  0x30, //3
  0x19, //4
  0x12, //5
  0x02, //6
  0x78, //7
  0x00, //8
  0x18, //9
  0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{ 
  //printf("target vel in here: %d \n", target_vel);
  int tmp = target_vel;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = out_high << 7 | out_low;
  if (cruise_control == on) {
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE, out);
  } else { // show velocity of zero if cruise control is off
    out = int2seven(0) << 7 | int2seven(0);
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE, out);
  }
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
    led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
    if (0 <= position && position < 400) {
        INT32U led_mask = led_red | LED_RED_17;  // turn on LEDR17
        led_mask = led_mask & ~LED_RED_16;       // turn off LEDR16
        led_mask = led_mask & ~LED_RED_15;       // turn off LEDR15
        led_mask = led_mask & ~LED_RED_14;       // turn off LEDR14
        led_mask = led_mask & ~LED_RED_13;       // turn off LEDR13
        led_mask = led_mask & ~LED_RED_12;       // turn off LEDR12
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_mask);
    } else if (position < 800) {
        INT32U led_mask = led_red & ~LED_RED_17; // turn off LEDR17
        led_mask = led_mask | LED_RED_16;        // turn on LEDR16
        led_mask = led_mask & ~LED_RED_15;       // turn off LEDR15
        led_mask = led_mask & ~LED_RED_14;       // turn off LEDR14
        led_mask = led_mask & ~LED_RED_13;       // turn off LEDR13
        led_mask = led_mask & ~LED_RED_12;       // turn off LEDR12
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_mask);
    } else if (position < 1200) {
        INT32U led_mask = led_red & ~LED_RED_17; // turn off LEDR17
        led_mask = led_mask & ~LED_RED_16;       // turn off LEDR16
        led_mask = led_mask |  LED_RED_15;       // turn on LEDR15
        led_mask = led_mask & ~LED_RED_14;       // turn off LEDR14
        led_mask = led_mask & ~LED_RED_13;       // turn off LEDR13
        led_mask = led_mask & ~LED_RED_12;       // turn off LEDR12
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_mask);
    } else if (position < 1600) {
        INT32U led_mask = led_red & ~LED_RED_17; // turn off LEDR17
        led_mask = led_mask & ~LED_RED_16;       // turn off LEDR16
        led_mask = led_mask & ~LED_RED_15;       // turn off LEDR15
        led_mask = led_mask | LED_RED_14;        // turn on LEDR14
        led_mask = led_mask & ~LED_RED_13;       // turn off LEDR13
        led_mask = led_mask & ~LED_RED_12;       // turn off LEDR12
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_mask);
    } else if (position < 2000) {
        INT32U led_mask = led_red & ~LED_RED_17; // turn off LEDR17
        led_mask = led_mask & ~LED_RED_16;       // turn off LEDR16
        led_mask = led_mask & ~LED_RED_15;       // turn off LEDR15
        led_mask = led_mask & ~LED_RED_14;       // turn off LEDR14
        led_mask = led_mask | LED_RED_13;        // turn on LEDR13
        led_mask = led_mask & ~LED_RED_12;       // turn off LEDR12
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_mask);
    } else if (position <= 2400) {
        INT32U led_mask = led_red & ~LED_RED_17; // turn off LEDR17
        led_mask = led_mask & ~LED_RED_16;       // turn off LEDR16
        led_mask = led_mask & ~LED_RED_15;       // turn off LEDR15
        led_mask = led_mask & ~LED_RED_14;       // turn off LEDR14
        led_mask = led_mask & ~LED_RED_13;       // turn off LEDR13
        led_mask = led_mask | LED_RED_12;        // turn on LEDR12
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_mask);
    } else {
        INT32U led_mask = led_red & ~LED_RED_17; // turn off LEDR17
        led_mask = led_mask & ~LED_RED_16;       // turn off LEDR16
        led_mask = led_mask & ~LED_RED_15;       // turn off LEDR15
        led_mask = led_mask & ~LED_RED_14;       // turn off LEDR14
        led_mask = led_mask & ~LED_RED_13;       // turn off LEDR13
        led_mask = led_mask & ~LED_RED_12;       // turn off LEDR12
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_mask);
    }
}

/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 * 
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 */
void VehicleTask(void* pdata)
{ 
  INT8U err;  

  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  
  void* msg;
  INT8U* throttle; 
  INT16S acceleration;  
  INT16U position = 0; 
  INT16S velocity = 0; 
  enum active brake_pedal_local = off;
  enum active engine_local = off;

  printf("Vehicle task created!\n");

  while(1)
  {
    err = OSMboxPost(Mbox_Velocity, (void *) &velocity);
    
    //OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD); 
    OSSemPend(Vehicle_Semaphore, 0, &err);

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */
    msg = OSMboxPend(Mbox_Throttle, 1, &err); 
    if (err == OS_NO_ERR) 
      throttle = (INT8U*) msg;
    /* Same for the brake signal that bypass the control law */
    msg = OSMboxPend(Mbox_Brake, 1, &err); 
    if (err == OS_NO_ERR) {
      brake_pedal_local = (enum active) msg;
    }
    /* Same for the engine signal that bypass the control law */
    msg = OSMboxPend(Mbox_Engine, 1, &err);
    if (err == OS_NO_ERR) {
      engine_local = (enum active) msg;
    }

    // vehichle cannot effort more than 80 units of throttle
    if (*throttle > 80) *throttle = 80;

    // brakes + wind
    if (brake_pedal == off)
    {
      // wind resistance
      acceleration = - wind_factor*velocity;
      // actuate with engines
      if (engine == on){
        acceleration += (*throttle);
        //printf("engine on");
      }
      //printf("Engine (2=on, 1=off): %d\n", engine);

      // gravity effects
      if (400 <= position && position < 800)
        acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)
        acceleration -= 2*gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)
        acceleration += 2*gravity_factor; //traveling downhill
      else if (2000 <= position)
        acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else 
      acceleration = - brake_factor*velocity;

    printf("Position: %d m\n", position);
    printf("Velocity: %d m/s\n", velocity);
    printf("Accell: %d m/s2\n", acceleration);
    printf("Throttle: %d V\n", *throttle);

    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
    // reset the position to the beginning of the track
    if(position > 2400)
      position = 0;

    show_velocity_on_sevenseg((INT8S) velocity);
    show_position(position); // new
  }
} 

/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */
void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 0; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S* current_velocity = 0;
  INT16S* target_velocity = 0;

  //enum active gas_pedal = off;
  //enum active top_gear = off;

  printf("Control Task created\n");

  while(1)
  {
    void* msg = OSMboxPend(Mbox_Velocity, 1, &err); 
    if (err == OS_NO_ERR) {
      current_velocity = (INT16S*) msg;
    }

// Here you can use whatever technique or algorithm that you prefer to control
    // the velocity via the throttle. There are no right and wrong answer to this controller, so
    // be free to use anything that is able to maintain the cruise working properly. You are also
    // allowed to store more than one sample of the velocity. For instance, you could define
    //
    // INT16S previous_vel;
    // INT16S pre_previous_vel;
    // ...
    //
    // If your control algorithm/technique needs them in order to function. 
    
    // switch off LEDG0 when cruise control is inactive

    printf("Target velocity: %d \n", *target_velocity);
    show_target_velocity(*target_velocity);

    if (cruise_control == off){
        led_green = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green & ~LED_GREEN_0);

        void* msg = OSMboxPend(Mbox_Velocity, 1, &err); 
        if (err == OS_NO_ERR) {
          current_velocity = (INT16S*) msg;
        }
        if (gas_pedal == on){
          *target_velocity = *current_velocity;
        }

    } else if (cruise_control == on && *target_velocity >= 20){
        // switch off LEDG0 when cruise control is inactive
        led_green = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green | LED_GREEN_0);

        // controller for cruise control
        if ((*current_velocity - *target_velocity) < 0) {
            brake_pedal = off;
            throttle = 40;
            err = OSMboxPost(Mbox_Throttle, (void*) &throttle);
        } else if ((*current_velocity - *target_velocity) > 0){
            brake_pedal = on;
            err = OSMboxPost(Mbox_Brake, (void*) &brake_pedal);
        }
    }

    if(gas_pedal == on) {
      throttle = 40;
      err = OSMboxPost(Mbox_Throttle, (void*) &throttle);
    } else {
      throttle = throttle/1.0001; // slowly decrease throttle so that vehicle does not stop immediately
      err = OSMboxPost(Mbox_Throttle, (void*) &throttle);
    }

    // //OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD);
    OSSemPend(Control_Semaphore, 0, &err);
  }
}

void ButtonIO(void* pdata){
  INT8U err;
  int buttons;
  
  INT16S* current_velocity = 0;

  while(1){
    OSSemPend(Buttons_Semaphore, 0, &err);
    buttons = buttons_pressed();
    led_green = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE);

    void* msg = OSMboxPend(Mbox_Velocity, 1, &err); 
      if (err == OS_NO_ERR) {
        current_velocity = (INT16S*) msg;
      }

    if (buttons & CRUISE_CONTROL_FLAG ) {  /* push button1 */
        if(cruise_control == on) {

          IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green & ~LED_GREEN_2);
          cruise_control = off;
          //target_velocity = 0;
          
        } else if(top_gear == on) {

            void* msg = OSMboxPend(Mbox_Velocity, 1, &err); 
            if (err == OS_NO_ERR) {
              current_velocity = (INT16S*) msg;
            }
            led_green = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE);
            if (*current_velocity >= 20 && brake_pedal == off && gas_pedal == off) {
                //target = *current_velocity;
                //show_target_velocity((INT8U) (target / 10));
                
                IOWR_ALTERA_AVALON_PIO_DATA (DE2_PIO_GREENLED9_BASE, led_green | LED_GREEN_2); // LEDG2 on
                //target_velocity = current_velocity;
                cruise_control = on;
            }
        }
    }

    led_green = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE);
    if (buttons & GAS_PEDAL_FLAG) {
      if (gas_pedal == on){
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green & ~LED_GREEN_6);
        gas_pedal = off;
        void* msg = OSMboxPend(Mbox_Velocity, 1, &err); 
        if (err == OS_NO_ERR) {
          current_velocity = (INT16S*) msg;
        }
        //err = OSMboxPost(Mbox_Brake, (void *) &brake_pedal);
      } else {
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green | LED_GREEN_6);
        gas_pedal = on;
        
        led_green = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE);

        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green & ~LED_GREEN_2);
        cruise_control = off;
        //target_velocity = 0;
        
        //err = OSMboxPost(Mbox_Brake, (void *) &brake_pedal);
      }
    }

    led_green = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE);
    if (buttons & BRAKE_PEDAL_FLAG) {
      if (brake_pedal == on){

        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green & ~LED_GREEN_4);
        brake_pedal = off;
        void* msg = OSMboxPend(Mbox_Velocity, 1, &err); 
        if (err == OS_NO_ERR) {
          current_velocity = (INT16S*) msg;
        }
        //err = OSMboxPost(Mbox_Brake, (void *) &brake_pedal);
      } else {
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green | LED_GREEN_4);
        brake_pedal = on;

        led_green = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE);

        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green & ~LED_GREEN_2);
        cruise_control = off;
        //target_velocity = 0;
        //err = OSMboxPost(Mbox_Brake, (void *) &brake_pedal);
      }
    }
  }
}

// new task
void SwitchIO(void* pdata)
{
    INT8U err;
    int is_SW0_active;
    int is_SW1_active;
    INT16S* current_velocity = NULL;

    while(1) {
        OSSemPend(Switches_Semaphore, 0, &err);

        desired_utilization = get_desired_utilization_from_switches(); // set global variable

        is_SW0_active = switches_pressed() & ENGINE_FLAG;
        is_SW1_active = switches_pressed() & TOP_GEAR_FLAG;
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        led_green = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE);

        // 1 - ENGINE
        if(is_SW0_active) {
            if(engine == off) {
                engine = on;
                IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red | LED_RED_0); // turn LED on
                err = OSMboxPost(Mbox_Engine, (void*) &engine);
            }
        } else { // SW0 is not active
            if(engine == on) {
                // read out current velocity
                /* Non-blocking read of mailbox: 
                - message in mailbox: update velocity
                - no message:         use old velocity
                */
                void* msg = OSMboxPend(Mbox_Velocity, 1, &err);
                if (err == OS_NO_ERR) {
                    current_velocity = (INT16S*) msg;
                }

                // Only turn off motor if the speed of the car is 0
                if(*current_velocity == 0) { 
                    engine = off;
                    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red & ~LED_RED_0); // turn LED off
                }
            }
        }

        // 2 - TOP GEAR
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        if(is_SW1_active) {
            if(top_gear == off) {
                top_gear = on;
                IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red | LED_RED_1); // turn LED on
            }
        } else { // SW1 is not active
            if(top_gear == on) {
                top_gear = off;
                IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red & ~LED_RED_1); // turn LED off

                // cruise control is only active in top gear
                cruise_control = off;
                IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_green & ~LED_GREEN_2);
            }
        }
    }
}

void Watchdog(void* pdata){
  INT8U err;
  while(1){
    OSSemPend(Watchdog_Semaphore, 0, &err);

    OSSemPend(OKSignal_Semaphore, 299, &err);

    if (err != OS_NO_ERR){
      printf("OVERLOADED!");
    }
  }
}

void Overload(void* pdata){
  INT8U err;
  while(1){
    OSSemPend(Overload_Semaphore, 0, &err);
    OSSemPost(OKSignal_Semaphore);
  }

}

void ExtraLoad(void* pdata){
    int i, x;
    INT8U err;
    
    while(1)
    {
        OSSemPend(ExtraLoad_Semaphore, 0, &err);
        printf("Desired utilization %d \n", desired_utilization);

        //PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
        for(i = 0; i <= desired_utilization*60; ++i) {
            x = x + 1;
        }
        //PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);
        //alt_u64 measured_time = perf_get_total_time(PERFORMANCE_COUNTER_BASE);
        //PERF_RESET(PERFORMANCE_COUNTER_BASE);
        //printf("(%d ticks)\n", measured_time);
    }
}

// Returns the desired utilization (in %) which is determined by the switch position
int get_desired_utilization_from_switches() {
    // Determine which switches are pressed
    int is_SW4_active = switches_pressed() & SW4_FLAG;
    int is_SW5_active = switches_pressed() & SW5_FLAG;
    int is_SW6_active = switches_pressed() & SW6_FLAG;
    int is_SW7_active = switches_pressed() & SW7_FLAG;
    int is_SW8_active = switches_pressed() & SW8_FLAG;
    int is_SW9_active = switches_pressed() & SW9_FLAG;

// Turn on LEDs where the switch is active
    if (is_SW4_active) {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red | LED_RED_4); 
        is_SW4_active = 1;
    } else {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red & ~LED_RED_4); 
        is_SW4_active = 0;
    }
    if (is_SW5_active) {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red | LED_RED_5); 
        is_SW5_active = 1;
    } else {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red & ~LED_RED_5); 
        is_SW5_active = 0;
    }
    if (is_SW6_active) {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red | LED_RED_6);
        is_SW6_active = 1;
    } else {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red & ~LED_RED_6); 
        is_SW6_active = 0;
    }
    if (is_SW7_active) {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red | LED_RED_7); 
        is_SW7_active = 1;
    } else {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red & ~LED_RED_7); 
        is_SW7_active = 0;
    }
    if (is_SW8_active) {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red | LED_RED_8); 
        is_SW8_active = 1;
    } else {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red & ~LED_RED_8); 
        is_SW8_active = 0;
    }
    if (is_SW9_active) {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red | LED_RED_9); 
        is_SW9_active = 1;
    } else {
        led_red = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red & ~LED_RED_9); 
        is_SW9_active = 0;
    }

    // Calculate the binary number used for calculating the utilization
    int binary_number = is_SW4_active + 2 * is_SW5_active + 4 * is_SW6_active +
                        8 * is_SW7_active + 16 * is_SW8_active + 32 * is_SW9_active;

    int desired_utilization_loc = 2 * binary_number;
    if (desired_utilization_loc > 100) {
        desired_utilization_loc = 100;
    }

    return desired_utilization_loc;
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 
void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
        delay,
        alarm_handler,
        context) < 0)
  {
    printf("No system clock available!n");
  }

  /* 
   * Create and start Software Timer 
   */

  /*
   * Creation of Kernel Objects
   */

  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 1); /* Empty Mailbox - Brake */
  Mbox_Engine = OSMboxCreate((void*) 1); /* Empty Mailbox - Engine */

  /*
   * Create statistics task
   */
  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */
err = OSTaskCreateExt(
      ControlTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      CONTROLTASK_PRIO,
      CONTROLTASK_PRIO,
      (void *)&ControlTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      VehicleTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      VEHICLETASK_PRIO,
      VEHICLETASK_PRIO,
      (void *)&VehicleTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      ButtonIO, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ButtonIO_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      BUTTONIO_PRIO,
      BUTTONIO_PRIO,
      (void *)&ButtonIO_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      SwitchIO, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &SwitchIO_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      SWITCHIO_PRIO,
      SWITCHIO_PRIO,
      (void *)&SwitchIO_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);


  err = OSTaskCreateExt(
      Watchdog, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &Watchdog_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      WATCHDOG_PRIO,
      WATCHDOG_PRIO,
      (void *)&Watchdog_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      Overload, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &Overload_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      OVERLOAD_PRIO,
      OVERLOAD_PRIO,
      (void *)&Overload_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      ExtraLoad, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ExtraLoad_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      EXTRALOAD_PRIO,
      EXTRALOAD_PRIO,
      (void *)&ExtraLoad_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

void vehicleCallback(){
  OSSemPost(Vehicle_Semaphore);
}

void controlCallback(){
  OSSemPost(Control_Semaphore);
}

void buttonCallback(){
  OSSemPost(Buttons_Semaphore);
}

void switchCallback(){
  OSSemPost(Switches_Semaphore);
}

void watchdogCallback(){
  OSSemPost(Watchdog_Semaphore);
}

void overloadCallback(){
  OSSemPost(Overload_Semaphore);
}

void extraLoadCallback(){
  OSSemPost(ExtraLoad_Semaphore);
}

/*
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 */
int main(void) {
  printf("Lab: Cruise Control\n");

  OSTaskCreateExt(
      StartTask, // Pointer to task code
      NULL,      // Pointer to argument that is
      // passed to task
      (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack 
      STARTTASK_PRIO,
      STARTTASK_PRIO,
      (void *)&StartTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,  
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  Vehicle_Semaphore = OSSemCreate(0);
  Control_Semaphore = OSSemCreate(0);
  Buttons_Semaphore = OSSemCreate(0);
  Switches_Semaphore = OSSemCreate(0);
  Watchdog_Semaphore = OSSemCreate(0);
  Overload_Semaphore = OSSemCreate(0);
  OKSignal_Semaphore = OSSemCreate(0);
  ExtraLoad_Semaphore = OSSemCreate(0);

  INT8U err; 
  Vehicle_Timer = OSTmrCreate(0,
                              VEHICLE_PERIOD * 0.001 * OS_TMR_CFG_TICKS_PER_SEC, // * 0.001 conversion from ms to s
                              OS_TMR_OPT_PERIODIC,
                              vehicleCallback,
                              NULL,
                              "Vehicle Timer",
                              &err);

  Control_Timer = OSTmrCreate(0,
                              CONTROL_PERIOD * 0.001 * OS_TMR_CFG_TICKS_PER_SEC, // * 0.001 conversion from ms to s
                              OS_TMR_OPT_PERIODIC,
                              controlCallback,
                              NULL,
                              "Control Timer",
                              &err);

  Buttons_Timer = OSTmrCreate(0,
                              BUTTONS_PERIOD * 0.001 * OS_TMR_CFG_TICKS_PER_SEC, // * 0.001 conversion from ms to s
                              OS_TMR_OPT_PERIODIC,
                              buttonCallback,
                              NULL,
                              "Buttons Timer",
                              &err);

  Switches_Timer = OSTmrCreate(0,
                              SWITCHES_PERIOD * 0.001 * OS_TMR_CFG_TICKS_PER_SEC, // * 0.001 conversion from ms to s
                              OS_TMR_OPT_PERIODIC,
                              switchCallback,
                              NULL,
                              "Switches Timer",
                              &err);

  Watchdog_Timer = OSTmrCreate(0,
                              WATCHDOG_PERIOD * 0.001 * OS_TMR_CFG_TICKS_PER_SEC, // * 0.001 conversion from ms to s
                              OS_TMR_OPT_PERIODIC,
                              watchdogCallback,
                              NULL,
                              "Switches Timer",
                              &err);

  Overload_Timer = OSTmrCreate(0,
                              OVERLOAD_PERIOD * 0.001 * OS_TMR_CFG_TICKS_PER_SEC, // * 0.001 conversion from ms to s
                              OS_TMR_OPT_PERIODIC,
                              overloadCallback,
                              NULL,
                              "Switches Timer",
                              &err);

  ExtraLoad_Timer = OSTmrCreate(0,
                              EXTRALOAD_PERIOD * 0.001 * OS_TMR_CFG_TICKS_PER_SEC, // * 0.001 conversion from ms to s
                              OS_TMR_OPT_PERIODIC,
                              extraLoadCallback,
                              NULL,
                              "Switches Timer",
                              &err);

  OSTmrStart(Vehicle_Timer, &err);
  OSTmrStart(Control_Timer, &err);
  OSTmrStart(Buttons_Timer, &err);
  OSTmrStart(Switches_Timer, &err);
  OSTmrStart(Watchdog_Timer, &err);
  OSTmrStart(Overload_Timer, &err);
  OSTmrStart(ExtraLoad_Timer, &err);

  OSStart();

  return 0;
}