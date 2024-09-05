/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/speaker.h>
#include <webots/device.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <base.h>
#include <gripper.h>

#include "BotSm.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32

enum Arm
{
  ARM1,
  ARM2,
  ARM3,
  ARM4,
  ARM5
};

enum SequenceStateId {
  SSI_PICKUP,
  SSI_PICKUP_1,
  SSI_PICKUP_2,
  SSI_PICKUP_3,
  SSI_PICKUP_DONE,

  SSI_MOVE_LEFT,
  SSI_MOVE_LEFT_1,
  SSI_MOVE_LEFT_DONE,

  SSI_FEED,
  SSI_FEED_1,
  SSI_FEED_DONE,

  SSI_PUSH_FOOD,
  SSI_PUSH_FOOD_1,
  SSI_PUSH_FOOD_DONE,

  SSI_DROP_FOOD,
  SSI_DROP_FOOD_1,
  SSI_DROP_FOOD_2,
  SSI_DROP_FOOD_DONE,

  SSI_CLEAN_KITCHEN,
  SSI_CLEAN_KITCHEN_1,
  SSI_CLEAN_KITCHEN_2,
  SSI_CLEAN_KITCHEN_3,
  SSI_CLEAN_KITCHEN_DONE,

  SSI_DONE,
};

////////////////////////////////////////////////
// variables
////////////////////////////////////////////////

static enum SequenceStateId sequence_state;
static double sequence_timeout_at_time;

static WbDeviceTag arm_elements[5];
static WbDeviceTag speaker;

static BotSm bot_sm; // state machine

////////////////////////////////////////////////
// private function prototypes
////////////////////////////////////////////////

static void arm_init();
// static void automatic_behavior();
static bool is_sequence_complete(void);
static bool is_sequence_running(void);
static bool is_sequence_timer_expired(void);
static bool sequence_delay(double timeout_duration_seconds);
static void sequence_abort(void);
static void sequence_mark_complete(void);
static void sequence_run(void);
static void sequence_start(enum SequenceStateId state_id);
static void speak_init(void);
static void speak(char const * const text);
static bool is_speaking_done(void);
static void webots_step();
static void safe_machine(void);


////////////////////////////////////////////////
// sequence functions
////////////////////////////////////////////////

static void sequence_mark_complete(void)
{
  sequence_state = SSI_DONE;
}

static void sequence_abort(void)
{
  sequence_mark_complete();
}

static void sequence_start(enum SequenceStateId state_id)
{
  sequence_timeout_at_time = 0;
  sequence_state = state_id;
}

static bool is_sequence_timer_expired(void)
{
  double current_time = wb_robot_get_time();
  return current_time > sequence_timeout_at_time;
}

static bool is_sequence_running(void)
{
  return sequence_state != SSI_DONE;
}

static bool is_sequence_complete(void)
{
  return !is_sequence_running();
}

static bool sequence_delay(double timeout_duration_seconds)
{
  double current_time = wb_robot_get_time();
  return sequence_timeout_at_time = current_time + timeout_duration_seconds;
}

static void sequence_run(void)
{
  if (is_sequence_timer_expired() == false)
  {
    // wait until ready for the next step
    return;
  }

  if (is_sequence_complete())
  {
    return;
  }

  // advance to next state in sequence
  sequence_state++;

  switch (sequence_state)
  {
    ////////////////////////////////////////////////
    // pickup
    ////////////////////////////////////////////////
    case SSI_PICKUP:
    case SSI_PICKUP_1:
      speak("Time for your vegetables!");
      gripper_release();
      wb_motor_set_position(arm_elements[ARM1], 0.0);
      wb_motor_set_position(arm_elements[ARM2], 0.0);
      wb_motor_set_position(arm_elements[ARM3], -0.77);
      wb_motor_set_position(arm_elements[ARM4], -1.21);
      sequence_delay(1.2);
      break;

    case SSI_PICKUP_2:
      gripper_grip();
      sequence_delay(1.0);
      break;

    case SSI_PICKUP_3:
      wb_motor_set_position(arm_elements[ARM2], M_PI/2);
      sequence_delay(1.0);
      break;

    case SSI_PICKUP_DONE:
      sequence_mark_complete();
      break;

    ////////////////////////////////////////////////
    // move left
    ////////////////////////////////////////////////
    case SSI_MOVE_LEFT:
    case SSI_MOVE_LEFT_1:
      speak("Here comes the nomm nomm train!");
      base_strafe_left();
      sequence_delay(3.0);
      break;

    case SSI_MOVE_LEFT_DONE:
      base_reset();
      sequence_mark_complete();
      break;

    ////////////////////////////////////////////////
    // feed
    ////////////////////////////////////////////////
    case SSI_FEED:
    case SSI_FEED_1:
      speak("Open up!");
      wb_motor_set_position(arm_elements[ARM2], M_PI * 0.2);
      wb_motor_set_position(arm_elements[ARM3], -M_PI / 2);
      wb_motor_set_position(arm_elements[ARM4], -M_PI / 8);
      sequence_delay(3.0);
      break;

    case SSI_FEED_DONE:
      sequence_mark_complete();
      break;

    ////////////////////////////////////////////////
    // push food
    ////////////////////////////////////////////////
    case SSI_PUSH_FOOD:
    case SSI_PUSH_FOOD_1:
      speak("FINE!... have it your way.");
      base_forwards();
      sequence_delay(1.0);
      break;

    case SSI_PUSH_FOOD_DONE:
      base_reset();
      sequence_mark_complete();
      break;


    ////////////////////////////////////////////////
    // drop food
    ////////////////////////////////////////////////
    case SSI_DROP_FOOD:
    case SSI_DROP_FOOD_1:
      wb_motor_set_position(arm_elements[ARM1], -0.09);
      wb_motor_set_position(arm_elements[ARM2], -0.5);
      wb_motor_set_position(arm_elements[ARM3], -M_PI / 2 + 0.5);
      wb_motor_set_position(arm_elements[ARM4], 0);
      sequence_delay(3.0);
      break;

    case SSI_DROP_FOOD_2:
      speak("Jerk.");
      gripper_release();
      sequence_delay(1.0);
      break;

    case SSI_DROP_FOOD_DONE:
      sequence_mark_complete();
      break;

    ////////////////////////////////////////////////
    // clean kitchen
    ////////////////////////////////////////////////
    case SSI_CLEAN_KITCHEN:
    case SSI_CLEAN_KITCHEN_1:
      // crouch position  
      wb_motor_set_position(arm_elements[ARM2], 1.57);
      wb_motor_set_position(arm_elements[ARM3], -2.635);
      wb_motor_set_position(arm_elements[ARM4], 1.78);

      speak("Now get out of my kitchen you filthy animal.");
      base_forwards();
      sequence_delay(6.0);
      break;

    case SSI_CLEAN_KITCHEN_2:
      base_reset();
      speak("BODY SLAM COMING UP.");
      sequence_delay(2.0);
      break;

    case SSI_CLEAN_KITCHEN_3:
      base_forwards();
      sequence_delay(4.0);
      break;

    case SSI_CLEAN_KITCHEN_DONE:
      speak("OUCH. I HAVE FALLEN AND CAN'T GET UP.");
      sequence_mark_complete();
      break;


    ////////////////////////////////////////////////
    case SSI_DONE:
      break;
  }
}


////////////////////////////////////////////////
// other functions
////////////////////////////////////////////////

static void safe_machine(void)
{
  base_reset();
  wb_motor_set_velocity(arm_elements[ARM1], 0);
  wb_motor_set_velocity(arm_elements[ARM2], 0);
  wb_motor_set_velocity(arm_elements[ARM3], 0);
  wb_motor_set_velocity(arm_elements[ARM4], 0);
  wb_motor_set_velocity(arm_elements[ARM5], 0);
}

static void webots_step()
{
  if (wb_robot_step(TIME_STEP) == -1)
  {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void arm_init()
{
  arm_elements[ARM1] = wb_robot_get_device("arm1"); // base rotation
  arm_elements[ARM2] = wb_robot_get_device("arm2"); // bottom motor
  arm_elements[ARM3] = wb_robot_get_device("arm3"); // middle motor
  arm_elements[ARM4] = wb_robot_get_device("arm4"); // top motor
  arm_elements[ARM5] = wb_robot_get_device("arm5"); // wrist angle

  wb_motor_set_velocity(arm_elements[ARM2], 0.5);
}

static void speak_init(void)
{
  speaker = wb_robot_get_device("speaker");

  #ifdef _WIN32
  wb_speaker_set_engine(speaker, "microsoft");
  #endif
  wb_speaker_set_language(speaker, "en-US");
}

static void speak(char const * const text)
{
  printf("SPEAK: %s\n", text);

  static char buf[1000];
  snprintf(buf, sizeof(buf), "<prosody rate=\"0.75\"><prosody pitch=\"-10st\">%s</prosody></prosody>", text); // assume it works
  wb_speaker_speak(speaker, buf, 1.0);
}

static bool is_speaking_done(void)
{
  return !wb_speaker_is_speaking(speaker);
}


// static void automatic_behavior()
// {
//   sequence_start(SSI_PICKUP);
//   while (is_sequence_running())
//   {
//     sequence_run();
//     webots_step();
//   }

//   sequence_start(SSI_MOVE_LEFT);
//   while (is_sequence_running())
//   {
//     sequence_run();
//     webots_step();
//   }

//   sequence_start(SSI_FEED);
//   while (is_sequence_running())
//   {
//     sequence_run();
//     webots_step();
//   }

//   sequence_start(SSI_PUSH_FOOD);
//   while (is_sequence_running())
//   {
//     sequence_run();
//     webots_step();
//   }

//   sequence_start(SSI_DROP_FOOD);
//   while (is_sequence_running())
//   {
//     sequence_run();
//     webots_step();
//   }

//   sequence_start(SSI_CLEAN_KITCHEN);
//   while (is_sequence_running())
//   {
//     sequence_run();
//     webots_step();
//   }
// }

int main(int argc, char **argv)
{
  wb_robot_init();
  speak_init();

  base_init();
  arm_init();
  gripper_init();
  printf("\n\n");

  BotSm_ctor(&bot_sm);
  BotSm_start(&bot_sm);

  wb_keyboard_enable(TIME_STEP);

  while (true)
  {
    int c = wb_keyboard_get_key();
    if (c > 0)
    {
      BotSm_dispatch_event(&bot_sm, BotSm_EventId_ANY_KEY);
    }
    BotSm_dispatch_event(&bot_sm, BotSm_EventId_DO);
    sequence_run();
    webots_step();
  }

  wb_robot_cleanup();

  return 0;
}


////////////////////////
#include "BotSm.inc"
