/*
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *

The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/*
High-level commander: computes smooth setpoints based on high-level inputs
such as: take-off, landing, polynomial trajectories.
*/

#include <string.h>
#include <errno.h>
#include <math.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Crazyswarm includes
#include "crtp.h"
#include "crtp_commander_high_level.h"
#include "planner.h"
#include "log.h"
#include "param.h"
#include "stm32_legacy.h"
#include "static_mem.h"

#define DEBUG_MODULE "CTRL_HL"
#include "debug_cf.h"
#include "mem.h"

// Local types
enum TrajectoryLocation_e {
  TRAJECTORY_LOCATION_INVALID = 0,
  TRAJECTORY_LOCATION_MEM     = 1, // for trajectories that are uploaded dynamically
  // Future features might include trajectories on flash or uSD card
};

struct trajectoryDescription
{
  uint8_t trajectoryLocation; // one of TrajectoryLocation_e
  uint8_t trajectoryType;     // one of TrajectoryType_e
  union
  {
    struct {
      uint32_t offset;  // offset in uploaded memory
      uint8_t n_pieces;
    } __attribute__((packed)) mem; // if trajectoryLocation is TRAJECTORY_LOCATION_MEM
  } trajectoryIdentifier;
} __attribute__((packed));

// allocate memory to store trajectories
// 4k allows us to store 31 poly4d pieces
// other (compressed) formats might be added in the future
#define TRAJECTORY_MEMORY_SIZE 4096
extern uint8_t trajectories_memory[TRAJECTORY_MEMORY_SIZE];

#define ALL_GROUPS 0

// Global variables
uint8_t trajectories_memory[TRAJECTORY_MEMORY_SIZE];
static struct trajectoryDescription trajectory_descriptions[NUM_TRAJECTORY_DEFINITIONS];

static bool isInit = false;
static struct planner planner;
static uint8_t group_mask;
static struct vec pos; // last known setpoint (position [m])
static struct vec vel; // last known setpoint (velocity [m/s])
static float yaw; // last known setpoint yaw (yaw [rad])
static struct piecewise_traj trajectory;
static struct piecewise_traj_compressed  compressed_trajectory;

// makes sure that we don't evaluate the trajectory while it is being changed
static xSemaphoreHandle lockTraj;
static StaticSemaphore_t lockTrajBuffer;

// safe default settings for takeoff and landing velocity
static float defaultTakeoffVelocity = 0.5f;
static float defaultLandingVelocity = 0.5f;

// TODO
// Trajectory memory handling from the memory module
// static uint32_t handleMemGetSize(void) { return crtpCommanderHighLevelTrajectoryMemSize(); }
// static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
// static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
// static const MemoryHandlerDef_t memDef = {
//   .type = MEM_TYPE_TRAJ,
//   .getSize = handleMemGetSize,
//   .read = handleMemRead,
//   .write = handleMemWrite,
// };

STATIC_MEM_TASK_ALLOC(crtpCommanderHighLevelTask, CMD_HIGH_LEVEL_TASK_STACKSIZE);

// CRTP Packet definitions

// trajectory command (first byte of crtp packet)
enum TrajectoryCommand_e {
  COMMAND_SET_GROUP_MASK          = 0,
  COMMAND_TAKEOFF                 = 1, // Deprecated, use COMMAND_TAKEOFF_2
  COMMAND_LAND                    = 2, // Deprecated, use COMMAND_LAND_2
  COMMAND_STOP                    = 3,
  COMMAND_GO_TO                   = 4,
  COMMAND_START_TRAJECTORY        = 5,
  COMMAND_DEFINE_TRAJECTORY       = 6,
  COMMAND_TAKEOFF_2               = 7,
  COMMAND_LAND_2                  = 8,
  COMMAND_TAKEOFF_WITH_VELOCITY   = 9,
  COMMAND_LAND_WITH_VELOCITY      = 10,
};

struct data_set_group_mask {
  uint8_t groupMask; // mask for which groups this CF belongs to
} __attribute__((packed));

// vertical takeoff from current x-y position to given height
// Deprecated
struct data_takeoff {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute)
  float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));

// vertical takeoff from current x-y position to given height
struct data_takeoff_2 {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute)
  float yaw;                // rad
  bool useCurrentYaw;       // If true, use the current yaw (ignore the yaw parameter)
  float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));

// vertical takeoff from current x-y position to given height, with prescribed
// velocity
struct data_takeoff_with_velocity {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute or relative)
  bool heightIsRelative;    // If true, height is relative to the current height (positive pointing up)
  float yaw;                // rad
  bool useCurrentYaw;       // If true, use the current yaw (ignore the yaw parameter)
  float velocity;           // m/sec (average velocity during takeoff)
} __attribute__((packed));

// vertical land from current x-y position to given height
// Deprecated
struct data_land {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute)
  float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));

// vertical land from current x-y position to given height
struct data_land_2 {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute)
  float yaw;                // rad
  bool useCurrentYaw;       // If true, use the current yaw (ignore the yaw parameter)
  float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));

// vertical land from current x-y position to given height, with prescribed
// velocity
struct data_land_with_velocity {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute or relative)
  bool heightIsRelative;    // If true, height is relative to the current height (positive pointing down)
  float yaw;                // rad
  bool useCurrentYaw;       // If true, use the current yaw (ignore the yaw parameter)
  float velocity;           // m/s (average velocity during landing)
} __attribute__((packed));

// stops the current trajectory (turns off the motors)
struct data_stop {
  uint8_t groupMask;        // mask for which CFs this should apply to
} __attribute__((packed));

// "take this much time to go here, then hover"
struct data_go_to {
  uint8_t groupMask; // mask for which CFs this should apply to
  uint8_t relative;  // set to true, if position/yaw are relative to current setpoint
  float x; // m
  float y; // m
  float z; // m
  float yaw; // rad
  float duration; // sec
} __attribute__((packed));

// starts executing a specified trajectory
struct data_start_trajectory {
  uint8_t groupMask; // mask for which CFs this should apply to
  uint8_t relative;  // set to true, if trajectory should be shifted to current setpoint
  uint8_t reversed;  // set to true, if trajectory should be executed in reverse
  uint8_t trajectoryId; // id of the trajectory (previously defined by COMMAND_DEFINE_TRAJECTORY)
  float timescale; // time factor; 1 = original speed; >1: slower; <1: faster
} __attribute__((packed));

// starts executing a specified trajectory
struct data_define_trajectory {
  uint8_t trajectoryId;
  struct trajectoryDescription description;
} __attribute__((packed));

// Private functions
static void crtpCommanderHighLevelTask(void * prm);

static int set_group_mask(const struct data_set_group_mask* data);
static int takeoff(const struct data_takeoff* data);
static int land(const struct data_land* data);
static int takeoff2(const struct data_takeoff_2* data);
static int land2(const struct data_land_2* data);
static int takeoff_with_velocity(const struct data_takeoff_with_velocity* data);
static int land_with_velocity(const struct data_land_with_velocity* data);
static int stop(const struct data_stop* data);
static int go_to(const struct data_go_to* data);
static int start_trajectory(const struct data_start_trajectory* data);
static int define_trajectory(const struct data_define_trajectory* data);

// Helper functions
static struct vec state2vec(struct vec3_s v)
{
  return mkvec(v.x, v.y, v.z);
}

bool isInGroup(uint8_t g) {
  return g == ALL_GROUPS || (g & group_mask) != 0;
}

void crtpCommanderHighLevelInit(void)
{
  if (isInit) {
    return;
  }

  //memoryRegisterHandler(&memDef);
  plan_init(&planner);

  //Start the trajectory task
  STATIC_MEM_TASK_CREATE(crtpCommanderHighLevelTask, crtpCommanderHighLevelTask, CMD_HIGH_LEVEL_TASK_NAME, NULL, CMD_HIGH_LEVEL_TASK_PRI);

  lockTraj = xSemaphoreCreateMutexStatic(&lockTrajBuffer);

  pos = vzero();
  vel = vzero();
  yaw = 0;

  isInit = true;
}

bool crtpCommanderHighLevelIsStopped()
{
  return plan_is_stopped(&planner);
}

void crtpCommanderHighLevelTellState(const state_t *state)
{
  xSemaphoreTake(lockTraj, portMAX_DELAY);
  pos = state2vec(state->position);
  vel = state2vec(state->velocity);
  yaw = radians(state->attitude.yaw);
  xSemaphoreGive(lockTraj);
}

void crtpCommanderHighLevelGetSetpoint(setpoint_t* setpoint, const state_t *state)
{
  xSemaphoreTake(lockTraj, portMAX_DELAY);
  float t = usecTimestamp() / 1e6;
  struct traj_eval ev = plan_current_goal(&planner, t);
  if (!is_traj_eval_valid(&ev)) {
    // programming error
    plan_stop(&planner);
  }
  xSemaphoreGive(lockTraj);

  // if we are on the ground, update the last setpoint with the current state estimate
  if (plan_is_stopped(&planner)) {
    pos = state2vec(state->position);
    vel = state2vec(state->velocity);
    yaw = radians(state->attitude.yaw);
  }

  if (is_traj_eval_valid(&ev)) {
    setpoint->position.x = ev.pos.x;
    setpoint->position.y = ev.pos.y;
    setpoint->position.z = ev.pos.z;
    setpoint->velocity.x = ev.vel.x;
    setpoint->velocity.y = ev.vel.y;
    setpoint->velocity.z = ev.vel.z;
    setpoint->attitude.yaw = degrees(ev.yaw);
    setpoint->attitudeRate.roll = degrees(ev.omega.x);
    setpoint->attitudeRate.pitch = degrees(ev.omega.y);
    setpoint->attitudeRate.yaw = degrees(ev.omega.z);
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->mode.quat = modeDisable;
    setpoint->acceleration.x = ev.acc.x;
    setpoint->acceleration.y = ev.acc.y;
    setpoint->acceleration.z = ev.acc.z;

    // store the last setpoint
    pos = ev.pos;
    vel = ev.vel;
    yaw = ev.yaw;
  }
}

static int handleCommand(const enum TrajectoryCommand_e command, const uint8_t* data)
{
  int ret = 0;

  switch(command)
  {
    case COMMAND_SET_GROUP_MASK:
      ret = set_group_mask((const struct data_set_group_mask*)data);
      break;
    case COMMAND_TAKEOFF:
      ret = takeoff((const struct data_takeoff*)data);
      break;
    case COMMAND_LAND:
      ret = land((const struct data_land*)data);
      break;
    case COMMAND_TAKEOFF_2:
      ret = takeoff2((const struct data_takeoff_2*)data);
      break;
    case COMMAND_LAND_2:
      ret = land2((const struct data_land_2*)data);
      break;
    case COMMAND_TAKEOFF_WITH_VELOCITY:
      ret = takeoff_with_velocity((const struct data_takeoff_with_velocity*)data);
      break;
    case COMMAND_LAND_WITH_VELOCITY:
      ret = land_with_velocity((const struct data_land_with_velocity*)data);
      break;
    case COMMAND_STOP:
      ret = stop((const struct data_stop*)data);
      break;
    case COMMAND_GO_TO:
      ret = go_to((const struct data_go_to*)data);
      break;
    case COMMAND_START_TRAJECTORY:
      ret = start_trajectory((const struct data_start_trajectory*)data);
      break;
    case COMMAND_DEFINE_TRAJECTORY:
      ret = define_trajectory((const struct data_define_trajectory*)data);
      break;
    default:
      ret = ENOEXEC;
      break;
  }

  return ret;
}

void crtpCommanderHighLevelTask(void * prm)
{
  CRTPPacket p;
  crtpInitTaskQueue(CRTP_PORT_SETPOINT_HL);

  while(1) {
    crtpReceivePacketBlock(CRTP_PORT_SETPOINT_HL, &p);

    int ret = handleCommand(p.data[0], &p.data[1]);

    //answer
    p.data[3] = ret;
    p.size = 4;
    crtpSendPacket(&p);
  }
}

int set_group_mask(const struct data_set_group_mask* data)
{
  group_mask = data->groupMask;

  return 0;
}

int takeoff(const struct data_takeoff* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    DEBUG_PRINTD("take off !!!!!");
    float t = usecTimestamp() / 1e6;
    result = plan_takeoff(&planner, pos, yaw, data->height, 0.0f, data->duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int takeoff2(const struct data_takeoff_2* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;

    float hover_yaw = data->yaw;
    if (data->useCurrentYaw) {
      hover_yaw = yaw;
    }

    result = plan_takeoff(&planner, pos, yaw, data->height, hover_yaw, data->duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int takeoff_with_velocity(const struct data_takeoff_with_velocity* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;

    float hover_yaw = data->yaw;
    if (data->useCurrentYaw) {
      hover_yaw = yaw;
    }

    float height = data->height;
    if (data->heightIsRelative) {
      height += pos.z;
    }

    float velocity = data->velocity > 0 ? data->velocity : defaultTakeoffVelocity;
    float duration = fabsf(height - pos.z) / velocity;
    result = plan_takeoff(&planner, pos, yaw, height, hover_yaw, duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int land(const struct data_land* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    result = plan_land(&planner, pos, yaw, data->height, 0.0f, data->duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int land2(const struct data_land_2* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;

    float hover_yaw = data->yaw;
    if (data->useCurrentYaw) {
      hover_yaw = yaw;
    }

    result = plan_land(&planner, pos, yaw, data->height, hover_yaw, data->duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int land_with_velocity(const struct data_land_with_velocity* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;

    float hover_yaw = data->yaw;
    if (data->useCurrentYaw) {
      hover_yaw = yaw;
    }

    float height = data->height;
    if (data->heightIsRelative) {
      height = pos.z - height;
    }

    float velocity = data->velocity > 0 ? data->velocity : defaultLandingVelocity;
    float duration = fabsf(height - pos.z) / velocity;
    result = plan_land(&planner, pos, yaw, height, hover_yaw, duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int stop(const struct data_stop* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    plan_stop(&planner);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int go_to(const struct data_go_to* data)
{
  static struct traj_eval ev = {
    // pos, vel, yaw will be filled before using
    .acc = {0.0f, 0.0f, 0.0f},
    .omega = {0.0f, 0.0f, 0.0f},
  };

  int result = 0;
  if (isInGroup(data->groupMask)) {
    struct vec hover_pos = mkvec(data->x, data->y, data->z);
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    if (plan_is_stopped(&planner)) {
      ev.pos = pos;
      ev.vel = vel;
      ev.yaw = yaw;
      result = plan_go_to_from(&planner, &ev, data->relative, hover_pos, data->yaw, data->duration, t);
    }
    else {
      result = plan_go_to(&planner, data->relative, hover_pos, data->yaw, data->duration, t);
    }
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int start_trajectory(const struct data_start_trajectory* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    if (data->trajectoryId < NUM_TRAJECTORY_DEFINITIONS) {
      struct trajectoryDescription* trajDesc = &trajectory_descriptions[data->trajectoryId];
      if (   trajDesc->trajectoryLocation == TRAJECTORY_LOCATION_MEM
          && trajDesc->trajectoryType == CRTP_CHL_TRAJECTORY_TYPE_POLY4D) {
        xSemaphoreTake(lockTraj, portMAX_DELAY);
        float t = usecTimestamp() / 1e6;
        trajectory.t_begin = t;
        trajectory.timescale = data->timescale;
        trajectory.n_pieces = trajDesc->trajectoryIdentifier.mem.n_pieces;
        trajectory.pieces = (struct poly4d*)&trajectories_memory[trajDesc->trajectoryIdentifier.mem.offset];
        if (data->relative) {
          trajectory.shift = vzero();
          struct traj_eval traj_init;
          if (data->reversed) {
            traj_init = piecewise_eval_reversed(&trajectory, trajectory.t_begin);
          }
          else {
            traj_init = piecewise_eval(&trajectory, trajectory.t_begin);
          }
          struct vec shift_pos = vsub(pos, traj_init.pos);
          trajectory.shift = shift_pos;
        } else {
          trajectory.shift = vzero();
        }
        result = plan_start_trajectory(&planner, &trajectory, data->reversed);
        xSemaphoreGive(lockTraj);
      } else if (trajDesc->trajectoryLocation == TRAJECTORY_LOCATION_MEM
          && trajDesc->trajectoryType == CRTP_CHL_TRAJECTORY_TYPE_POLY4D_COMPRESSED) {

        if (data->timescale != 1 || data->reversed) {
          result = ENOEXEC;
        } else {
          xSemaphoreTake(lockTraj, portMAX_DELAY);
          float t = usecTimestamp() / 1e6;
          piecewise_compressed_load(
            &compressed_trajectory,
            &trajectories_memory[trajDesc->trajectoryIdentifier.mem.offset]
          );
          compressed_trajectory.t_begin = t;
          if (data->relative) {
            struct traj_eval traj_init = piecewise_compressed_eval(
              &compressed_trajectory, compressed_trajectory.t_begin
            );
            struct vec shift_pos = vsub(pos, traj_init.pos);
            compressed_trajectory.shift = shift_pos;
          } else {
            compressed_trajectory.shift = vzero();
          }
          result = plan_start_compressed_trajectory(&planner, &compressed_trajectory);
          xSemaphoreGive(lockTraj);
        }

      }
    }
  }
  return result;
}

int define_trajectory(const struct data_define_trajectory* data)
{
  if (data->trajectoryId >= NUM_TRAJECTORY_DEFINITIONS) {
    return ENOEXEC;
  }
  trajectory_descriptions[data->trajectoryId] = data->description;
  return 0;
}

// static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
//   return crtpCommanderHighLevelReadTrajectory(memAddr, readLen, buffer);
// }

// static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
//   return crtpCommanderHighLevelWriteTrajectory(memAddr, writeLen, buffer);
// }

uint8_t* initCrtpPacket(CRTPPacket* packet, const enum TrajectoryCommand_e command)
{
  packet->port = CRTP_PORT_SETPOINT_HL;
  packet->data[0] = command;
  return &packet->data[1];
}

int crtpCommanderHighLevelTakeoff(const float absoluteHeight_m, const float duration_s)
{
  struct data_takeoff_2 data =
  {
    .height = absoluteHeight_m,
    .duration = duration_s,
    .useCurrentYaw = true,
    .groupMask = ALL_GROUPS,
  };

  return handleCommand(COMMAND_TAKEOFF_2, (const uint8_t*)&data);
}

int crtpCommanderHighLevelTakeoffYaw(const float absoluteHeight_m, const float duration_s, const float yaw)
{
  struct data_takeoff_2 data =
  {
    .height = absoluteHeight_m,
    .duration = duration_s,
    .useCurrentYaw = false,
    .yaw = yaw,
    .groupMask = ALL_GROUPS,
  };

  return handleCommand(COMMAND_TAKEOFF_2, (const uint8_t*)&data);
}

int crtpCommanderHighLevelTakeoffWithVelocity(const float height_m, const float velocity_m_s, bool relative)
{
  struct data_takeoff_with_velocity data =
  {
    .height = height_m,
    .heightIsRelative = relative,
    .velocity = velocity_m_s,
    .useCurrentYaw = true,
    .groupMask = ALL_GROUPS,
  };

  return handleCommand(COMMAND_TAKEOFF_WITH_VELOCITY, (const uint8_t*)&data);
}

int crtpCommanderHighLevelLand(const float absoluteHeight_m, const float duration_s)
{
  struct data_land_2 data =
  {
    .height = absoluteHeight_m,
    .duration = duration_s,
    .useCurrentYaw = true,
    .groupMask = ALL_GROUPS,
  };

  return handleCommand(COMMAND_LAND_2, (const uint8_t*)&data);
}

int crtpCommanderHighLevelLandWithVelocity(const float height_m, const float velocity_m_s, bool relative)
{
  struct data_land_with_velocity data =
  {
    .height = height_m,
    .heightIsRelative = relative,
    .velocity = velocity_m_s,
    .useCurrentYaw = true,
    .groupMask = ALL_GROUPS,
  };

  return handleCommand(COMMAND_LAND_WITH_VELOCITY, (const uint8_t*)&data);
}

int crtpCommanderHighLevelLandYaw(const float absoluteHeight_m, const float duration_s, const float yaw)
{
  struct data_land_2 data =
  {
    .height = absoluteHeight_m,
    .duration = duration_s,
    .useCurrentYaw = false,
    .yaw = yaw,
    .groupMask = ALL_GROUPS,
  };

  return handleCommand(COMMAND_LAND_2, (const uint8_t*)&data);
}

int crtpCommanderHighLevelStop()
{
  struct data_stop data =
  {
    .groupMask = ALL_GROUPS,
  };

  return handleCommand(COMMAND_STOP, (const uint8_t*)&data);
}

int crtpCommanderHighLevelGoTo(const float x, const float y, const float z, const float yaw, const float duration_s, const bool relative)
{
  struct data_go_to data =
  {
    .x = x,
    .y = y,
    .z = z,
    .yaw = yaw,
    .duration = duration_s,
    .relative = relative,
    .groupMask = ALL_GROUPS,
  };

  return handleCommand(COMMAND_GO_TO, (const uint8_t*)&data);
}

bool crtpCommanderHighLevelIsTrajectoryDefined(uint8_t trajectoryId)
{
  return (
    trajectoryId < NUM_TRAJECTORY_DEFINITIONS &&
    trajectory_descriptions[trajectoryId].trajectoryLocation != TRAJECTORY_LOCATION_INVALID
  );
}

int crtpCommanderHighLevelStartTrajectory(const uint8_t trajectoryId, const float timeScale, const bool relative, const bool reversed)
{
  struct data_start_trajectory data =
  {
    .trajectoryId = trajectoryId,
    .timescale = timeScale,
    .relative = relative,
    .reversed = reversed,
    .groupMask = ALL_GROUPS,
  };

  return handleCommand(COMMAND_START_TRAJECTORY, (const uint8_t*)&data);
}

int crtpCommanderHighLevelDefineTrajectory(const uint8_t trajectoryId, const crtpCommanderTrajectoryType_t type, const uint32_t offset, const uint8_t nPieces)
{
  struct data_define_trajectory data =
  {
    .trajectoryId = trajectoryId,
    .description.trajectoryLocation = TRAJECTORY_LOCATION_MEM,
    .description.trajectoryType = type,
    .description.trajectoryIdentifier.mem.offset = offset,
    .description.trajectoryIdentifier.mem.n_pieces = nPieces,
  };

  return handleCommand(COMMAND_DEFINE_TRAJECTORY, (const uint8_t*)&data);
}

uint32_t crtpCommanderHighLevelTrajectoryMemSize()
{
  return sizeof(trajectories_memory);
}

bool crtpCommanderHighLevelWriteTrajectory(const uint32_t offset, const uint32_t length, const uint8_t* data)
{
  bool result = false;

  if ((offset + length) <= sizeof(trajectories_memory)) {
    memcpy(&(trajectories_memory[offset]), data, length);
    result = true;
  }

  return result;
}

bool crtpCommanderHighLevelReadTrajectory(const uint32_t offset, const uint32_t length, uint8_t* destination)
{
  bool result = false;

  if (offset + length <= sizeof(trajectories_memory) && memcpy(destination, &(trajectories_memory[offset]), length)) {
    result = true;
  }

  return result;
}

bool crtpCommanderHighLevelIsTrajectoryFinished() {
  float t = usecTimestamp() / 1e6;
  return plan_is_finished(&planner, t);
}

PARAM_GROUP_START(hlCommander)
PARAM_ADD(PARAM_FLOAT, vtoff, &defaultTakeoffVelocity)
PARAM_ADD(PARAM_FLOAT, vland, &defaultLandingVelocity)
PARAM_GROUP_STOP(hlCommander)
