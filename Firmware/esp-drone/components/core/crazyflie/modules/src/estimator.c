#define DEBUG_MODULE "ESTIMATOR"
#include "debug_cf.h"
#include "cfassert.h"
#include "estimator.h"
#include "estimator_complementary.h"
#include "estimator_kalman.h"
#include "debug_cf.h"
#define ESTIMATOR_NAME anyEstimator
#define DEFAULT_ESTIMATOR complementaryEstimator
static StateEstimatorType currentEstimator = anyEstimator;
static StateEstimatorType requiredEstimator = anyEstimator;

static void initEstimator(const StateEstimatorType estimator);
static void deinitEstimator(const StateEstimatorType estimator);

typedef struct {
  void (*init)(void);
  void (*deinit)(void);
  bool (*test)(void);
  void (*update)(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
  const char* name;
  bool (*estimatorEnqueueTDOA)(const tdoaMeasurement_t *uwb);
  bool (*estimatorEnqueuePosition)(const positionMeasurement_t *pos);
  bool (*estimatorEnqueuePose)(const poseMeasurement_t *pose);
  bool (*estimatorEnqueueDistance)(const distanceMeasurement_t *dist);
  bool (*estimatorEnqueueTOF)(const tofMeasurement_t *tof);
  bool (*estimatorEnqueueAbsoluteHeight)(const heightMeasurement_t *height);
  bool (*estimatorEnqueueFlow)(const flowMeasurement_t *flow);
  bool (*estimatorEnqueueYawError)(const yawErrorMeasurement_t *error);
  //bool (*estimatorEnqueueSweepAngles)(const sweepAngleMeasurement_t *angles);
} EstimatorFcns;

#define NOT_IMPLEMENTED ((void*)0)

static EstimatorFcns estimatorFunctions[] = {
    {
        .init = NOT_IMPLEMENTED,
        .deinit = NOT_IMPLEMENTED,
        .test = NOT_IMPLEMENTED,
        .update = NOT_IMPLEMENTED,
        .name = "None",
        .estimatorEnqueueTDOA = NOT_IMPLEMENTED,
        .estimatorEnqueuePosition = NOT_IMPLEMENTED,
        .estimatorEnqueuePose = NOT_IMPLEMENTED,
        .estimatorEnqueueDistance = NOT_IMPLEMENTED,
        .estimatorEnqueueTOF = NOT_IMPLEMENTED,
        .estimatorEnqueueAbsoluteHeight = NOT_IMPLEMENTED,
        .estimatorEnqueueFlow = NOT_IMPLEMENTED,
        .estimatorEnqueueYawError = NOT_IMPLEMENTED,
        //.estimatorEnqueueSweepAngles = NOT_IMPLEMENTED,
    }, // Any estimator
    {
        .init = estimatorComplementaryInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorComplementaryTest,
        .update = estimatorComplementary,
        .name = "Complementary",
        .estimatorEnqueueTDOA = NOT_IMPLEMENTED,
        .estimatorEnqueuePosition = NOT_IMPLEMENTED,
        .estimatorEnqueuePose = NOT_IMPLEMENTED,
        .estimatorEnqueueDistance = NOT_IMPLEMENTED,
        .estimatorEnqueueTOF = estimatorComplementaryEnqueueTOF,
        .estimatorEnqueueAbsoluteHeight = NOT_IMPLEMENTED,
        .estimatorEnqueueFlow = NOT_IMPLEMENTED,
        .estimatorEnqueueYawError = NOT_IMPLEMENTED,
        //.estimatorEnqueueSweepAngles = NOT_IMPLEMENTED,
    },
    {
        .init = estimatorKalmanInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorKalmanTest,
        .update = estimatorKalman,
        .name = "Kalman",
        .estimatorEnqueueTDOA = estimatorKalmanEnqueueTDOA,
        .estimatorEnqueuePosition = estimatorKalmanEnqueuePosition,
        .estimatorEnqueuePose = estimatorKalmanEnqueuePose,
        .estimatorEnqueueDistance = estimatorKalmanEnqueueDistance,
        .estimatorEnqueueTOF = estimatorKalmanEnqueueTOF,
        .estimatorEnqueueAbsoluteHeight = estimatorKalmanEnqueueAbsoluteHeight,
        .estimatorEnqueueFlow = estimatorKalmanEnqueueFlow,
        .estimatorEnqueueYawError = estimatorKalmanEnqueueYawError,
        //.estimatorEnqueueSweepAngles = estimatorKalmanEnqueueSweepAngles,
    },
};

bool registerRequiredEstimator(StateEstimatorType estimator)
{
  bool isError = false;

  if (anyEstimator != estimator) {
    if (anyEstimator == requiredEstimator) {
      requiredEstimator = estimator;
    }
    else {
      if (requiredEstimator != estimator) {
        isError = true;
        DEBUG_PRINTW("WARNING: Two decks require different estimators\n");
      }
    }
  }

  return isError;
}

StateEstimatorType deckGetRequiredEstimator()
{
  return requiredEstimator;
}

void stateEstimatorInit(StateEstimatorType estimator) {
  stateEstimatorSwitchTo(estimator);
}

void stateEstimatorSwitchTo(StateEstimatorType estimator) {
  if (estimator < 0 || estimator >= StateEstimatorTypeCount) {
    return;
  }

  StateEstimatorType newEstimator = estimator;

  if (anyEstimator == newEstimator) {
    newEstimator = DEFAULT_ESTIMATOR;
  }

  StateEstimatorType forcedEstimator = ESTIMATOR_NAME;
  if (forcedEstimator != anyEstimator) {
    DEBUG_PRINTI("Estimator type forced\n");
    newEstimator = forcedEstimator;
  }

  initEstimator(newEstimator);
  StateEstimatorType previousEstimator = currentEstimator;
  currentEstimator = newEstimator;
  deinitEstimator(previousEstimator);

  DEBUG_PRINTI("Using %s (%d) estimator\n", stateEstimatorGetName(), currentEstimator);
}

StateEstimatorType getStateEstimator(void) {
  return currentEstimator;
}

static void initEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].init) {
    estimatorFunctions[estimator].init();
  }
}

static void deinitEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].deinit) {
    estimatorFunctions[estimator].deinit();
  }
}

bool stateEstimatorTest(void) {
  return estimatorFunctions[currentEstimator].test();
}

void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
  estimatorFunctions[currentEstimator].update(state, sensors, control, tick);
}

const char* stateEstimatorGetName() {
  return estimatorFunctions[currentEstimator].name;
}


bool estimatorEnqueueTDOA(const tdoaMeasurement_t *uwb) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueTDOA) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueTDOA(uwb);
  }

  return false;
}

bool estimatorEnqueueYawError(const yawErrorMeasurement_t* error) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueYawError) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueYawError(error);
  }

  return false;
}

bool estimatorEnqueuePosition(const positionMeasurement_t *pos) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueuePosition) {
    return estimatorFunctions[currentEstimator].estimatorEnqueuePosition(pos);
  }

  return false;
}

bool estimatorEnqueuePose(const poseMeasurement_t *pose) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueuePose) {
    return estimatorFunctions[currentEstimator].estimatorEnqueuePose(pose);
  }

  return false;
}

bool estimatorEnqueueDistance(const distanceMeasurement_t *dist) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueDistance) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueDistance(dist);
  }

  return false;
}

bool estimatorEnqueueTOF(const tofMeasurement_t *tof) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueTOF) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueTOF(tof);
  }

  return false;
}

bool estimatorEnqueueAbsoluteHeight(const heightMeasurement_t *height) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueAbsoluteHeight) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueAbsoluteHeight(height);
  }

  return false;
}

bool estimatorEnqueueFlow(const flowMeasurement_t *flow) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueFlow) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueFlow(flow);
  }

  return false;
}

// bool estimatorEnqueueSweepAngles(const sweepAngleMeasurement_t *angles) {
//   if (estimatorFunctions[currentEstimator].estimatorEnqueueSweepAngles) {
//     return estimatorFunctions[currentEstimator].estimatorEnqueueSweepAngles(angles);
//   }

//   return false;
// }
