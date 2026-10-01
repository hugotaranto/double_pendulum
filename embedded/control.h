#ifndef CONTROL_H
#define CONTROL_H

#include "util.h"
#include "AS5048A.h"


/* ---------------- Encoder --------------- */
const int32_t COUNTS_PER_REV = 16384;
const int32_t HALF_REV = 8192;
const float alpha = 0.40;
const float beta = 0.15;

/* --------------- Mechanics -------------- */

const double PULLEY_PITCH_RADIUS = 0.00637;
const double BELT_DISTANCE_PER_REV = 0.040;

/* -------------- ODRV Const -------------- */

const double TORQUE_SCALE_FACTOR = 1.2;
const double TORQUE_CONSTANT_ADD = 0.2;


class PendulumController {

  public:

    enum Mode {
      FORCE,
      VELOCITY
    };

    PendulumController(AS5048A &encoder, const double K[4], 
                        const double target_state[4],
                        Mode mode, float max_torque, float max_velocity);

    // setup with variables
    void calibrate(float center_pose, float zero_raw, float last_raw, float unwrapped_raw);

    // run an iteration
    double iterate();

    // check if there is an error
    bool getError();

    void setInput(double input);


  private:

    void getEncoderState();
    void getOdrvState();

    double computeOutput();

    bool fault;

    // as5048a stuff
    AS5048A &encoder;
    bool encoder_filter_initialised;
    double x_est_pos;
    double x_est_vel;
    unsigned long last_encoder_read_time;

    Mode mode;

    // odrv calibration
    float center_pose;
    float max_torque;
    float max_velocity;

    // as5048a calibration
    float zero_raw;
    float last_raw;
    float unwrapped_raw;

    double K[4];
    double target_state[4];

    double x[4];

};

#endif
