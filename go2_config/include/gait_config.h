#ifndef GAIT_CONFIG_H
#define GAIT_CONFIG_H

// Leg geometry and kinematics
#define KNEE_ORIENTATION ">>"  // knee bend direction for all four legs
#define ODOM_SCALER 1.25       // multiplier applied to leg-odometry output
#define PANTOGRAPH_LEG false   // true if the leg uses a pantograph linkage

// Velocity limits commanded to the gait generator
#define MAX_LINEAR_VELOCITY_X 0.5   // m/s, forward/back
#define MAX_LINEAR_VELOCITY_Y 0.25  // m/s, lateral
#define MAX_ANGULAR_VELOCITY_Z 1.0  // rad/s, yaw

// Gait shape and timing
#define COM_X_TRANSLATION 0.0  // m, fore/aft center-of-mass offset
#define SWING_HEIGHT 0.04      // m, foot lift during swing phase
#define STANCE_DEPTH 0.00      // m, foot push-down during stance phase
#define STANCE_DURATION 0.25   // s, time each foot spends on the ground per cycle
#define NOMINAL_HEIGHT 0.20    // m, default body height above ground

#endif