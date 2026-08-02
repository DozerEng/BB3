/**
 * pid.c
 *
 *      Author: Michael Pillon
 *
 */

#include "pid.h"

pid_t pid_new(float kp, float ki, float kd) {
	pid_t newPid;

	newPid.kp = kp;
	newPid.ki = ki;
	newPid.kd = kd;

	newPid.base.step = pid_step;

	newPid.previousError = 0;
	newPid.integrator = 0;

	// These can be manually configured by the user if limits are needed
	newPid.windupMax = FLT_MAX;
	newPid.outputMax = FLT_MAX;

}

int32_t pid_step(controller_base_t *self, int32_t setPoint, int32_t measurement) {
    pid_t *pid = (pid_t *)self;

    // ToDo: Implement PID algorithm
    return 0;
}
