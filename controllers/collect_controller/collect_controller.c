#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/radar.h>
#include <webots/touch_sensor.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <stdlib.h>
#include <stdio.h>

#define TIME_STEP 32
#define LEFT 0
#define RIGHT 1
#define MIDDLE 2
#define BOTTOM 0
#define TOP 1
#define X 0
#define Z 2
#define SPEED 10
#define HINGE_SPEED 1
#define HINGE_SPEED_TOP 2
#define FINGER_SPEED 10
#define PI M_PI
#define NORTH PI/2
#define EAST -PI
#define SOUTH -PI/2
#define WEST 0
WbDeviceTag distance[3];
WbDeviceTag wheels[2];
WbDeviceTag hinge[2];
WbDeviceTag finger[4];
WbDeviceTag touch[4];
WbDeviceTag radar;
WbDeviceTag gps;
WbDeviceTag compass;

char ds_names[3][10] = {"ds_left", "ds_right", "ds_middle"};
char wheels_names[2][10] = {"wheel_lf", "wheel_rt"};
char hinge_names[2][10] = {"hinge1", "hinge2"};
char finger_names[4][10] = {"finger1", "finger2", "finger3", "finger4"};
char touch_names[4][10] = {"touch1", "touch2", "touch3", "touch4"};
typedef enum {WANDERING, POSITIONING, EXTRACTING, RETURNING, DISPOSING} states;
double grabAngle = 0;

int isClose(double *distanceValues) {
    if (distanceValues[LEFT] < 950.0 || distanceValues[RIGHT] < 950.0 || distanceValues[MIDDLE] < 950.0) {
        return 1;
    }
    else {
        return 0;
    }
}

int isAngle(double angle, double direction) {
    if(angle < direction - .1 || angle > direction + .1) {
        return 1;
    }
    else {
        return 0;
    }
}

int isInContact(double* touchValues) {
    if (touchValues[0] == 1 && touchValues[1] == 1 && touchValues[2] == 1 && touchValues[3] == 1) {
        return 1;
    }
    else {
        return 0;
    }
}

int isSomeInContact(double* touchValues) {
    if (touchValues[0] == 1 || touchValues[1] == 1 || touchValues[2] == 1 || touchValues[3] == 1) {
        return 1;
    }
    else {
        return 0;
    }
}

void updateDistanceValues(WbDeviceTag *distance, double *distanceValues){
    for(int i = 0; i < 3; i++) {
        distanceValues[i] = wb_distance_sensor_get_value(distance[i]);
    }
}

void rotateCW(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[LEFT], 1);
    wb_motor_set_velocity(wheels[RIGHT], -1);
    wb_robot_step(TIME_STEP);
}

void rotateCCW(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[LEFT], -1);
    wb_motor_set_velocity(wheels[RIGHT], 1);
    wb_robot_step(TIME_STEP);
}

void driveStraight(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[LEFT], SPEED);
    wb_motor_set_velocity(wheels[RIGHT], SPEED);
    wb_robot_step(TIME_STEP);
}

void stopWheels(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[LEFT], 0);
    wb_motor_set_velocity(wheels[RIGHT], 0);
    wb_robot_step(TIME_STEP);
}

void avoidObstacle(WbDeviceTag *wheels, double *distanceValues) {
    if(isClose(distanceValues)) {
        rotateCW(wheels);
        wb_robot_step(3000);
    }
}

void repositionHead(WbDeviceTag *wheels, double *distanceValues) {
    if (distanceValues[MIDDLE] < 850) {
        wb_motor_set_velocity(wheels[LEFT], -1);
        wb_motor_set_velocity(wheels[RIGHT], -1);
        wb_robot_step(100);
    }
    else if(distanceValues[LEFT] < 950) {
        rotateCCW(wheels);
    }
    else if(distanceValues[RIGHT] < 950) {
        rotateCW(wheels);
    }
}


int main(int argc, char **argv) {
    int i;
    states state = WANDERING;
    wb_robot_init();

    for (i = 0; i < 3; i++) {
        distance[i] = wb_robot_get_device(ds_names[i]);
        wb_distance_sensor_enable(distance[i], TIME_STEP);
    }

    for (i = 0; i < 2; i++) {
        wheels[i] = wb_robot_get_device(wheels_names[i]);
        wb_motor_set_position(wheels[i], INFINITY);
    }

    for (i = 0; i < 2; i++) {
        hinge[i] = wb_robot_get_device(hinge_names[i]);
        wb_motor_set_velocity(hinge[i], HINGE_SPEED);
    }
    wb_motor_set_velocity(hinge[1], HINGE_SPEED_TOP);

    for (i = 0; i < 4; i++) {
        finger[i] = wb_robot_get_device(finger_names[i]);
        wb_motor_set_velocity(finger[i], FINGER_SPEED);
    }

    for (i = 0; i < 4; i++) {
        touch[i] = wb_robot_get_device(touch_names[i]);
        wb_touch_sensor_enable(touch[i], TIME_STEP);
    }

    radar = wb_robot_get_device("radar");
    wb_radar_enable(radar, TIME_STEP);

    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);

    compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, TIME_STEP);

    while (wb_robot_step(TIME_STEP) != -1) {
        double distanceValues[3];
        updateDistanceValues(distance, distanceValues);
        double touchValues[4];
        for(i = 0; i < 4; i++) {
            touchValues[i] = wb_touch_sensor_get_value(touch[i]);
        }
        int numOfObjects = wb_radar_get_number_of_targets(radar);

        if(state == WANDERING) {
            if (numOfObjects > 0 || !isClose(distanceValues)) {
                driveStraight(wheels);
                if(isClose(distanceValues)) {
                    stopWheels(wheels);
                    state = POSITIONING;
                }
            }
            else if(isClose(distanceValues)) {
                avoidObstacle(wheels, distanceValues);
            }
        }
        else if(state == POSITIONING) {
            if(distanceValues[LEFT] < 950 || distanceValues[RIGHT] < 950 || distanceValues[MIDDLE] > 950) {
                repositionHead(wheels, distanceValues);
            }
            else {
                stopWheels(wheels);
                state = EXTRACTING;
            }
        }
        else if(state == EXTRACTING) {
            for(i = 0; i < 2; i++) {
                wb_robot_step(TIME_STEP);
                wb_motor_set_position(hinge[i], PI/2);
            }
            while (!isInContact(touchValues)) {
                wb_robot_step(TIME_STEP);
                grabAngle = grabAngle + 0.01;
                for(i = 0; i < 4; i++) {
                    wb_motor_set_position(finger[i], grabAngle);
                }
                for(i = 0; i < 4; i++) {
                    touchValues[i] = wb_touch_sensor_get_value(touch[i]);
                }
            }
            for(i = 0; i < 2; i++) {
                wb_robot_step(TIME_STEP);
                wb_motor_set_position(hinge[i], 0);
            }
            wb_robot_step(2000);
            state = RETURNING;
        }
        else if(state == RETURNING) {
            const double *gpsValues = wb_gps_get_values(gps);
            const double *compassValue = wb_compass_get_values(compass);
            double angle = atan2(compassValue[X], compassValue[Z]);
            if(gpsValues[X] > 0.2) {
                while(isAngle(angle, EAST)) {
                    rotateCW(wheels);
                    angle = atan2(compassValue[X], compassValue[Z]);
                }
                while(gpsValues[X] > 0.15) {
                    avoidObstacle(wheels, distanceValues);
                    driveStraight(wheels);
                    gpsValues = wb_gps_get_values(gps);
                    updateDistanceValues(distance, distanceValues);
                }
            }
            else if(gpsValues[X] < -0.2) {
                while(isAngle(angle, WEST)) {
                    rotateCW(wheels);
                    angle = atan2(compassValue[X], compassValue[Z]);
                }
                while(gpsValues[X] < -0.15) {
                    avoidObstacle(wheels, distanceValues);
                    driveStraight(wheels);
                    gpsValues = wb_gps_get_values(gps);
                    updateDistanceValues(distance, distanceValues);
                }
            }
            else if(gpsValues[Z] > 0.2) {
                while(isAngle(angle, SOUTH)) {
                    rotateCW(wheels);
                    angle = atan2(compassValue[X], compassValue[Z]);
                }
                while(gpsValues[Z] > 0.15) {
                    avoidObstacle(wheels, distanceValues);
                    driveStraight(wheels);
                    gpsValues = wb_gps_get_values(gps);
                    updateDistanceValues(distance, distanceValues);
                }
            }
            else if(gpsValues[Z] < -0.2) {
                while(isAngle(angle, NORTH)) {
                    rotateCW(wheels);
                    angle = atan2(compassValue[X], compassValue[Z]);
                }
                while(gpsValues[Z] < -0.15) {
                    avoidObstacle(wheels, distanceValues);
                    driveStraight(wheels);
                    gpsValues = wb_gps_get_values(gps);
                    updateDistanceValues(distance, distanceValues);
                }
            }
            else {
                stopWheels(wheels);
                state = DISPOSING;
            }
        }
        else if(state == DISPOSING) {
            if(!isClose(distanceValues)) {
                driveStraight(wheels);
                state = RETURNING;
            }
            else if(distanceValues[MIDDLE] > 875) {
                repositionHead(wheels, distanceValues);
            }
            else if(isSomeInContact(touchValues)) {
                stopWheels(wheels);
                wb_motor_set_position(hinge[TOP], PI/2);
                wb_robot_step(500);
                for(i = 0; i < 4; i++) {
                    wb_motor_set_position(finger[i], 0);
                }
                wb_robot_step(2000);
            }
            else if(!isInContact(touchValues)) {
                wb_motor_set_position(hinge[TOP], 0);
                wb_robot_step(TIME_STEP);
                grabAngle = 0;
                wb_robot_step(2000);
                state = WANDERING;
            }
        }
    }
    wb_robot_cleanup();
    return 0;
}



