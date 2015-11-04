#include <stdio.h>
#include <stdlib.h>
#include "wiringPi.h"
#define DRIVE_ZERO 152

class DriveController{

private:
    float linearVelocity;
    float rotationalVelocity;
    float left;
    float right;
    FILE* fp;

    void ChangeSpeed(){
        left = DRIVE_ZERO + linearVelocity + rotationalVelocity;   
        right = DRIVE_ZERO - linearVelocity + rotationalVelocity;
        fprintf(fp, "0=%d\n", left);
        fprintf(fp, "1=%d\n", right);
        fflush(fp);
    }

public:

    DriveController() {
    	fp = fopen("/dev/servoblaster", "w");
    	if(fp == NULL){
    		printf("Error opening servo blaster file\n");
    		exit(0);
    	}
    	left = DRIVE_ZERO;
    	right = DRIVE_ZERO;
    	linearVelocity = 0;
    	rotationalVelocity = 0;
    }

    ~DriveController() {
        fclose(fp);
    }

    void DriveLinear(float linearVelocity_){
        linearVelocity = linearVelocity_;
        ChangeSpeed();
    }

    void DriveRotational(float rotationalVelocity_){
        rotationalVelocity = rotationalVelocity_;
        ChangeSpeed();
    }
};
