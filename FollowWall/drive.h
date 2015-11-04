#include <stdio.h>
#include <stdlib.h>
#include "wiringPi.h"
#define DRIVE_ZERO 152

struct Wheels{
    int linearVelocity;
    int rotationalVelocity;
    int left;
    int right;
};

FILE *fp;

void DriveInit(Wheels *myWheel){
	fp = fopen("/dev/servoblaster", "w");
	if(fp == NULL){
		printf("Error opening servo blaster file\n");
		exit(0);
	}

	myWheel->left = DRIVE_ZERO;
	myWheel->right = DRIVE_ZERO;
	myWheel->linearVelocity = 0;
	myWheel->rotationalVelocity = 0;
}

void DriveUpdateWheels(Wheels *myWheel){
   if(myWheel->rotationalVelocity > (myWheel->linearVelocity-2))
	myWheel->rotationalVelocity = myWheel->linearVelocity -2;
   if(myWheel->rotationalVelocity < (-myWheel->linearVelocity+2))
	myWheel->rotationalVelocity = -myWheel->linearVelocity +2;
    myWheel->left = DRIVE_ZERO + myWheel->linearVelocity + myWheel->rotationalVelocity;   
    myWheel->right = DRIVE_ZERO - myWheel->linearVelocity + myWheel->rotationalVelocity;
    fprintf(fp, "0=%d\n", myWheel->left);
    fprintf(fp, "1=%d\n", myWheel->right);
    fflush(fp);
}

void DriveSpeedLinear(Wheels *myWheel, int linearVelocity){
    myWheel->linearVelocity = linearVelocity;
    DriveUpdateWheels(myWheel);
}

void DriveSpeedRotational(Wheels *myWheel, int rotationalVelocity){
    myWheel->rotationalVelocity = rotationalVelocity;

    DriveUpdateWheels(myWheel);
}
