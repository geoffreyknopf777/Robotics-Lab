#include<stdio.h>
#include "pid.h"
#include "sonar.h"
#include "drive.h"

PIDController *myWallFollower = new PIDController;
Wheels *myWheels = new Wheels;

void Init(){
	sonarInit();
	PIDInit(myWallFollower,0.05,5,0);
	DriveInit(myWheels);
	DriveSpeedLinear(myWheels, 10);
	printf("Init done");
	fflush(stdout);
}

int main() {
	printf("sdfkjadsklf");
	Init();
    while(1) {
        float goalDistance = 50;
        float curDistance = SonarGetCM();
	printf("\nSonar value:%f", curDistance);
        float output = PIDGetValue(myWallFollower, goalDistance, curDistance);       
	printf("\nPID:%f",output);
	fflush(fp);
    	DriveSpeedRotational(myWheels, output);
}
    return 0;
}
