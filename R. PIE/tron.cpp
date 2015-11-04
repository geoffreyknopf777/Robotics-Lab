#include<stdio.h>
#include "pid.h"
#include "sonar.h"
#include "drive.h"

void Init(){
    SonarInit();
}

int main() {

    PIDController myPID(1, 0, 0);
    DriveController myDrive;
    float goalDistance = 5, curDistance = 0;
    float pidOutput = 0;
    Init();
	
    myDrive.DriveLinear(10);
	int i = 3;
    while(i < 10) {
	printf("\nCurrr Distance value - %d ", i); 
        curDistance = i++;//SonarGetCM();
        pidOutput = myPID.GetPIDValue(goalDistance, curDistance);       
	myDrive.DriveRotational(pidOutput);
	printf("\n%f ", pidOutput); 
    }
    return 0;
}
