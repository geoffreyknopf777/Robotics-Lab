#include<stdio.h>
#include "sonar.h"
#include "drive.h"

#define ARRAY_LEN(X) (sizeof(X) / sizeof((X)[0]))
#define MIN_THRESHOLD 20
#define RADIUS 150

void Init(){
    SonarInit();
}

int getMin(float arr[]) {
    int min = 0;
    for(int i=0; i< 19; i++) {
	//printf("Index:%d, arr: %f",i, arr[i]);
        if(arr[i] <= arr[min])
            min = i;
    }
	//fflush(stdout);

    return min;
}

bool getDirectionToMove(float measurements[], float angles[], float& finalDistance, float& finalAngle) { 
    int minDistId = getMin(measurements);
    finalDistance = 30;//RADIUS - measurements[minDistId];
    finalAngle = angles[minDistId];
//	printf("\nminId:%d", minDistId);
	printf("\nminAngle:%f", finalAngle);
	fflush(stdout);
	
  
  if(abs(measurements[minDistId] - RADIUS) < MIN_THRESHOLD)
       return true;
    return false; 
}

int main() {
   
    DriveController myDrive;
    float measurements[19];
    float angles[19];
    float angle = 0;
    int index = 0;
	float curDistance = 0;
    
    Init();
//    myDrive.DriveLinear(5);
    
    while(true){
   	   myDrive.turnSonar(0);
	angle = 0;
	index = 0;
        while(angle <= 180) {
    	   myDrive.turnSonar(angle);
           curDistance = SonarGetCM();
           measurements[index] = curDistance;
           angles[index] = angle;
           angle += 10;
           index++;
        }
   	   myDrive.turnSonar(0);
        float finalDistance = 0, finalAngle = 0;
        bool isCenter = false;
        isCenter = getDirectionToMove(measurements, angles, finalDistance, finalAngle);
//	printf("\nShouldStop:%d", isCenter);
//	printf("\nSmallestDistance:%d", finalDistance);
//	fflush(stdout);
        if(isCenter) break;
//	printf("\nfinalAngle:%f", finalAngle);
//	fflush(stdout);

        myDrive.DriveTurn(90-finalAngle);
        myDrive.DriveDistance(-finalDistance);
    }
    return 0;
}
