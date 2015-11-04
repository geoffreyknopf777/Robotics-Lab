#include <stdio.h>
#include <stdlib.h>
#include "wiringPi.h"
int main(void)
{
 FILE *fp;
 fp = fopen("/dev/servoblaster", "w");
 if (fp == NULL) {
	 printf("Error opening file\n");
	 exit(0);
 }
 while (1) { 

       fprintf(fp, "0=170\n"); //Forward
       fprintf(fp, "1=130\n"); 
       fflush(fp);
       delay(1000);

       fprintf(fp, "0=152\n"); //Stop
       fprintf(fp, "1=152\n"); //Stop
       fflush(fp);
       delay(1000);
	

	//gobackwards - 0=130, 1=170
       fprintf(fp, "0=130\n"); //182 Servo#0, Counter Clockwise
       fprintf(fp, "1=170\n"); //182 Servo#0, Counter Clockwise
       fflush(fp);
       delay(1000);
      
       fprintf(fp, "0=152\n"); //Stop
       fprintf(fp, "1=152\n"); //Stop
       fflush(fp);
       delay(1000);
       
	
       fprintf(fp, "0=130\n"); //Clockwise
       fprintf(fp, "1=130\n"); //Clockwise
       fflush(fp);
       delay(1000);


       //fprintf(fp, "0=170\n"); //Forward
      /// fprintf(fp, "1=130\n"); 
      // fflush(fp);
      // delay(1000);

       fprintf(fp, "0=152\n"); //Stop
       fprintf(fp, "1=152\n"); //Stop
       fflush(fp);
       delay(1000);

       fprintf(fp, "0=170\n"); //Counter Clockwise
       fprintf(fp, "1=170\n"); //Counter Clockwise
       fflush(fp);
       delay(1000);

      // fprintf(fp, "0=170\n"); //Forward
       //fprintf(fp, "1=130\n"); 
       //fflush(fp);
       //delay(1000);

 }
 fclose(fp); 
 return 0;
}
