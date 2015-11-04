#define ARRAY_LEN(X) (sizeof(X) / sizeof((X)[0]))
#define ERROR_SIZE 5

struct PIDController {
    float Kp;
    float Kd;
    float Ki;
    float output;
    float prevError;
    float derivative;
    float integral[ERROR_SIZE];
};


void PIDInit(PIDController *myController, float Kp, float Kd, float Ki){
    myController->Kp = Kp;
    myController->Kd = Kd;
    myController->Ki = Ki;
    myController->output = 0;
    myController->prevError = 0;
    myController->derivative = 0;
    for(int i=0; i<ERROR_SIZE; i++){
        myController->integral[i] = 0;
    }
}


float sumErrors(float Arr[]) {
    float sum=0;
    for(int i=0; i<ERROR_SIZE; i++)
        sum += Arr[i];
    return sum;
}

void RightShiftArray(float Arr[]){
    //Shift each element to the right
    for(int i=ARRAY_LEN(Arr)-1; i>0; i--) {
        Arr[i] = Arr[i-1];
    }
    Arr[0] = 0;
}

int PIDGetValue(PIDController *myController, float goalDistance, float curDistance) {
    float error = -1*goalDistance + curDistance;
  
    //Calculate the PID terms
    float proportionalTerm     = myController->Kp * error;
    float derivativeTerm         = myController->Kd * (error - myController->prevError);
    
    //Right shift integral values and put newest value in left
    RightShiftArray(myController->integral);
    //Insert latest error into first element of array
    myController->integral[0]  = error;
    
    //calculate integralTerm
    float integralTerm         = sumErrors(myController->integral);    
    myController->output       = proportionalTerm + derivativeTerm + integralTerm;
    myController->prevError    = error;
    
    return myController->output;
}
