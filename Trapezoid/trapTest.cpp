#include <math.h>
//if no position just stress testing
//remove pos

//call in main for (deltaT=0;deltaT<number*maxV/maxA;deltaT+=.01{call}
//usr has a position that updates, user has vel or derivative of pos
void trapMotion(double maxA,double maxV,double deltaT,double pos,double vel){
    double timeMaxV=maxV/maxA;
    double distMaxV=.5*maxA*timeMaxV*timeMaxV;
    if(deltaT<timeMaxV){
        vel=maxA*deltaT;
        pos=.5*maxA*deltaT*deltaT;
    } else if (deltaT<2*timeMaxV){
        double timeFromMaxVel=deltaT-timeMaxV;
        vel=maxV-maxA*timeMaxV;
        pos=distMaxV+maxV*timeFromMaxVel-.5*maxA*timeFromMaxVel*timeFromMaxVel;

    }else {
        vel=0;
        pos=2*distMaxV;
        //pos is target
        //vel is target 
    }
}
int main(){

    double pos=0,double vel=0;
    double maxSpeed=200*2*3.1415/60;
    double acc=20*2*3.1415/360;
    for(double deltaT=0;deltaT<maxSpeed/acc;deltaT+=.01){
        trapMotion(acc,maxSpeed,deltaT,pos,vel);
    }
}