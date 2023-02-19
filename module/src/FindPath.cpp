#include <FindPath.h>

FindPath::Instruction FindPath::optimalPath(int pointFromOrigin[2], double angleStart, double angleEnd){
    


    return FindPath::Instruction(0,0,0);
}

FindPath::Instruction FindPath::circlePath(int pointFromOrigin[2], double angleStart, double angleEnd){
    
    // int pointFromOrigin[2] = {4,-5};
    // double angleStart = 0.6;
    // double angleEnd = 0.5;
    
    double dist = sqrt(pow(pointFromOrigin[0],2) + pow(pointFromOrigin[1],2));
    double angle = atan2(pointFromOrigin[1],pointFromOrigin[0]) / M_PI;

    double angleLeft = newangle(angle, -0.5);
    double angleRight = newangle(angle, 0.5);

    double startLeft1 = abs(minturn(angleStart, angleLeft));
    double startLeft2 = abs(minturn(angleRight, angleEnd));

    double startRight1 = abs(minturn(angleStart, angleRight));
    double startRight2 = abs(minturn(angleLeft, angleEnd));

    std::array<double, 2> turns;
    if((startLeft1 + startLeft2) < (startRight1 + startRight2)){
        turns = {startLeft1, startLeft2};
    }
    else{
        turns = {startRight1, startRight2};
    }

    return FindPath::Instruction(turns[0],turns[1],diamToPower(dist/2));
}

void FindPath::arcPath(double x, double y, double *left, double *right){
    double *r, *t, *in, *out;
    arcPathRadius(x,y,r,t);
    arcPathDist(*r,*t,in,out);

    if (x<0){
        *left = *in;
        *right = *out;
    }
    else{
        *left = *out;
        *right = *in;
    }
}

void FindPath::arcPathRadius(double x, double y, double *r, double *t){
    *r = (y*y)/(2*x) + x/2;
    *t = acos((*r - x)/(*r));
}

void FindPath::arcPathDist(double r, double t, double *in, double *out){
    *in = t * (r - this->robotWidth);
    *out = t * (r + this->robotWidth);
}

double FindPath::minturn(double start, double end){
    double diff = abs(start-end);
    if(diff > 1) diff = 2 - diff;

    if((start+1 < end) || (start-1 < end < start)){
        diff *= -1;
    }

    return diff;
}

double FindPath::newangle(double start, double turn){
    double sum = start + turn + 1;
    double total = (fmod(sum, 2)) - 1;
    return total;
}

double FindPath::diamToPower(double radius){
    return 1 - robotWidth/radius;
}



