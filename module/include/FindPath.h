#include <list>
#include <math.h>      
#include <stdlib.h>
#include <array>

class FindPath{
    public:    
    struct Instruction{
        double startTurn;
        double endTurn;
        double pathTurn;
        Instruction(double startTurn, double endTurn, double pathTurn):
        startTurn(startTurn), endTurn(endTurn), pathTurn(pathTurn) { 
        }
    };
    // same
    int robotWidth = 30;

    Instruction optimalPath(int pointFromOrigin[2], double angleStart, double angleEnd);
    Instruction circlePath(int pointFromOrigin[2], double angleStart, double angleEnd);

    void arcPath(double x, double y, double *left, double *right);
    void arcPathRadius(double x, double y, double *r, double *t);
    void arcPathDist(double r, double t, double *in, double *out);

    private:
    double minturn(double, double);
    double newangle(double, double);
    double diamToPower(double);
};
