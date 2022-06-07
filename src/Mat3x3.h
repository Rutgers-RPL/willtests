#ifndef Mat3x3_h
#define Mat3x3_h
#include "Vec3.h"

class Mat3x3{
    public:
        Vec3 elem[3];
        Mat3x3(Vec3 r1, Vec3 r2, Vec3 r3){
            elem[0] = r1;
            elem[1] = r2;
            elem[2] = r3;
        }
    
}




#endif