#ifndef Vec3_H
#define Vec3_H
#include <Arduino.h>
class Vec3{
    public:
        double x;
        double y;
        double z;
        bool vertical = true;

        Vec3(){
            x = 0;
            y = 0;
            z = 0;
        }

        Vec3(double tx, double ty, double tz){
            x = tx;
            y = ty;
            z = tz;
        }
        void transpose(){
            vertical = !vertical;
        }
        double magnitude(){
            return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
        }
        void normalize(){
            double m = magnitude();
            x/=m;
            y/=m;
            z/=m;
        }
        Vec3 cross(Vec3 vec){
            return Vec3(y*vec.z-z*vec.y,z*vec.x-x*vec.z,x*vec.y-y*vec.x);
        }
        double dot(Vec3 vec){
            return x*vec.x+y*vec.y+z*vec.z;
        }
        Vec3 operator * (const double& s){ 
            return Vec3(x * s, y * s, z*s); 
        }
        Vec3 operator + (const Vec3& a){
            return Vec3(x+a.x,y+a.y,z+a.z);
        }
};

double angleBetweenVec3(Vec3 a,Vec3 b){
    return (180/PI)*acos(a.dot(b)/(a.magnitude()*b.magnitude()));// read somewhere that this isnt the fastest arccos function available
}
#endif