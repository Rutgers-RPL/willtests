#ifndef NORMALDISTRIBUTION_H
#define NORMALDISTRIBUTION_H
#include <Arduino.h>
#define LONGMAX 2147483647
class NormalDistribution{
    // P(x) = (e^((x-m)^2/-2s))/(s*sqrt(2*pi))

    // we calculate some constants to make future calculations faster:
    // a = 1/(s*sqrt(2*pi))
    double a;
    // b = -1/(2s)
    double b;
    // P(x) = ae^(b(x-m)^2)
    public:
        double s;
        double m;
        NormalDistribution(double mean, double stddev){
            m = mean;
            s = stddev;
            a = 1/(stddev*sqrt(2*PI));
            b = -1/(2*stddev);
        }

        double p(double x){
            // P(x) = ae^(b(x-m)^2)
            return a*exp(b*(x-m)*(x-m));
        }

        double sample(){
            //Using Box-Muller transform: https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
            double u1 = (double)random(0,LONGMAX)/LONGMAX;
            double u2 = (double)random(0,LONGMAX)/LONGMAX;
            double z0 = sqrt(-2*log(u1))*cos(2*PI*u2);

            return s*z0+m;
        }
};


#endif