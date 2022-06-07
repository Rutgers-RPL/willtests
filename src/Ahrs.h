#ifndef AHRS_H
#define AHRS_H
#include <Arduino.h>
#include <Vec3.h>
#include <Quaternion.h>

#define _g_ (9.80665)

class Ahrs{
    public:
        double cornerFrequency = 0.05;

        double wmin = 4; //threshold under which the gyro is considered stationary
        double tb = 2; //time threshold after which we activate gyro bias compensation
        Vec3 fb;//time spent moving

        double ta = 0.1;
        double grange = 0.1;
        double fa = 0;

        double Knorm = 0.5;
        double Kinit = 15;
        double tinit = 3;

        Quaternion q = Quaternion();

        Quaternion aglobal = Quaternion();

        long lastTime;

        Ahrs(){
            lastTime = micros();
        }

        void update(Vec3 acc, Vec3 gyr, Vec3 mag){
            
            // Scale acc to in terms of g
            acc = acc * (1/_g_);
            //Serial.printf("scaledacc: %f, %f, %f\n", acc.x, acc.y, acc.z);

            double time2 = micros();
            double delta = ((double)(time2-lastTime))/1000000;
            lastTime = time2;

            double K;
            if(time2/1000000 < tinit) K = Knorm + ((tinit-(time2/1000000))/tinit)*(Kinit - Knorm);
            else K = Knorm;

            Vec3 wprime = gyr;
            if (gyr.x < wmin){
                fb.x += delta;
                if(fb.x > tb){
                    double dwx = gyr.x * delta;
                    wprime.x -= gyroBiasCompensation(dwx);
                }
            } else fb.x = 0;
            if (gyr.y < wmin){
                fb.y += delta;
                if(fb.y > tb){
                    double dwy = gyr.y * delta;
                    wprime.y -= gyroBiasCompensation(dwy);
                }
            } else fb.y = 0;
            if (gyr.z < wmin){
                fb.z += delta;
                if(fb.z > tb){
                    double dwz= gyr.z * delta;
                    wprime.z -= gyroBiasCompensation(dwz);
                }
            } else fb.z = 0;


            Vec3 mprime(0,0,0);
            if (mag.magnitude() > 22 && mag.magnitude() < 67)
                mprime = mag;

            Vec3 aprime = acc;
            if(!(acc.magnitude() - 1 > -1 * grange && acc.magnitude() - 1 < grange)){ //the acceleration has NOT exceeded the range for gravity
                fa+=delta;
                if(fa > ta)
                    aprime = Vec3(0, 0, 0);
            }
            else fa = 0;
            Vec3 gainAdjustedw = wprime + (errorTerm(aprime, mprime, q)*K);
            Quaternion qdot = (q * 0.5) * Quaternion(gainAdjustedw.x, gainAdjustedw.y, gainAdjustedw.z);
            q += qdot * delta;
            q = q.normalize();

            Vec3 onegoffset = Vec3(2*(q.b*q.d)-2*(q.a*q.c), 
                                   2*(q.c*q.d)+2*(q.a*q.b), 
                                  (2*pow(q.a, 2.0))-1+(2*pow(q.d, 2.0))) * -1;

            //Serial.printf("onegoffset: %f, %f, %f, %f\n", onegoffset.x, onegoffset.y, onegoffset.z, onegoffset.magnitude());
            Vec3 azero = acc+(onegoffset);
            
            //Serial.printf("azero: %f, %f, %f\n", azero.x, azero.y, azero.z);
            aglobal = q.rotate(Quaternion(azero.x, azero.y, azero.z));
            aglobal = aglobal * _g_;
        }

        Vec3 errorTerm(Vec3 acc, Vec3 mag, Quaternion q){
            Vec3 ahat = acc;
            ahat.normalize();
            Vec3 ea = ahat.cross(Vec3(2*(q.b*q.d)-2*(q.a*q.c),
                                  2*(q.c*q.d)+2*(q.a*q.b),
                                  2*pow(q.a,2)-1+2*pow(q.d,2)));

            Vec3 em = acc.cross(mag);
            em.normalize();
            em=em.cross(Vec3(2*(q.b*q.c)+2*(q.a*q.d),
                         2*pow(q.a,2)-1+2*pow(q.c,2),
                         2*(q.c*q.d)-2*(q.a*q.d)));

            Vec3 e(0,0,0);
            if(acc.magnitude() > 0 && mag.magnitude() > 0)
                e = ea + em;
            else if(acc.magnitude() > 0)
                e = ea;
            return e;
        }

        double gyroBiasCompensation(double gyr){ //only call if stationary. pass component of integrated gyro output (dw)
            return 2*PI*cornerFrequency*gyr;
        }
};
#endif