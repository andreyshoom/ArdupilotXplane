#include "Plane.h"


// from 0 to 360 deg
float Plane::AngleReduction0_360_andr(float angle_andr)
{
    if (angle_andr < 0)
        return angle_andr = angle_andr + 360;
    else if (angle_andr >= 360)
        return angle_andr = angle_andr - 360;
    else
        return angle_andr;
 }
 
float Plane::deg2rad_andr(float degrees_andr) 
{
    return degrees_andr * 3.14159265 / 180.0;
}

float Plane::CourseToPointShortDis(float lat1_andr, float lon1_andr, float lat2_andr, float lon2_andr)
{
    float bb, l, Bm, eta, Nm, Mm, Q, P, aa, Am; 
    bb = deg2rad_andr(lat2_andr) - deg2rad_andr(lat1_andr);
    l = deg2rad_andr(lon2_andr) - deg2rad_andr(lon1_andr);
    Bm = (deg2rad_andr(lat1_andr) + deg2rad_andr(lat2_andr)) / 2;
    eta = 0.0067385254 * powf(cosf(Bm), 2);
    Nm = 6399698.9018 / sqrtf(1 + eta);
    Mm = Nm / (1 + eta);
    Q = bb * Mm * (1 - ((2 * powf(l, 2) + powf(l, 2) * powf(sinf(Mm), 2)) / 24));
    P = l * Nm * cosf(Bm) * (1 + ((powf(bb, 2) - powf(l, 2) * powf(sinf(Bm), 2)) / 24));
    aa = l * sinf(Bm) * (1 + ((3 * powf(bb, 2) + 2 * powf(l, 2) - 2 * powf(l, 2) * powf(sinf(Bm), 2)) / 24));
    Am = atan2f(P, Q);
    
    return AngleReduction0_360_andr(180 / 3.14159265 * (Am - aa / 2));
}

float Plane::DistanceBetween2Points(float lat1_andr, float lon1_andr, float lat2_andr, float lon2_andr)
{
    float bb, l, Bm, eta, Nm, Mm, Q, P;
    bb = deg2rad_andr(lat2_andr) - deg2rad_andr(lat1_andr);
    l = deg2rad_andr(lon2_andr) - deg2rad_andr(lon1_andr);
    Bm = (deg2rad_andr(lat1_andr) + deg2rad_andr(lat2_andr)) / 2;
    eta = 0.0067385254 * powf(cosf(Bm), 2);
    Nm = 6399698.9018 / sqrtf(1 + eta);
    Mm = Nm / (1 + eta);
    Q = bb * Mm * (1 - ((2 * powf(l, 2) + powf(l, 2) * powf(sinf(Mm), 2)) / 24));
    P = l * Nm * cosf(Bm) * (1 + ((powf(bb, 2) - powf(l, 2) * powf(sinf(Bm), 2)) / 24));
    

    return sqrtf(powf(Q, 2) + powf(P, 2));
}



// Angle error to -180/180
float Plane::AngleErrTo180(float angle)
{
    if (angle < -180)
        return angle = angle + 360;
    else if (angle >= 180)
        return angle = angle - 360;
    else
        return angle;
 }