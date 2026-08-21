#ifndef SOLVER_4_AXES_ALWAYS_VERTICAL_H
#define SOLVER_4_AXES_ALWAYS_VERTICAL_H

#include "KinematicSolver.h"

#define _USE_MATH_DEFINES
#include <math.h>

template <int axes>
class Solver4AxesAlwaysVertical : public KinematicSolver<axes>
{
public:
    Position fk(JointAngles<axes> jointAngles) override
    {
        float Tu = L1 * sin(jointAngles.angles[1]) + L2 * sin(jointAngles.angles[1] + jointAngles.angles[2]);
        float Tv = L1 * cos(jointAngles.angles[1]) + L2 * cos(jointAngles.angles[1] + jointAngles.angles[2]) - L3;
        float Tx = Tu * sin(jointAngles.angles[0]);
        float Ty = Tu * cos(jointAngles.angles[0]);

        Position result;
        result.x = Tx + dP1P2x;
        result.y = Ty + dP1P2y;
        result.z = Tv + dP1P2z;

        return result;
    }

    JointAngles<axes> ik(Position targetPosition) override
    {
        JointAngles<axes> angles;
        float Tx = targetPosition.x - dP1P2x;
        float Ty = targetPosition.y - dP1P2y;
        float Tu = sqrt(Tx * Tx + Ty * Ty);

        float Tv = targetPosition.z - dP1P2z;
        float P4v = L3 + Tv;

        float D = sqrt(Tu * Tu + P4v * P4v);
        float gamma1 = atan2(P4v, Tu);

        float gamma2_arg = (L1 * L1 + D * D - L2 * L2) / (2 * L1 * D);
        if (gamma2_arg < -1 || 1 < gamma2_arg)
        {
            angles.isValid = false;
            return angles;
        }
        float gamma2 = acos(gamma2_arg);

        float gamma3_arg = (L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2);
        if (gamma3_arg < -1 || 1 < gamma3_arg)
        {
            angles.isValid = false;
            return angles;
        }
        float gamma3 = acos(gamma3_arg);


        angles.angles[0] = atan2(Tx, Ty) * (180.0 / M_PI);
        angles.angles[1] = (M_PI_2 - gamma1 - gamma2) * (180.0 / M_PI);
        angles.angles[2] = (M_PI - gamma3) * (180.0 / M_PI);
        angles.angles[3] = 180 - angles.angles[1] - angles.angles[2];
        angles.isValid = true;

        return angles;
    }

private:
    float L1 = 374;
    float L2 = 365.5;
    float L3 = 105;
    float dP1P2x = 0;
    float dP1P2y = 49;
    float dP1P2z = 286;
};

#endif