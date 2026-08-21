#ifndef KINEMATIC_SOLVER_H
#define KINEMATIC_SOLVER_H

template <int axes>
struct JointAngles
{
    float angles[axes];
    bool isValid = false;
};

struct Position
{
    float x = 0;
    float y = 0;
    float z = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    String toStr()
    {
        String result = "PX" + String(x, 3) + 
                        " PY" + String(y, 3) +
                        " PZ" + String(z, 3) +
                        " OR" + String(roll, 3) +
                        " OP" + String(pitch, 3) +
                        " OW" + String(yaw, 3);
        return result;   
    }
};

template <int axes>
class KinematicSolver
{
public:
    virtual JointAngles<axes> ik(Position targetPosition) = 0;
    virtual Position fk(JointAngles<axes> jointAngles) = 0;

private:
    int axesNum = axes;
};

#endif