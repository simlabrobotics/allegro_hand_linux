/////////////////////////////////////////////////////////////////////////////////////////
// Program main

#include "can_communicator.h"


int main(int argc, TCHAR* argv[])
{
    PrintInstruction();

    memset(&vars, 0, sizeof(vars));
    memset(q, 0, sizeof(q));
    memset(q_des, 0, sizeof(q_des));
    memset(tau_des, 0, sizeof(tau_des));
    memset(cur_des, 0, sizeof(cur_des));
    curTime = 0.0;

    if (CreateBHandAlgorithm() && OpenCAN())
        MainLoop();

    CloseCAN();
    DestroyBHandAlgorithm();

    return 0;
}