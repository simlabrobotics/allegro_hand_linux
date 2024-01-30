
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>
#include <pthread.h>
#include "canAPI.h"
#include "rDeviceAllegroHandCANDef.h"
#include "RockScissorsPaper.h"
#include <BHand/BHand.h>

#define PEAKCAN (1)
typedef char    TCHAR;
#define _T(X)   X
#define _tcsicmp(x, y)   strcmp(x, y)

/////////////////////////////////////////////////////////////////////////////////////////
// for CAN communication
extern const double delT;
extern int CAN_Ch;
extern bool ioThreadRun;
extern pthread_t        hThread;
extern int recvNum;
extern int sendNum;
extern double statTime;
extern AllegroHand_DeviceMemory_t vars;

extern double curTime;

/////////////////////////////////////////////////////////////////////////////////////////
// for BHand library
extern BHand* pBHand;
extern double q[MAX_DOF];
extern double q_des[MAX_DOF];
extern double tau_des[MAX_DOF];
extern double cur_des[MAX_DOF];

// USER HAND CONFIGURATION
extern const bool	RIGHT_HAND;
extern const int	HAND_VERSION;

extern const double tau_cov_const_v4; // 1200.0 for SAH040xxxxx

/////////////////////////////////////////////////////////////////////////////////////////
// functions declarations
char Getch();
void PrintInstruction();
void MainLoop();
bool OpenCAN();
void CloseCAN();
int GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorque();