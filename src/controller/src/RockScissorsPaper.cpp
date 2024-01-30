
#include "controller/rDeviceAllegroHandCANDef.h"
#include <BHand/BHand.h>

// ROCK-SCISSORS-PAPER(LEFT HAND)
//static double rock[] = {
//	0.1194, 1.2068, 1.0, 1.4042,
//	0.0093, 1.2481, 1.4073, 0.8163,
//	-0.1116, 1.2712, 1.3881, 1.0122,
//	0.6017, 0.2976, 0.9034, 0.7929};
//static double paper[] = {
//	0.1220, 0.4, 0.6, -0.0769,
//	-0.0312, 0.4, 0.6, -0.0,
//	-0.1767, 0.4, 0.6, -0.0528,
//	0.5284, 0.3693, 0.8977, 0.4863};
//static double scissors[] = {
//	-0.0885, 0.4, 0.6, -0.0704,
//	-0.0312, 0.4, 0.6, -0.0,
//	-0.1019, 1.2375, 1.1346,
//	1.0244, 1.0, 0.6331, 1.3509, 1.0};
// ROCK-SCISSORS-PAPER(RIGHT HAND)
static double rock[] = {
	-0.1194, 1.2068, 1.0, 1.4042,
	-0.0093, 1.2481, 1.4073, 0.8163,
	0.1116, 1.2712, 1.3881, 1.0122,
	0.6017, 0.2976, 0.9034, 0.7929};

static double paper[] = {
	-0.1220, 0.4, 0.6, -0.0769,
	0.0312, 0.4, 0.6, -0.0,
	0.1767, 0.4, 0.6, -0.0528,
	0.5284, 0.3693, 0.8977, 0.4863};

static double scissors[] = {
	0.0885, 0.4, 0.6, -0.0704,
	0.0312, 0.4, 0.6, -0.0,
	0.1019, 1.2375, 1.1346,
	1.0244, 1.0, 0.6331, 1.3509, 1.0};


static double cust1[] = {
	0.0, 1.6, 0.426, 0.0,
	0.109, 0.0, 0.0, 0.0,
	0.048, 0.0, 0.0, 0.072,
	1.396, 0.018, 0.0, 1.089};

static double cust2[] = { 
	-0.2124, -0.177, 0.148, 1.312,
	0.012, 0.323, 1.079, 1.335, 
	-0.144, 0.943, 0.335, 1.458, 
	0.323, 0.182, 0.702, 0.297
};

static double cust3[] = {
	-0.389, 1.250, 1.088, -0.20,
	0.011, 0.262,-0.036, 1.278, 
	0.210, 0.191, 0.875, 0.122, 
	0.405, 0.798, 0.863, 0.202
};


extern BHand* pBHand;
extern double q_des[MAX_DOF];

static void SetGainsRSP()
{
	// This function should be called after the function SetMotionType() is called.
	// Once SetMotionType() function is called, all gains are reset using the default values.
	if (!pBHand) return;
	double kp[] = {
		500, 800, 900, 500,
		500, 800, 900, 500,
		500, 800, 900, 500,
		1000, 700, 600, 600
	};
	double kd[] = {
		25, 50, 55, 40,
		25, 50, 55, 40,
		25, 50, 55, 40,
		50, 50, 50, 40
	};
	pBHand->SetGainsEx(kp, kd);
}

void MotionRock()
{
	for (int i=0; i<16; i++)
		q_des[i] = rock[i];
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGainsRSP();

}

void MotionScissors()
{
	for (int i=0; i<16; i++)
		q_des[i] = scissors[i];
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGainsRSP();
}

void MotionPaper()
{
	for (int i=0; i<16; i++)
		q_des[i] = paper[i];
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGainsRSP();
}

#include <random>

void random_pos()
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution<double> dis(-1.0, 1.0);

	double custom1[] = { 
		dis(gen), dis(gen), dis(gen), dis(gen),
		dis(gen), 0.4, 0.6, dis(gen),
		dis(gen), 0.4, 0.6, dis(gen),
		dis(gen), 0.4, 0.6, dis(gen)};

	for (int i=0; i<16; i++) {
		q_des[i] = custom1[i];
	}
	
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGainsRSP();
}

void custom1()
{
	for (int i=0; i<16; i++)
		q_des[i] = cust1[i];
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGainsRSP();
}

void custom2()
{
	for (int i=0; i<16; i++)
		q_des[i] = cust2[i];
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGainsRSP();
}

void custom3()
{
	for (int i=0; i<16; i++)
		q_des[i] = cust3[i];
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGainsRSP();
}

void reset() { 
	for (int i=0; i<16; i++)
		q_des[i] = 0.0;
	if (pBHand) pBHand->SetMotionType(eMotionType_JOINT_PD);
	SetGainsRSP();
}
