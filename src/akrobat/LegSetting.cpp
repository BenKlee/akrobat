#include <akrobat/LegSetting.h>

LegSetting::LegSetting() :
	rollOv(0.0f),
	rotOfCoxa(0.0f),
	bdConstX(0.0f),
	bdConstY(0.0f),
	bdConstZ(0.0f),
	jointInitA(0.0f),
	jointInitB(0.0f),
	jointInitC(0.0f),
	minCoxa(0.0f),
	minFemur(0.0f),
	minTibia(0.0f),
	maxCoxa(0.0f),
	maxFemur(0.0f),
	maxTibia(0.0f)
{
}

LegSetting::LegSetting(double rollOv, double rotOfCoxa, double bdConstX, double bdConstY, double bdConstZ, double jointInitA, double jointInitB, double jointInitC, double minCoxa, double minFemur, double minTibia, double maxCoxa, double maxFemur, double maxTibia) :
	rollOv(rollOv),
	rotOfCoxa(rotOfCoxa),
	bdConstX(bdConstX),
	bdConstY(bdConstY),
	bdConstZ(bdConstZ),
	jointInitA(jointInitA),
	jointInitB(jointInitB),
	jointInitC(jointInitC),
	minCoxa(minCoxa),
	minFemur(minFemur),
	minTibia(minTibia),
	maxCoxa(maxCoxa),
	maxFemur(maxFemur),
	maxTibia(maxTibia)
{
}