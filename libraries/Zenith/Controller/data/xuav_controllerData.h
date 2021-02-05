/**
 * @brief xuav gain header
 * @author Autogenerated by HeaderGen
 * @date 13-Jan-2021 18:24:15
 * @hash 3230221f8df12130a9ed587e2e2c493e
 */

namespace xuav_controllerData {
	static struct {
		const float enable = 1.000000;
		const float maxCmdDeg = 30.000000;
		const float maxElevDeg = 25.000000;
		const char *stateNames[3] = {"thetaErrDeg", "thetaErrInt", "<qDeg>"};
		const float Ktas[2] = {12.000000, 20.000000};
		const float K[6] = {0.268900, 0.268900, 0.088200, 0.088200, -0.120500, -0.120500};
	} pitch;

	static struct {
		const float enable = 1.000000;
		const float maxCmdDeg = 30.000000;
		const float maxAilDeg = 25.000000;
		const float maxRudDeg = 25.000000;
		const char *stateNames[4] = {"phiErrDeg", "phiErrInt", "<pDeg>", "rDegHP"};
		const float Ktas[2] = {12.000000, 20.000000};
		const float K[16] = {0.837900, 0.837900, 0.074100, 0.074100, -0.236700, -0.236700, -0.019400, -0.019400, 0.000000, 0.000000, 0.000000, 0.000000, 0.002700, 0.002700, -0.074500, -0.074500};
	} rollYaw;

	static struct {
		const float enable = 1.000000;
		const float maxAltErr = 5.000000;
		const float maxTasErr = 5.000000;
		const char *stateNames[7] = {"hErr", "hErrInt", "tasErr", "tasErrInt", "hDotErr", "<thetaDeg>", "<qDeg>"};
		const float Ktas[2] = {12.000000, 20.000000};
		const float K[28] = {0.293900, 0.293900, 0.000000, 0.000000, 0.372600, 0.372600, 0.025700, 0.025700, 0.092700, 0.092700, 0.040500, 0.040500, -0.000000, -0.000000, 4.878900, 4.878900, 0.000000, 0.000000, -1.062400, -1.062400, -0.185400, -0.185400, 5.136800, 5.136800, -0.850900, -0.850900, 0.000000, 0.000000};
	} spdAlt;

	static struct {
		const float enable = 0.000000;
		const float L1 = 80.000000;
		const float legacyDamping = 0.850000;
		const float legacyPeriodInv = 0.040000;
		const float legacyI = 0.000000;
	} xTrack;

	static struct {
		const float tas[6] = {10.000000, 12.000000, 14.000000, 16.000000, 18.000000, 20.000000};
		const float ux[6] = {9.982900, 11.999150, 13.997930, 15.989410, 17.977820, 19.964990};
		const float uy[6] = {-0.000050, -0.000000, -0.000010, -0.000070, -0.000200, -0.000380};
		const float uz[6] = {0.584500, 0.143140, -0.240650, -0.582060, -0.893400, -1.182890};
		const float phi[6] = {-0.000080, -0.000020, 0.000040, 0.000130, 0.000220, 0.000320};
		const float theta[6] = {0.058480, 0.011930, -0.017190, -0.036390, -0.049650, -0.059180};
		const float psi[6] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
		const float thrLeft[6] = {0.142520, 0.129640, 0.156670, 0.173340, 0.214750, 0.246360};
		const float thrRight[6] = {0.142520, 0.129640, 0.156670, 0.173340, 0.214750, 0.246360};
		const float ailDeg[6] = {-0.004490, -0.000670, 0.001600, 0.002810, 0.003890, 0.004580};
		const float elevDeg[6] = {3.915720, 3.000950, 2.500960, 2.194060, 1.995460, 1.857540};
		const float rudDeg[6] = {-0.035980, -0.006530, 0.009730, 0.021850, 0.030240, 0.034600};
	} trim;

	static struct {
		const float tas[2] = {12.000000, 20.000000};
		const float max[5] = {1.000000, 1.000000, 25.000000, 15.000000, 15.000000};
		const float table[60] = {18.790400, 18.790400, -0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 18.786400, 18.786400, -0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.018800, 0.018800, -0.000000, -0.000000, 0.000000, 0.000000, 57.292400, 57.292400, 0.000000, 0.000000, 49.956500, 49.956500, 100.025200, 100.025200, -0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 85.711400, 85.711400, 0.000000, 0.000000, -0.003500, -0.003500, 0.000000, 0.000000, -0.000000, -0.000000, 4.253600, 4.253600, 0.000000, 0.000000, 566.468300, 566.468300};
	} alloc;

};