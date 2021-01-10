/**
 * @brief xuav gain header
 * @author Autogenerated by HeaderGen
 * @date 04-Jan-2021 14:37:59
 * @hash cc096bf06200381dbd2bec8894340bd5
 */

namespace xuav_controllerData {
	static struct {
		const float enable = 1.000000;
		const float maxCmdDeg = 30.000000;
		const float maxElevDeg = 25.000000;
		const char *stateNames[3] = {"thetaErrDeg", "thetaErrInt", "<qDeg>"};
		const float Ktas[2] = {12.000000, 20.000000};
		const float K[6] = {0.237700, 0.237700, 0.081600, 0.081600, -0.105100, -0.105100};
	} pitch;

	static struct {
		const float enable = 1.000000;
		const float maxCmdDeg = 30.000000;
		const float maxAilDeg = 25.000000;
		const float maxRudDeg = 25.000000;
		const char *stateNames[4] = {"phiErrDeg", "phiErrInt", "<pDeg>", "rDegHP"};
		const float Ktas[2] = {12.000000, 20.000000};
		const float K[16] = {0.150700, 0.150700, 0.011700, 0.011700, -0.042900, -0.042900, -0.000200, -0.000200, 0.000000, 0.000000, 0.000000, 0.000000, 0.000400, 0.000400, -0.023500, -0.023500};
	} rollYaw;

	static struct {
		const float enable = 1.000000;
		const float maxAltErr = 5.000000;
		const float maxTasErr = 5.000000;
		const char *stateNames[7] = {"hErr", "hErrInt", "tasErr", "tasErrInt", "hDotErr", "<thetaDeg>", "<qDeg>"};
		const float Ktas[2] = {12.000000, 20.000000};
		const float K[28] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
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
		const float uy[6] = {-0.000050, -0.000000, -0.000010, -0.000070, -0.000190, -0.000360};
		const float uz[6] = {0.584500, 0.143130, -0.240640, -0.582060, -0.893400, -1.182890};
		const float phi[6] = {-0.000090, -0.000020, 0.000040, 0.000120, 0.000210, 0.000300};
		const float theta[6] = {0.058480, 0.011930, -0.017190, -0.036390, -0.049650, -0.059180};
		const float psi[6] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
		const float thr[6] = {0.142520, 0.129640, 0.156670, 0.173340, 0.214760, 0.246360};
		const float thrDiff[6] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
		const float ailDeg[6] = {-0.004730, -0.000920, 0.001170, 0.002490, 0.003390, 0.003970};
		const float elevDeg[6] = {3.915700, 3.000950, 2.500960, 2.194050, 1.995470, 1.857560};
		const float rudDeg[6] = {-0.037340, -0.007620, 0.009270, 0.019800, 0.027210, 0.031690};
	} trim;

	static struct {
		const float tas[2] = {12.000000, 20.000000};
		const float max[5] = {1.000000, 1.000000, 25.000000, 15.000000, 15.000000};
		const float table[60] = {18.788400, 18.788400, -0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -0.000000, -0.000000, 0.000000, 0.000000, 57.292400, 57.292400, 0.000000, 0.000000, 49.956500, 49.956500, 100.019700, 100.019700, -0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 85.711400, 85.711400, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 4.253600, 4.253600, 0.000000, 0.000000, 566.468300, 566.468300};
	} alloc;

};
