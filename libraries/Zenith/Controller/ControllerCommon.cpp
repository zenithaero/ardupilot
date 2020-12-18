/**
 * @brief LinearController
 * @date Created August 08, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "ControllerCommon.h"

template<size_t n, size_t n_tas, size_t n_k>
LinearController::LinearController(
	AP_AHRS &_ahrs,
	const char* (&stateNames)[n],
	const char* (&expectedNames)[n],
	const float (&_K_tas)[n_tas],
	const float (&_K_grid)[n_k]) : ahrs(_ahrs) {
	// Control state names
	for (size_t j = 0; j < n; j++)
		hard_assert(strcmp(stateNames[j], expectedNames[j]) == 0,
			"State mismatch: %s (expected %s)\n", stateNames[j], expectedNames[j]);

	// Create tas interpolation
	std::vector<float> tas_lookup(
		ControllerData::trim.tas,
		ControllerData::trim.tas + DIM(ControllerData::trim.tas)
	);
    std::vector<std::vector<float>> interp = { tas_lookup };
	tas_interp = new Interp<float>(interp);

	// Create K interpolation
	this->K_grid = std::vector<float>(_K_grid, _K_grid + n_k);
	std::vector<float> K_tas(_K_tas, _K_tas + n_tas);
    interp = {K_tas};
	K_interp = new Interp<float>(interp);
	
	// Initialize gain matrix
	size_t m = n_k / n / n_tas;
	hard_assert(m * n * n_tas == n_k,
		"The grid length (%lu) must equal m (%lu) * n (%lu) * n_tas (%lu)\n",
		n_k, m, n, n_tas);
	for (size_t i = 0; i < m; i++)
		K.push_back(std::vector<float>(n, 0));
}

LinearController::~LinearController() {
	delete(tas_interp);
	delete(K_interp);
}

void LinearController::update() {
	// Update timestep
	uint64_t t_now = AP_HAL::micros64();
	dt = (float)(t_now - t_prev) / 1e6f;
	t_prev = t_now;

	// Update tas filter
	float airspeed;
    if (ahrs.airspeed_estimate(airspeed)) {
		tas_filter.set_cutoff_frequency(1.f / 5); // T = 5s
		tas_filter.apply(airspeed, dt);
	}

	// Update controller gains
	tas_value[0] = tas_filter.get();
	std::vector<float> vec = K_interp->get_vec(K_grid, tas_value);
	reshape(vec, K);
}