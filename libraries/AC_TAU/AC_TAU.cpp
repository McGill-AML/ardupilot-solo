#include "AC_TAU.h"

// Constructor 
AC_TAU::AC_TAU(float fin_time, float k_const_val1, float sat_tau, float sat_err)
{
	// Set Initial values
	_position = 0.0;
	_velocity = 1.0;
	_final_time = fin_time;
	_time_now = 0.0;
	_k_const = k_const_val1;
	_sat_tau = sat_tau;
	_sat_err = sat_err;
	_switch_time = floorf(0.5*_final_time);
	_tau_info.tauref 	= 0.0;
	_tau_info.taumeas 	= 0.0;
	_tau_info.kendoul 	= 0.0;
	_tau_info.hybrid 	= 0.0;
	_tau_info.error 	= 0.0;
	_tau_info.timenow 	= 0.0;
}

void AC_TAU::operator() (float fin_time, float k_const_val)
{ 
	_k_const = k_const_val;
	_final_time = fin_time; 
	_switch_time = floorf(0.5*_final_time);

	// Set Initial values
	_position = 0.0;
	_velocity = 1.0;
	_time_now = 0.0;

	_tau_info.tauref 	= 0.0;
	_tau_info.taumeas 	= 0.0;
	_tau_info.kendoul 	= 0.0;
	_tau_info.hybrid 	= 0.0;
	_tau_info.error 	= 0.0;
	_tau_info.timenow 	= 0.0;
}



void AC_TAU::set_pos_vel_time(float position, float velocity, float time_now)
{
	_position = position;
	_velocity = velocity;
	_time_now = time_now; 	// update current time when this function is called
}

// Calculate the reference tau value given the tau parameters
//      outputs the reference tau
void AC_TAU::ref_tau()
{	
	// Definition for reference tau
	if (_time_now < 0.1) {
		_reference_tau = -99999.0;
	} else if(_time_now < _final_time) {
		_reference_tau = 0.5*_k_const*(_time_now - _final_time*_final_time/_time_now);
	} else {
		_reference_tau = 0.0;
	}

	// Saturate the reference tau
	_reference_tau = constrain_float(_reference_tau, -1.0*_sat_tau, _sat_tau);
}

// Calculate the measured tau value given the tau parameters, 
//      current position and velocity. 
//      outputs the measured tau value 
void AC_TAU::meas_tau()
{	
	// Definition for measured tau
	if (fabsf(_position) <= 0.02 ) {
		_measured_tau = 0.0;
	} else if (fabsf(_velocity) >= 0.05) {
		_measured_tau = _position/_velocity;
	} else {
		float time_mod = _time_now - 0.02;
		_measured_tau = 0.5*_k_const*(time_mod - _final_time*_final_time/time_mod);
	}

	// Saturate the measured tau
	_measured_tau = constrain_float(_measured_tau, -1.0*_sat_tau, _sat_tau);
}

// Calculate the error in tau value gven the tau parameters, 
//      current position and velocity, reference and measured tau. 
//      outputs the hybrid, kendoul, and mixed error value 
//      will be used as the input to the pid
void AC_TAU::error_tau()
{	
	// if tau measured is very small, the hybrid and kendoul error are 0
	if (fabsf(_measured_tau) <= 0.01) {
		_kendoul = 0.0;
		_hybrid = 0.0;
	
	// else compute the kendoul and hybrid error as per the definition  		
	} else {
		if (fabsf(_velocity) <= 0.01) {
			_velocity = -1.0*copysignf(1.0, _position)*0.01;
		}

		// Hybrid Definition
		if (_reference_tau < -1.0) {
			_hybrid = -1.0*copysignf(1.0, _velocity)*(_reference_tau - _measured_tau)/fabsf(_reference_tau);
		} else {
			_hybrid = -1.0*copysignf(1.0, _velocity)*(_reference_tau - _measured_tau)*fabsf(_reference_tau);
		}

		// Kendoul Definition
		_kendoul = -1.0*copysignf(1.0, _measured_tau)*-1.0*copysignf(1.0, _velocity)*(1 - _reference_tau/_measured_tau);
	}

	// Saturate both Kendoul and Hybrid
	_kendoul = constrain_float(_kendoul, -1.0*_sat_err, _sat_err);
	_hybrid = constrain_float(_hybrid, -1.0*_sat_err, _sat_err);
}

// Compute the switch between the two errors at 50% of final time
void AC_TAU::error_switch_tau()
{	
	// The epsilon is used for the convex error mixing
	_epsilon = _time_now - floorf(_time_now);

	if (_time_now < _switch_time) {
		_error_switch = _kendoul;

	} else if (_time_now < _switch_time + 1.0) {
		_error_switch = (1.0 - _epsilon)*_kendoul + (_epsilon)*_hybrid;

	} else {
		_error_switch = _hybrid;
	}
}

// Update tau ref, tau meas, error
void AC_TAU::update_tau()
{
	// 1) Call Reference Tau
	ref_tau();

	// 2) Calculate Measured Tau
	meas_tau();

	// 3) Calculate Kendoul and Hybrid Error
	error_tau();

	// 4) Smoothly switch the error at 0.5*_final_time
	error_switch_tau();

	// 5) update _tau_info for logging purposes
	_tau_info.tauref 	= _reference_tau;
	_tau_info.taumeas 	= _measured_tau;
	_tau_info.kendoul 	= _kendoul;
	_tau_info.hybrid 	= _hybrid;
	_tau_info.error 	= _error_switch;
	_tau_info.timenow 	= _time_now;
}


// Update tau reference position, velocity and acceleration
// Make sure initial_pos has been set. Also ensure that time_now has been 
// updated before calling this function.
void AC_TAU::update_reference()
{	
	if (_time_now < _final_time) {
		// Values that get re-used
		float const1 = _initial_position/pow(_final_time, 2.0/_k_const);
		float ftime_ctime = _final_time*_final_time - _time_now*_time_now;

		// Update pos, vel and acceleration
		_tau_position = const1*pow(ftime_ctime, 1.0/_k_const);
		_tau_velocity = -2.0*const1*_time_now/_k_const*pow(ftime_ctime, 1.0/_k_const - 1.0);
		_tau_acceleration = 2.0*const1/_k_const*((2.0 - _k_const)/_k_const*_time_now*_time_now - _final_time*_final_time)*pow(ftime_ctime, 1.0/_k_const - 2.0);

	} else {
		
		_tau_position = 0.0;
		_tau_velocity = 0.0;
		_tau_acceleration = 0.0;
	}
}