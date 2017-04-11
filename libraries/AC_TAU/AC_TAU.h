#ifndef _AC_TAU_H
#define _AC_TAU_H
// Tau Object class
// Computes all the required things

#include <AP_Math.h>
#include <DataFlash.h>

class AC_TAU {
public:

	// Constructor for TAU
	AC_TAU(float fin_time, float k_const_val1, float sat_tau, float sat_err);

	// Set position, velocity, and update time
	void set_pos_vel_time(float position, float velocity, float time_in);
	void set_time_now(float v) 		{ _time_now = v; }

	// Update tau meas, tau ref, tau err, time now
	void update_tau();

	// Operator for easy changing
	void operator() (float fin_time, float k_const_val);
	void operator() (float fin_time, float k_const_val, float fin_pos);

	// To change the min allowed position and min allowed velocity in the TAU Measured calculation
	void set_minpos_minvel(float min_pos, float min_vel);

	// Update reference position, velocity and acceleration
	void update_reference();
	float get_tau_position() 		{ return _tau_position; }
	float get_tau_velocity()		{ return _tau_velocity; }
	float get_tau_acceleration()	{ return _tau_acceleration; }

	// get accessors
	// Should only be called after update_tau()
	float kendoul() 		{ return _kendoul; }
	float hybrid() 			{ return _hybrid; }
	float error_switch() 	{ return _error_switch; }
	float final_time() 		{ return _final_time; }
	float meas() 			{ return _measured_tau; }
	float ref() 			{ return _reference_tau; }
	float switch_time() 	{ return _switch_time; }
	float get_time_now()    { return _time_now; }

	// set accessors
	void initial_position(float v)	{ _initial_position = v; }

	// output for logging purposes
	const       DataFlash_Class::Tau_info get_tau_info(void) const { return _tau_info; }

protected:

	// Protected functions
	void ref_tau();
	void meas_tau();
	void error_tau();
	void error_switch_tau();
	void initialize_tau();

	// Parameters
	float		_epsilon;
	float 		_k_const;
	float 		_sat_tau;
	float		_sat_err;
	float 		_switch_time;
	float 		_min_allowed_pos;
	float		_min_allowed_vel;

	// Variables
	float 		_position;
	float 		_velocity;
	float 		_measured_tau;
	float 		_reference_tau;

	float 		_kendoul;
	float 		_hybrid;
	float		_error_switch;

	float 		_final_time;
	float		_final_position; // only used for tauposland and tauvelland (to feed correct position and velocity set points)
	float 		_time_now;

	// Reference Tau Data
	float 		_tau_position;
	float 		_tau_velocity;
	float 		_tau_acceleration;
	float 		_initial_position;

	// Logging purposes
	DataFlash_Class::Tau_info        _tau_info;

};

#endif