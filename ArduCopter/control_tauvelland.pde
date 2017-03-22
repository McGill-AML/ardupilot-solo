#include <AC_PID.h> 
#include "AC_TAU.h"

/*
 Tau theory land using position as a feedback. This implementation is
 for vertical control only. 
 */

static uint32_t tauvelland_start_time;
// static float land_final_time; // final time for landing; should be a parameter
// static float time_now;        // current time for tau control   

// static bool land_pause;

// static float position_z;
// static float velocity_z;

// static float tau_control_input;

// /// PID Law
// // PID setup
// static float tau_p = 0.0;
// static float tau_i = 0.0;
// static float tau_d = 0.0;
// static float tau_imax = 25.0;
// static float tau_filter = 5.0;
// static float tau_dt = 0.0025;

// // declare pid block
// static AC_PID tau_pid_z(tau_p, tau_i, tau_d, tau_imax, tau_filter, tau_dt); 

// // Tau object
// static AC_TAU tau_z(0.0, 0.0, 100.0, 1.0); 


// land_init - initialise land controller
static bool tauvelland_init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    land_with_gps = position_ok();
    if (land_with_gps) {
        // set target to stopping point
        Vector3f stopping_point;
        wp_nav.get_loiter_stopping_point_xy(stopping_point);
        wp_nav.init_loiter_target(stopping_point);
    }
  
    // initialize time for landing
    tauvelland_start_time = millis();     // millis is miliseconds, do proper conversion
    land_final_time = g.tau_time_final;

    land_pause = false;

    // Tau object
    tau_z(g.tau_time_final, g.tau_z_cons);      // Set final time, and k const
    tau_z.initial_position(inertial_nav.get_altitude()/100.0 - 1.0);  // Set the initial position

    // Initialize PID to the correct values
    tau_pid_z(g.tau_z_pid_p, g.tau_z_pid_i, g.tau_z_pid_d, tau_imax, tau_filter, tau_dt);
    tau_pid_z.reset_filter();


    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
static void tauvelland_run()
{
    if (land_with_gps) {
        tauvelland_gps_run();
    }else{
        land_nogps_run();
    }
}

// land_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
static void tauvelland_gps_run()
{
    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || ap.land_complete ) {
        wp_nav.init_loiter_target();

        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
        return;
    }

    // pause before beginning land descent
    if(land_pause && millis()-tauvelland_start_time >= LAND_WITH_DELAY_MS) {
        land_pause = false;    pos_control.set_velocity_control(tau_z.get_tau_velocity());
    // pos_control.set_alt_target_from_climb_rate(100.0*tau_z.get_tau_velocity(), G_Dt, true);
    // pos_control.update_z_controller();

    }
    
    land_run_horizontal_control();
    tau_vel_land_run_vertical_control(land_pause);
}

// Vertical controller for tau landing called from tauvelland_run()
//      also checks if we have landed.
static void tau_vel_land_run_vertical_control(bool pause_descent)
{
    /// TAU CONTROL SECTION 
 
    // get current time    
    time_now = millis()*0.001f - tauvelland_start_time*0.001f; // (s), getting converted to a float 
    tau_z.set_time_now(time_now);
    tau_z.update_reference();    

    // get position z
    position_z = inertial_nav.get_altitude()/100.0 - 0.05; // (m)
    float desired_position_z = tau_z.get_tau_position();   // (m)

    // get velocity
    // at time 0.0 s hard code velocity to 0.0 m/s (this may need to be changed)
    if (time_now == 0.0) {
        velocity_z = 0.0;
    } else {
        float velocity = inertial_nav.get_velocity_z(); // this is a function call to the ahrs (estimator) (AP_inertialNav) (cm/s)
        velocity_z = velocity/100.0; // (m/s)
    }

    // Tau object
    tau_z.set_pos_vel_time(position_z, velocity_z, time_now);
    tau_z.update_tau();

    // Set desired velocity and call function
    pos_control.set_velocity_control(tau_z.get_tau_velocity());

    // Logging the data
    tau_z_info = tau_z.get_tau_info();


    /// PRINT TO SCREEN:
    hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f \n", position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref());
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, total_out: %3.3f, hov: %3.3f, cont_inp: %3.3f \n",position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref(), error, tau_thr_out, pos_control.get_throttle_hover(), -tau_control_updated/10.0f);
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, pid: %3.3f, ken: %3.3f, hyb: %3.3f, hov: %3.3f, cont_inp: %3.3f ",position_z, velocity_z, time_now, tau_meas_z, tau_ref_z, tau_err_z.error_switch, tau_thr_out, tau_err_z.kendoul, tau_err_z.hybrid, thr_hover, tau_control_input);
    // hal.console->printf("get_p: %3.3f, get_i: %3.3f, get_pid: %3.3f, thr_in: %3.3f, thr_upd: %3.3f \n", tau_pid_z.get_p(), tau_pid_z.get_i(), tau_pid_z.get_pid(), tau_control_input, tau_control_updated);
    // hal.console->printf("get_p: %3.3f, get_i: %3.3f, get_pid: %3.3f \n", tau_pid_z.get_p(), tau_pid_z.get_i(), tau_pid_z.get_pid());
    // hal.console->printf("%3.3f %3.3f\n", myval, myval2); //(millis()-tauvelland_start_time)/1000.0);

}

