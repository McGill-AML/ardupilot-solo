#include <AC_PID.h> 
#include "AC_TAU.h"

/*
 Tau theory land using position as a feedback. This implementation is
 for vertical control only. 
 */

static uint32_t tauposland_start_time;
static float land_final_time; // final time for landing; should be a parameter

// land_init - initialise land controller
static bool tauposland_init(bool ignore_checks)
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
    tauposland_start_time = millis();     // millis is miliseconds, do proper conversion
    land_final_time = g.tau_time_final;

    land_pause = false;

    // Tau object
    tau_z(g.tau_time_final, g.tau_z_cons, g.tau_target_z);      // Set final time, and k const
    tau_z.initial_position(inertial_nav.get_altitude()/100.0); // - g.tau_target_z);  // Set the initial position

    // Initialize PID to the correct values
    tau_pid_z(g.tau_z_pid_p, g.tau_z_pid_i, g.tau_z_pid_d, tau_imax, tau_filter, tau_dt);
    tau_pid_z.reset_filter();

    if (g.tau_z_cons == 0.0) {
        set_mode(STOP);
    }


    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
static void tauposland_run()
{
    if (land_with_gps) {
        tauposland_gps_run();
    }else{
        land_nogps_run();
    }
}

// land_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
static void tauposland_gps_run()
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
    if(land_pause && millis()-tauposland_start_time >= LAND_WITH_DELAY_MS) {
        land_pause = false;
    }
    
    tauland_run_horizontal_control();
    tauposland_run_vertical_control(land_pause);
}

// Vertical controller for tau landing called from tauposland_run()
//      also checks if we have landed.
static void tauposland_run_vertical_control(bool pause_descent)
{
    /// TAU CONTROL SECTION 
 
    // get current time    
    time_now = millis()*0.001f - tauposland_start_time*0.001f; // (s), getting converted to a float 
    tau_z.set_time_now(time_now);
    tau_z.update_reference();    

    // get position z
    position_z = inertial_nav.get_altitude()/100.0; // (m)
    float desired_position_z = tau_z.get_tau_position(); // + g.tau_target_z;   // (m) --> ADD 1.0 to land 1.0 above the ground because desired tau position already knows we are travelling 1m less

    // get velocity
    // at time 0.0 s hard code velocity to 0.0 m/s (this may need to be changed)
    if (time_now <= 0.01) {
        velocity_z = 0.0;
    } else {
        float velocity = inertial_nav.get_velocity_z(); // this is a function call to the ahrs (estimator) (AP_inertialNav) (cm/s)
        velocity_z = velocity/100.0; // (m/s)
    }

    // Tau object because we want to update tau meas and tau ref, these are not being updated with update_reference()
    tau_z.set_pos_vel_time(position_z, velocity_z, time_now);
    tau_z.update_tau();

    // Set desired position and call function
    pos_control.set_alt_target(desired_position_z*100.0);
    pos_control.update_z_controller();

    // To return control back to me 
    if (time_now >= tau_z.final_time()+0.5) {   
        if (count_landed > 5) {
            set_mode(STOP);
        }
        count_landed++;
    }

    // Logging the data
    tau_z_info = tau_z.get_tau_info();

    /// PRINT TO SCREEN:
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f \n", position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref());
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, total_out: %3.3f, hov: %3.3f, cont_inp: %3.3f \n",position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref(), error, tau_thr_out, pos_control.get_throttle_hover(), -tau_control_updated/10.0f);
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, pid: %3.3f, ken: %3.3f, hyb: %3.3f, hov: %3.3f, cont_inp: %3.3f ",position_z, velocity_z, time_now, tau_meas_z, tau_ref_z, tau_err_z.error_switch, tau_thr_out, tau_err_z.kendoul, tau_err_z.hybrid, thr_hover, tau_control_input);
    // hal.console->printf("get_p: %3.3f, get_i: %3.3f, get_pid: %3.3f, thr_in: %3.3f, thr_upd: %3.3f \n", tau_pid_z.get_p(), tau_pid_z.get_i(), tau_pid_z.get_pid(), tau_control_input, tau_control_updated);
    // hal.console->printf("get_p: %3.3f, get_i: %3.3f, get_pid: %3.3f \n", tau_pid_z.get_p(), tau_pid_z.get_i(), tau_pid_z.get_pid());
    // hal.console->printf("%3.3f %3.3f\n", myval, myval2); //(millis()-tauposland_start_time)/1000.0);
}

// horizontal controller for tau landing called from tauposland_run() using position setpoints
//      also checks if we have landed.
/*static void tauposland_run_horizontal_control()
{
    // get current time    
    time_now = millis()*0.001f - tauland_start_time*0.001f; // (s), getting converted to a float 

    // Get Position
    position_x = inertial_nav.get_position().x/100.0;
    position_y = inertial_nav.get_position().y/100.0;

    // Get Velocity 
    // at time <0.05s hard code velocity to 0.0 m/s (this may need to be changed)
    float velocityx = inertial_nav.get_velocity().x; // this is a function call to the ahrs (estimator) (AP_inertialNav) (cm/s)
    float velocityy = inertial_nav.get_velocity().y; // this is a function call to the ahrs (estimator) (AP_inertialNav) (cm/s)
    
    if (time_now <= 0.5) {
        velocity_x = 0.0;
        velocity_y = 0.0;
    } else if (time_now < 1.0) {
        float epsilon = (time_now - 0.5)*2.0;
        velocity_x = epsilon*velocityx/100.0; // (m/s)
        velocity_y = epsilon*velocityy/100.0; // (m/s)
    } else {
        velocity_x = velocityx/100.0; // (m/s)
        velocity_y = velocityy/100.0; // (m/s)
    }

    // Set data inside tau block
    // tau_x.set_pos_vel_time(pos_t_x, vel_t_x, time_now);
    tau_x.set_pos_vel_time(position_x, velocity_x, time_now);
    tau_x.update_tau();

    // tau_y.set_pos_vel_time(pos_t_y, vel_t_y, time_now);
    tau_y.set_pos_vel_time(position_y, velocity_y, time_now);
    tau_y.update_tau();

    // Set tau reference time 
    tau_x.set_time_now(time_now);
    tau_x.update_reference();

    tau_y.set_time_now(time_now);
    tau_y.update_reference();      

    // get position x and y 
    float desired_position_x = tau_x.get_tau_position(); // + g.tau_target_z;   // (m) --> ADD 1.0 to land 1.0 above the ground because desired tau position already knows we are travelling 1m less
    float desired_position_y = tau_y.get_tau_position();

    // Set desired position and call function
    pos_control.set_xy_target(desired_position_x*100.0, desired_position_y*100.0);
    pos_control.set_target_to_stopping_point_xy();
    pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY, 1.0f);

    // Log Data
    tau_x_info = tau_x.get_tau_info();
    tau_y_info = tau_y.get_tau_info();

    /// PRINT TO SCREEN:
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f \n", position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref());
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, total_out: %3.3f, hov: %3.3f, cont_inp: %3.3f \n",position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref(), error, tau_thr_out, pos_control.get_throttle_hover(), -tau_control_updated/10.0f);
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, pid: %3.3f, ken: %3.3f, hyb: %3.3f, hov: %3.3f, cont_inp: %3.3f ",position_z, velocity_z, time_now, tau_meas_z, tau_ref_z, tau_err_z.error_switch, tau_thr_out, tau_err_z.kendoul, tau_err_z.hybrid, thr_hover, tau_control_input);
    // hal.console->printf("get_p: %3.3f, get_i: %3.3f, get_pid: %3.3f, thr_in: %3.3f, thr_upd: %3.3f \n", tau_pid_z.get_p(), tau_pid_z.get_i(), tau_pid_z.get_pid(), tau_control_input, tau_control_updated);
    // hal.console->printf("get_p: %3.3f, get_i: %3.3f, get_pid: %3.3f \n", tau_pid_z.get_p(), tau_pid_z.get_i(), tau_pid_z.get_pid());
    // hal.console->printf("%3.3f %3.3f\n", myval, myval2); //(millis()-tauposland_start_time)/1000.0);
}*/