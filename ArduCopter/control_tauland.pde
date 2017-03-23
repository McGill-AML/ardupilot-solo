/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AC_PID.h> 
#include "AC_TAU.h"


/*
 control_tauland.pde is based on tau theory to land. This implementation is
 for vertical control only. I will then focus on getting all three dimensions
 working properly.

 */

// Initializations
static uint32_t tauland_start_time;
static float tauland_final_time; // final time for landing; should be a parameter
static float time_now;        // current time for tau control   

static bool tauland_pause;

static float position_z;
static float velocity_z;
static float position_x;
static float velocity_x;
static float position_y;
static float velocity_y;

static float tau_control_input;
static int count_landed = 0;

static float beta_approach; // approach angled from the vehicle perspective
static float alpha_0_approach; // initial approach angle using planar position
static float alpha_approach; // approach angle used 

// Saturations / Constrains
float sat_tau_meas = 100.0;
float sat_tau_err = 1.0;

/// PID Law
// PID setup
static float tau_p = 0.0;
static float tau_i = 0.0;
static float tau_d = 0.0;
static float tau_imax = 2.0;
static float tau_filter = 5.0;
static float tau_dt = 0.0025;

// declare pid block
// ****************************************************************************************************************
// * This should ideally be declared in parameters and be accesible globally. Further the parameters such as:     *
// * K_cons, PID, and final time should be changeable from QGC                                                    *
// ****************************************************************************************************************
static AC_PID tau_pid_z(tau_p, tau_i, tau_d, tau_imax, tau_filter, tau_dt); 
static AC_PID tau_pid_x(tau_p, tau_i, tau_d, tau_imax, tau_filter, tau_dt); 
static AC_PID tau_pid_y(tau_p, tau_i, tau_d, tau_imax, tau_filter, tau_dt); 

// Tau object
static AC_TAU tau_z(0.0, 0.0, sat_tau_meas, sat_tau_err); 
static AC_TAU tau_x(0.0, 0.0, sat_tau_meas, sat_tau_err); 
static AC_TAU tau_y(0.0, 0.0, sat_tau_meas, sat_tau_err); 


// land_init - initialise land controller
static bool tauland_init(bool ignore_checks)
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
    tauland_start_time = millis();     // millis is miliseconds, do proper conversion
    tauland_final_time = g.tau_time_final;

    tauland_pause = false;

    // Tau object
    tau_z(g.tau_time_final, g.tau_z_cons);
    tau_x(g.tau_time_final, g.tau_x_cons);
    tau_y(g.tau_time_final, g.tau_y_cons);

    // Initialize PID to the correct values
    tau_pid_z(g.tau_z_pid_p, g.tau_z_pid_i, g.tau_z_pid_d, tau_imax, tau_filter, tau_dt);
    tau_pid_z.reset_filter();

    // tau_pid_x(g.tau_xy_pid_p, g.tau_xy_pid_i, g.tau_xy_pid_d, tau_imax, tau_filter, tau_dt);
    // tau_pid_x.reset_filter();

    // tau_pid_y(g.tau_xy_pid_p, g.tau_xy_pid_i, g.tau_xy_pid_d, tau_imax, tau_filter, tau_dt);
    // tau_pid_y.reset_filter();

    tau_pid_x(14.0, 1.2, g.tau_xy_pid_d, tau_imax, tau_filter, tau_dt);
    tau_pid_x.reset_filter();

    tau_pid_y(13.0, 1.5, g.tau_xy_pid_d, tau_imax, tau_filter, tau_dt);
    tau_pid_y.reset_filter();

    // Approach angle relative to vector joining the initial and target
    beta_approach = 0.0; // rad
    alpha_0_approach = fast_atan2(inertial_nav.get_velocity().y, inertial_nav.get_velocity().x);
    alpha_approach = alpha_0_approach + beta_approach;

    return true;
}

// tauland_run - runs the land controller
// should be called at 100hz or more
static void tauland_run()
{
    if (land_with_gps) {
        tauland_gps_run();
    }else{
        // land_nogps_run();
        land_gps_run();
    }
}

// tauland_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
static void tauland_gps_run()
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
    if(tauland_pause && millis()-tauland_start_time >= LAND_WITH_DELAY_MS) {
        tauland_pause = false;
    }
    
    land_run_horizontal_control();
    // land_run_vertical_control(tauland_pause);
    
    // tau_land_run_horizontal_control();
    tau_land_run_vertical_control(tauland_pause);
}

// Vertical controller for tau landing called from myland_run()
//      also checks if we have landed.
static void tau_land_run_vertical_control(bool pause_descent)
{
    /// TAU CONTROL SECTION 
 
    // get current time    
    time_now = millis()*0.001f - tauland_start_time*0.001f; // (s), getting converted to a float 

    // get velocity
    // at time 0.0 s hard code velocity to 0.0 m/s (this may need to be changed)
    if (time_now <= 0.05) {
        velocity_z = 0.0;
    } else {
        float velocity = inertial_nav.get_velocity_z(); // this is a function call to the ahrs (estimator) (AP_inertialNav) (cm/s)
        velocity_z = velocity/100.0; // (m/s)
    }

    // get position z
    position_z = inertial_nav.get_altitude()/100.0 - g.tau_target_z; // (m) --> to land 1.0 m above the ground

    // Tau object
    tau_z.set_pos_vel_time(position_z, velocity_z, time_now);
    tau_z.update_tau();

    // PID SECTION
    // set Error
    float error = tau_z.error_switch();
    tau_pid_z.set_input_filter_d(error);
    
    // get pid error out. Note sometimes it is neceassy to reset the i flags? and sometimes you call them separately. 
    //      see some of the examples in the accel_to_throttle() function in the AC_PosControl library
    tau_control_input = tau_pid_z.get_pid();

    // Multiply by a constant and get the correct sign
    float tau_control_updated = -1.0*tau_control_input*10.0;

    /// SEND COMMANDS
    float tau_thr_out = tau_control_updated + pos_control.get_throttle_hover(); // in the range 0~1 (from AP_MotorsMulticopter.h)
    attitude_control.set_throttle_out(tau_thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);

    
    // check to see if we have landed
    // this assumes that we are on the ground even if we are not fully landed. i.e. we are going from 10m -> 5m
    if (position_z <= 0.1 and time_now >= tau_z.final_time()*0.99) {   
        if (count_landed > 5) {
            init_disarm_motors();
        }
        count_landed++;
    }

    // Logging the data
    tau_z_info = tau_z.get_tau_info();

    // Print to screen
    // float thr_hover = motors.get_throttle_hover();
    // float myval = tau_pid_z.kP();
    // float myval2 = tau_pid_z.kI();
    // float myval3 = tau_pid_z.kD();

    /// PRINT TO SCREEN:
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, thr_out: %3.3f, ken: %3.3f, hyb: %3.3f, hov: %3.3f, cont_inp: %3.3f \n", position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref(), error, tau_thr_out, tau_z.kendoul(), tau_z.hybrid(), motors.get_throttle_hover(), tau_control_updated);
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, total_out: %3.3f, hov: %3.3f, cont_inp: %3.3f \n",position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref(), error, tau_thr_out, pos_control.get_throttle_hover(), tau_control_updated);
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, pid: %3.3f, ken: %3.3f, hyb: %3.3f, hov: %3.3f, cont_inp: %3.3f ",position_z, velocity_z, time_now, tau_meas_z, tau_ref_z, tau_err_z.error_switch, tau_thr_out, tau_err_z.kendoul, tau_err_z.hybrid, thr_hover, tau_control_input);
    // hal.console->printf("get_p: %3.3f, get_i: %3.3f, get_pid: %3.3f, thr_in: %3.3f, thr_upd: %3.3f \n", tau_pid_z.get_p(), tau_pid_z.get_i(), tau_pid_z.get_pid(), tau_control_input, tau_control_updated);
    // hal.console->printf("get_p: %3.3f, get_i: %3.3f, get_pid: %3.3f \n", tau_pid_z.get_p(), tau_pid_z.get_i(), tau_pid_z.get_pid());
    // hal.console->printf("%3.3f %3.3f\n", myval, myval2); //(millis()-tauland_start_time)/1000.0);

}

// Horizontal controller for tau landing called from myland_run()
static void tau_land_run_horizontal_control()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
    
    // process pilot inputs
    if (!failsafe.radio) {
        if(rc_throttle_control_in_filter.get() > 700){
            // exit land
            if (position_ok()) {
                set_mode(LOITER);
            } else {
                set_mode(ALT_HOLD);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = g.rc_1.control_in;
            pitch_control = g.rc_2.control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    int32_t nav_roll  = wp_nav.get_roll();
    int32_t nav_pitch = wp_nav.get_pitch();

    //// TAU CONTROL STARTS HERE

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
    tau_x.set_pos_vel_time(position_x, velocity_x, time_now);
    tau_x.update_tau();

    tau_y.set_pos_vel_time(position_y, velocity_y, time_now);
    tau_y.update_tau();

    // Error calculation
    float error_x = tau_x.kendoul();
    float error_y = tau_y.kendoul();
    tau_pid_x.set_input_filter_d(error_x);
    tau_pid_y.set_input_filter_d(error_y);

    /// Nonlinear transform
    // Approach angle
    float psi = wp_nav.get_yaw()/100.0*DEG_TO_RAD;   // get_yaw() returned in centi-degrees

    float cos_ap = cos(alpha_approach - psi);
    float sin_ap = sin(alpha_approach - psi);
    // float cos_ap = cos(0);
    // float sin_ap = sin(0);

    float pitch = 1.0*(tau_pid_x.get_pid()*cos_ap - tau_pid_y.get_pid()*sin_ap);  // phi, may need a negative infront of it
    float roll  = 1.0*(tau_pid_y.get_pid()*cos_ap + tau_pid_x.get_pid()*sin_ap);  // theta.. THiS NEEDS TO BE NEGATIVE IF IM USING COS_AP = 1.0, SIN_AP = 0.0

    // constrain angles using tanh() and convert to centidegrees
    pitch = int(pitch*100.0); 
    roll = int(roll*100.0);

    //// TAU CONTROL ENDS HERE

 
    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw (roll, pitch, target_yaw_rate);

    // To make sure I get control back 
    if (time_now >= tauland_final_time+1.0) {
        set_mode(LAND);
    }

    float pitch_actual = pos_control.get_pitch();
    float roll_actual = pos_control.get_roll();

    // Print to screen
    // hal.console->printf("time: %3.3f, posx: %3.3f, velx: %3.3f, posy: %3.3f, vely: %3.3f, roll: %5.3f, pitch: %5.3f, tau_x: %3.3f, tau_y: %3.3f, tau_x_err: %3.3f, tau_y_err: %3.3f \n", time_now, position_x, velocity_x, position_y, velocity_y, roll, pitch, tau_x.meas(), tau_y.meas(), tau_x.error_switch(), tau_y.error_switch());
    hal.console->printf("time: %3.3f, posx: %3.3f, velx: %3.3f, pitch_des: %3.3f, pitch_actual: %5.3f, tau_x: %3.3f, tau_x_meas: %3.3f, err_x: %3.3f \n", time_now, position_x, velocity_x, pitch, pitch_actual, tau_x.meas(), tau_x.ref(), error_x);
    hal.console->printf("time: %3.3f, posy: %3.3f, vely: %3.3f, roll_des:  %3.3f, roll_actual:  %5.3f, tau_y: %3.3f, tau_y_meas: %3.3f, err_y: %3.3f \n", time_now, position_y, velocity_y, roll,  roll_actual,  tau_y.meas(), tau_y.ref(), error_y);
    hal.console->printf("cos_ap: %3.3f, sin_ap: %3.3f, alpha_approach: %3.3f, beta_approach: %3.3f, psi: %3.3f \n", cos_ap, sin_ap, alpha_approach, beta_approach, psi);
}

// Look at how input_euler_angle_roll_pitch_euler_rate_yaw_smooth() works
// this is using the internal working of the RTL, its mainly for error checking
/*static void tau_land_run_horizontal_control()
{   
    rtl_build_path();

    // rtl_return_start
    wp_nav.set_wp_destination(rtl_path.return_target);

    // run waypoint controller
    wp_nav.update_wpnav();

    //Stuff
    float pitch = wp_nav.get_pitch();
    // if (time_now < 0.5*tauland_final_time) {
    //     float pitch = -10.0*100.0;
    // } else {
    //     float pitch = 10.0*100.0;
    // }

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0.0, pitch, 0.0);
    
    // Print out
    float pitch_actual = pos_control.get_pitch();
    time_now = millis()*0.001 - tauland_start_time*0.001;
    position_x = inertial_nav.get_position().x/100.0;
    velocity_x = inertial_nav.get_velocity().x/100.0;
    float tau_x_meas = position_x/velocity_x;
    float tau_x_ref = 1.0;

    hal.console->printf("time: %3.3f, posx: %3.3f, velx: %3.3f, pitch_des: %3.3f, pitch_actual: %5.3f, tau_x: %3.3f, tau_x_meas, %3.3f \n", time_now, position_x, velocity_x, pitch, pitch_actual, tau_x_meas, tau_x_ref);
    // hal.console->printf("times: %3.3f, posx: %3.3f, velx: %3.3f, pitch_des: %3.3f, pitch_actual: %5.3f, tau_x: %3.3f \n", time_now, position_x, velocity_x, pitch, pitch_actual, tau_xx);
}*/

static void land_run_horizontal_control()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
    
    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }
    
    // process pilot inputs
    if (!failsafe.radio) {
        if(rc_throttle_control_in_filter.get() > 700){
            // exit land
            if (position_ok()) {
                set_mode(LOITER);
            } else {
                set_mode(ALT_HOLD);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = g.rc_1.control_in;
            pitch_control = g.rc_2.control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

  
    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    float nav_roll  = wp_nav.get_roll();
    float nav_pitch = wp_nav.get_pitch();

  
    // call attitude controller
    // AC_AttitudeControl::attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(nav_roll, nav_pitch, target_yaw_rate, get_smoothing_gain());
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(nav_roll, nav_pitch, target_yaw_rate);
}

