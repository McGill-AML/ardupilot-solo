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

static float position_z;
static float velocity_z;
static float position_x;
static float velocity_x;
static float position_y;
static float velocity_y;

static float tau_control_input;

static float target_z;
static float target_x;
static float target_y;

// Approach Angle
static float beta_approach; // approach angled from the vehicle perspective
static float alpha_0_approach; // initial approach angle using planar position
static float alpha_approach; // approach angle used 
static float cos_ap, sin_ap;

// Saturations / Constrains
static float sat_tau_meas = 100.0;
static float sat_tau_err = 1.0;

// PID setup
static float tau_p = 0.0;
static float tau_i = 0.0;
static float tau_d = 0.0;
static float tau_imax = 4.0;
static float tau_filter = 5.0;
static float tau_dt = 0.0025;
static float hov_thr_default;

// declare pid block
static AC_PID tau_pid_z(tau_p, tau_i, tau_d, tau_imax, tau_filter, tau_dt); 
static AC_PID tau_pid_x(tau_p, tau_i, tau_d, tau_imax, tau_filter, tau_dt); 
static AC_PID tau_pid_y(tau_p, tau_i, tau_d, tau_imax, tau_filter, tau_dt);
static AC_PID tau_pid_yaw(tau_p, tau_i, tau_d, tau_imax, tau_filter, tau_dt); 

// Tau object
static AC_TAU tau_z(0.0, 0.0, sat_tau_meas, sat_tau_err); 
static AC_TAU tau_x(0.0, 0.0, sat_tau_meas, sat_tau_err); 
static AC_TAU tau_y(0.0, 0.0, sat_tau_meas, sat_tau_err);
static AC_TAU tau_yaw(0.0, 0.0, sat_tau_meas, sat_tau_err);

// Landing Case
enum TauLandMode {
    TAU_VERTICAL_TAU_HORIZONTAL,
    TAU_VERTICAL_HOLD_HORIZONTAL,
    HOLD_VERTICAL_TAU_HORIZONTAL,
    TAU_VERTICAL_TAU_HORIZONTAL_TAU_YAW,
    HOLD_VERTICAL_TAU_HORIZONTAL_TAU_YAW,
} tau_land_mode;

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

    // Initialize Tau objects
    tau_z(g.tau_time_final, g.tau_z_cons);
    tau_x(g.tau_time_final, g.tau_x_cons);
    tau_y(g.tau_time_final, g.tau_y_cons);
    tau_yaw(g.tau_time_final, g.tau_psi_cons);

    // Set min vel to lower value than default for horizontal control because it works better
    tau_x.set_minpos_minvel(0.02, 0.02);
    tau_y.set_minpos_minvel(0.02, 0.02); 

    // Approach angle relative to vector joining the initial and target
    beta_approach = constrain_float(beta_approach, -89.0, 89.0);
    if (g.tau_approach_angle == 0.0) {
        // The following is done so that we dont have 0 tau reference at 0 degree angle and try to match that
        tau_y(g.tau_time_final, g.tau_x_cons);  // Set the same value for the K constant
        beta_approach = 45.0*PI/180.0;          // Set pseudo angle of 45 degrees to ensure equal gap closure

    } else {
        beta_approach = g.tau_approach_angle*PI/180.0; // rad       
    }

    alpha_0_approach = fast_atan2(inertial_nav.get_position().y, inertial_nav.get_position().x);
    alpha_approach = alpha_0_approach + beta_approach;

    // Store hover initial value and target values
    target_z = g.tau_target_z;
    target_x = g.tau_target_x;
    target_y = g.tau_target_y;
    hov_thr_default = pos_control.get_throttle_hover();

    // Gain Scheduling
    float gain_x = (inertial_nav.get_position().x/100.0 - target_x)/g.final_time;
    float gain_y = (inertial_nav.get_position().y/100.0 - target_y)/g.final_time;
    float gain_z = (current_loc.alt/100.0 - target_z)/g.final_time;

    // Initialize PID to the correct values
    tau_pid_z(g.tau_z_pid_p*gain_z, g.tau_z_pid_i*gain_z, g.tau_z_pid_d*gain_z, 15.0, tau_filter, tau_dt);
    tau_pid_z.reset_filter();

    tau_pid_x(g.tau_x_pid_p*gain_x, g.tau_x_pid_i*gain_x, g.tau_xy_pid_d*gain_x, tau_imax, tau_filter, tau_dt);
    tau_pid_x.reset_filter();

    tau_pid_y(g.tau_y_pid_p*gain_y, g.tau_y_pid_i*gain_y, g.tau_xy_pid_d*gain_y, tau_imax, tau_filter, tau_dt);
    tau_pid_y.reset_filter();

    tau_pid_yaw(g.tau_psi_pid_p, g.tau_psi_pid_i, g.tau_xy_pid_d, tau_imax, tau_filter, tau_dt);
    tau_pid_yaw.reset_filter();

    // Set the desired tau land mode
    if (g.tau_z_cons == 0.0) {
        if (g.tau_x_cons != 0.0 || g.tau_y_cons != 0.0) {
            tau_land_mode = HOLD_VERTICAL_TAU_HORIZONTAL;
        } else {
            set_mode(STOP); // if all the K constants are 0, switch in to STOP mode
        }

    } else {
        if (g.tau_x_cons != 0.0 || g.tau_y_cons != 0.0) {
            tau_land_mode = TAU_VERTICAL_TAU_HORIZONTAL;
        } else {
            tau_land_mode = TAU_VERTICAL_HOLD_HORIZONTAL;
        }
    }

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

    // Switch case to run the correct landing controllers
    switch(tau_land_mode) {
        case TAU_VERTICAL_HOLD_HORIZONTAL:
            tauland_run_hold_horizontal_control();
            tauland_run_vertical_control();
            break;

        case TAU_VERTICAL_TAU_HORIZONTAL:
            tauland_run_horizontal_control();
            tauland_run_vertical_control();
            break;

        case HOLD_VERTICAL_TAU_HORIZONTAL:
            tauland_run_horizontal_control();
            tauland_run_hold_vertical_control();
            break;

        default:
            set_mode(STOP);
            break;    
    }
    // Check to see if we have landed to switch out of autonomous land mode
    tauland_check_destination(); 
}

// Vertical controller for tau landing called from myland_run()
//      also checks if we have landed.
static void tauland_run_vertical_control()
{   
    ///////////////////////////
    /// TAU CONTROL SECTION ///
    ///////////////////////////

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

    // get position z in target frame
    // position_z = inertial_nav.get_altitude()/100.0 - g.tau_target_z; // the inav position estimate drifts a lot
    position_z = current_loc.alt/100.0 - target_z; // (m) --> to land 1.0 m above the ground

    // Tau object
    tau_z.set_pos_vel_time(position_z, velocity_z, time_now);
    tau_z.update_tau();

    // Update other objects for land_detector to work properly
    Vector3f des_vel;
    des_vel.x = tau_x.get_tau_velocity()*100.0;;
    des_vel.y = tau_y.get_tau_velocity()*100.0;;
    des_vel.z = tau_z.get_tau_velocity()*100.0;
    pos_control.set_desired_velocity(des_vel);

    // PID SECTION
    // set Error
    // float error = tau_z.error_switch(); // No longer using error switch
    float error = tau_z.hybrid();
    tau_pid_z.set_input_filter_d(error);
    
    // get pid error out. Note sometimes it is neceassy to reset the i flags? and sometimes you call them separately. 
    //      see some of the examples in the accel_to_throttle() function in the AC_PosControl library
    tau_control_input = tau_pid_z.get_pid();

    // Multiply by a constant and get the correct sign
    float tau_control_updated = -1.0*tau_control_input*10.0;

    ///////////////////////////
    /// TAU CONTROL SECTION ///
    ///////////////////////////
    
    /// SEND COMMANDS
    float tau_thr_out = tau_control_updated + hov_thr_default;  // Use only constant hov_thr_default value 
    tau_thr_out = constrain_float(tau_thr_out, hov_thr_default*0.7, hov_thr_default*3.5);
    attitude_control.set_throttle_out(tau_thr_out, true,  POSCONTROL_THROTTLE_CUTOFF_FREQ);
    
    // Logging the data
    tau_z_info = tau_z.get_tau_info();

    tau_xy_log.ext1 = tau_control_updated;

    /// PRINT TO SCREEN:
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, thr_out: %3.3f, ken: %3.3f, hyb: %3.3f, hov: %3.3f, cont_inp: %3.3f \n", position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref(), error, tau_thr_out, tau_z.kendoul(), tau_z.hybrid(), motors.get_throttle_hover(), tau_control_updated);
    // hal.console->printf("pos: %3.3f, vel: %3.3f, time: %3.3f, meas: %3.3f, ref: %3.3f, err: %3.3f, total_out: %3.3f, hov_thr: %3.3f, cont_inp: %3.3f \n",position_z, velocity_z, time_now, tau_z.meas(), tau_z.ref(), error, tau_thr_out, pos_control.get_throttle_hover(), tau_control_updated);
}

// Horizontal controller for tau landing called from myland_run()
static void tauland_run_horizontal_control()
{
    /////////////////////////////////
    //// TAU CONTROL STARTS HERE ////
    /////////////////////////////////

    // get current time    
    time_now = millis()*0.001f - tauland_start_time*0.001f; // (s), getting converted to a float 

    // Get Position
    position_x = inertial_nav.get_position().x/100.0;
    position_y = inertial_nav.get_position().y/100.0;

    // Get Velocity 
    // Smoothly transition between an initial velocity of 0.0 to actual velocity
    float velocityx = inertial_nav.get_velocity().x; // this is a function call to the ahrs (estimator) (AP_inertialNav) (cm/s)
    float velocityy = inertial_nav.get_velocity().y; // this is a function call to the ahrs (estimator) (AP_inertialNav) (cm/s)
    
    if (time_now <= 0.05) {
        velocity_x = 0.0;
        velocity_y = 0.0;
    } else if (time_now < 0.55) {
        float epsilon = (time_now - 0.05)*2.0;
        velocity_x = epsilon*velocityx/100.0; // (m/s)
        velocity_y = epsilon*velocityy/100.0; // (m/s)
    } else {
        velocity_x = velocityx/100.0; // (m/s)
        velocity_y = velocityy/100.0; // (m/s)
    }

    // Convert position to target frame
    float pos_t_x =  cos(alpha_approach)*(position_x - target_x) + sin(alpha_approach)*(position_y - target_y);
    float pos_t_y = -sin(alpha_approach)*(position_x - target_x) + cos(alpha_approach)*(position_y - target_y);

    // Convert velocity to target frame
    float vel_t_x =  cos(alpha_approach)*(velocity_x) + sin(alpha_approach)*(velocity_y);
    float vel_t_y = -sin(alpha_approach)*(velocity_x) + cos(alpha_approach)*(velocity_y);

    // Set data inside tau block
    tau_x.set_pos_vel_time(pos_t_x, vel_t_x, time_now);
    tau_y.set_pos_vel_time(pos_t_y, vel_t_y, time_now);

    tau_x.update_tau();
    tau_y.update_tau();

    // Error calculation - Use Hybrid instead of error_switch
    float error_x = tau_x.hybrid();
    float error_y = tau_y.hybrid();
    tau_pid_x.set_input_filter_d(error_x);
    tau_pid_y.set_input_filter_d(error_y);

    // Approach angle
    float psi = wrap_PI(ahrs.yaw); // Because sometimes the value of psi goes over 100

    cos_ap = cos(alpha_approach - psi);
    sin_ap = sin(alpha_approach - psi);

    /// Nonlinear transform
    float pitch =  1.0*(tau_pid_x.get_pid()*cos_ap - tau_pid_y.get_pid()*sin_ap); 
    float roll  = -1.0*(tau_pid_x.get_pid()*sin_ap + tau_pid_y.get_pid()*cos_ap);

    // constrain angles between [-15.0, 15.0] degrees and convert to centidegrees
    pitch = 100.0*constrain_float(pitch, -15.0, 15.0); 
    roll =  100.0*constrain_float(roll,  -15.0, 15.0); 

    ///////////////////////////////
    //// TAU CONTROL ENDS HERE ////
    ///////////////////////////////
 
    // Call attitude controller 
    // TO DO: replace the following function call with tau_land_run_yaw_control()
    //        because it will be the only function that calls the attitude_control obj
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(roll, pitch, 0.0);

    // Log Data - Declared in Arducopter
    tau_x_info = tau_x.get_tau_info();
    tau_y_info = tau_y.get_tau_info();

    tau_xy_log.cos_ap = cos_ap;
    tau_xy_log.sin_ap = sin_ap;
    tau_xy_log.alpha_approach = alpha_approach;
    tau_xy_log.alpha_0_approach = alpha_0_approach;
    tau_xy_log.beta_approach = beta_approach;
    tau_xy_log.roll = roll;
    tau_xy_log.pitch = pitch;
    tau_xy_log.psi = psi;

    // Print to screen
    float pitch_actual = pos_control.get_pitch(); // this is not actual pitch... some other measure
    float roll_actual = pos_control.get_roll();

    // hal.console->printf("time: %3.3f, posx: %3.3f, velx: %3.3f, pitch_des: %3.3f, pitch_actual: %5.3f, tau_x: %3.3f, tau_x_meas: %3.3f, err_x: %3.3f \n", time_now, position_x, velocity_x, pitch, pitch_actual, tau_x.meas(), tau_x.ref(), error_x);
    // hal.console->printf("time: %3.3f, posy: %3.3f, vely: %3.3f, roll_des:  %3.3f, roll_actual:  %5.3f, tau_y: %3.3f, tau_y_meas: %3.3f, err_y: %3.3f \n", time_now, position_y, velocity_y, roll,  roll_actual,  tau_y.meas(), tau_y.ref(), error_y);
    // hal.console->printf("cos_ap: %3.3f, sin_ap: %3.3f, alpha_approach: %3.3f, beta_approach: %3.3f, psi: %3.3f, pos_t_x: %3.3f, pos_t_y: %3.3f, vel_t_x: %3.3f, vel_t_y: %3.3f \n", cos_ap, sin_ap, alpha_approach, beta_approach, psi, pos_t_x, pos_t_y, vel_t_x, vel_t_y);
}

// Psi controller using tau 
static void tauland_run_yaw_control(float roll, float pitch)
{   
    // Call tau_land_run_yaw_control from tau_land_run_horizontal, passing roll, and pitch
    //  This function will then call the attitude controller. This keeps both of the horizontal
    //  and yaw control separate (and hopefully neater). 
    if (g.tau_psi_cons == 0.0){
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(roll, pitch, 0.0);
        return;
    }

    // Initializtions
    Vector3f euler;
    float yaw;
    float yaw_rate;

    ///////////////////////////
    /// TAU CONTROL SECTION ///
    ///////////////////////////

    // Get time now
    time_now = millis()*0.001f - tauland_start_time*0.001f;

    // Get yaw
    if (!ahrs.get_secondary_attitude(euler)) {
        ahrs.get_NavEKF().getEulerAngles(euler);    // get yaw from either get_secondary or getEulerAngles
    }
    yaw = wrap_PI(euler.z); // radians
    
    // Get yaw rate
    yaw_rate = ahrs.get_gyro_for_control().z;       // Returned in body frame radians/s

    // Tau object
    tau_yaw.set_pos_vel_time(yaw, yaw_rate, time_now);
    tau_yaw.update_tau();

    // Tau PID
    float error = tau_yaw.error_switch();
    tau_pid_yaw.set_input_filter_d(error);
    float yaw_control = tau_pid_yaw.get_pid();

    ///////////////////////////
    /// TAU CONTROL SECTION ///
    ///////////////////////////

    // Send Commands
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(roll, pitch, yaw_control);

    // Log Data
    tau_yaw_info = tau_yaw.get_tau_info();

    // Print to screen
    // hal.console->printf("time: %3.3f, yaw: %3.3f, yaw_rate: %3.3f, tau_yaw_meas: %3.3f, tau_yaw_ref: %3.3f, err: %3.3f, control_input: %3.3f \n", time_now, yaw, yaw_rate, tau_yaw.meas(), tau_yaw.ref(), error, yaw_control);
}

// Horizontal controller to hold horizontal position (copied from control_land in newer versions)
static void tauland_run_hold_horizontal_control()
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
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(nav_roll, nav_pitch, target_yaw_rate);
}

// Vertical controller to hold vertical position (copied from mode_stop) 
static void tauland_run_hold_vertical_control()
{
    float target_climb_rate = 0.0;

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
    pos_control.update_z_controller();
}

// Check to see if we have reached the target point and switch out of tauland mode
static void tauland_check_destination()
{   
    if (time_now >= tau_z.final_time()*0.95) { 
        switch(tau_land_mode) {
            case TAU_VERTICAL_TAU_HORIZONTAL:
            
            case TAU_VERTICAL_HOLD_HORIZONTAL:
                // Removed requirment of count_landed for vertical
                if (g.tau_target_z <= 0.2 and current_loc.alt <= 25.0) { 
                    set_mode(LAND);
                } else if (time_now >= tau_z.final_time()*0.99) {
                    set_mode(STOP);
                }

                break;

            case HOLD_VERTICAL_TAU_HORIZONTAL:
                if (time_now >= tau_z.final_time()*0.99) {
                    set_mode(STOP);
                }
                break;

            default:
                set_mode(STOP);
                break;    
        }
    }
}