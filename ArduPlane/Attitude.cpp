#include "Plane.h"

/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
 */
float Plane::get_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(aspeed)) {
        if (aspeed > auto_state.highest_airspeed) {
            auto_state.highest_airspeed = aspeed;
        }
        if (aspeed > 0.0001f) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        // ensure we have scaling over the full configured airspeed
        float scale_min = MIN(0.5, (0.5 * aparm.airspeed_min) / g.scaling_speed);
        float scale_max = MAX(2.0, (1.5 * aparm.airspeed_max) / g.scaling_speed);
        speed_scaler = constrain_float(speed_scaler, scale_min, scale_max);

        if (quadplane.in_vtol_mode() && hal.util->get_soft_armed()) {
            // when in VTOL modes limit surface movement at low speed to prevent instability
            float threshold = aparm.airspeed_min * 0.5;
            if (aspeed < threshold) {
                float new_scaler = linear_interpolate(0, g.scaling_speed / threshold, aspeed, 0, threshold);
                speed_scaler = MIN(speed_scaler, new_scaler);

                // we also decay the integrator to prevent an integrator from before
                // we were at low speed persistint at high speed
                rollController.decay_I();
                pitchController.decay_I();
                yawController.decay_I();
            }
        }
    } else if (hal.util->get_soft_armed()) {
        // scale assumed surface movement using throttle output
        float throttle_out = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), 1);
        speed_scaler = sqrtf(THROTTLE_CRUISE / throttle_out);
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain_float(speed_scaler, 0.6f, 1.67f);
    } else {
        // no speed estimate and not armed, use a unit scaling
        speed_scaler = 1;
    }
    return speed_scaler;
}

/*
  return true if the current settings and mode should allow for stick mixing
 */
bool Plane::stick_mixing_enabled(void)
{
    if (auto_throttle_mode && auto_navigation_mode) {
        // we're in an auto mode. Check the stick mixing flag
        if (g.stick_mixing != STICK_MIXING_DISABLED &&
            g.stick_mixing != STICK_MIXING_VTOL_YAW &&
            geofence_stickmixing() &&
            failsafe.state == FAILSAFE_NONE &&
            !rc_failsafe_active()) {
            // we're in an auto mode, and haven't triggered failsafe
            return true;
        } else {
            return false;
        }
    }

    if (failsafe.rc_failsafe && g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // don't do stick mixing in FBWA glide mode
        return false;
    }

    // non-auto mode. Always do stick mixing
    return true;
}


/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
void Plane::stabilize_roll(float speed_scaler)
{
	/// Limit for nav roll
	nav_roll_cd	= constrain_float(nav_roll_cd, -4000, 4000);
	
	if(control_mode == &mode_stabilize){
        //hal.console->printf("\nMODE STABILIZE\n");
		if(ail_val_const <= 1200){
			ail_val_const = ail_val_const + 10;
		}
        roll_value_pid  = rollController.get_pid_roll( - ahrs.roll_sensor*0.01f, start_cl_sina) + ail_val_const;
    }
    else{
        /// Auto mode
		if(ail_val_const >= 900){
			ail_val_const = ail_val_const - 10;
		}
        if(values_test_sina[9] <= 1500){
            roll_value_pid  = rollController.get_pid_roll(nav_roll_cd*0.01f - ahrs.roll_sensor*0.01f, start_cl_sina) + ail_val_const;
        }
        else{
		///Dive mode           
            Course_to_target_point  = CourseToPointShortDis(loc5.lat*0.0000001, loc5.lng*0.0000001, target_point_lat_sina, target_point_lng_sina);
            course_target_error_    = constrain_float(Course_to_target_point - gps.ground_course(), -50, 50);
			course_target_error_ 	= AngleErrTo180(course_target_error_);
            roll_value_pid 			= rollController.get_pid_roll(course_target_error_ - ahrs.roll_sensor*0.01f, 1) + ail_val_const;      
        }
    }

    // Full controll functions, start CL if delay 100ms after launch
    if(start_cl_sina == 1 && control_mode != &mode_manual){
        //hal.console->printf("\nCL WORK!!!!!!!!!!!!!!\n");
        SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_right,     - roll_value_pid*1 - pitch_value_pid*0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_left,      - roll_value_pid*1 + pitch_value_pid*0);
    }
	else{
		SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_right,     - (values_test_sina[0]-1500)*11.25);
        SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_left,      - (values_test_sina[0]-1500)*11.25);
	}	
}

float Plane::set_xpl_var(int xpl_start_var)
{
    if(xpl_start_var == 1)
        return pitch_Xplane;
    else
        return pitch_Xplane;
}



/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
void Plane::stabilize_pitch(float speed_scaler)
{
    
	if(control_mode == &mode_stabilize){
        //hal.console->printf("\nMODE STABILIZE\n");
		if(elev_val_const <= 1100){
			elev_val_const = elev_val_const + 10;
		}
        pitch_value_pid = pitchController.get_servo_out_pitch(ahrs.pitch_sensor*0.01f - 15, start_cl_sina) + elev_val_const;
    }
    else{
        /// Auto mode
		if(elev_val_const >= 0){
			elev_val_const = elev_val_const - 10;
		}
        if(values_test_sina[9] <= 1500){
            pitch_value_pid = pitchController.get_pid_alt(next_WP_loc.alt*0.01f - loc5.alt*0.01f, loc5.alt*0.01f, ahrs.pitch_sensor*0.01f, start_cl_sina) + elev_val_const;
        }
        else{
		///Dive mode           
            Dist_to_target_point        = DistanceBetween2Points(loc5.lat*0.0000001f, loc5.lng*0.0000001f, target_point_lat_sina, target_point_lng_sina);
            dive_ang_sina               = atanf((loc5.alt*0.01f - target_point_alt_sina)/(Dist_to_target_point + 10))*180/3.1415926;
            yat                         = atanf(gps.velocity().z/gps.ground_speed())*180/3.1415926;    
            pitch_value_pid             = pitchController.get_pid_pitch_dive_gps(dive_ang_sina - yat, start_cl_sina, dive_ang_sina, yat, loc5.alt*0.01f) + elev_val_const;      
        }
    }

    // Full controll functions, start CL if delay 100ms after launch
    if(start_cl_sina == 1 && control_mode != &mode_manual){
        //hal.console->printf("\nCL WORK!!!!!!!!!!!!!!\n");
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_right,    - roll_value_pid*0 + pitch_value_pid*1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_left,     - roll_value_pid*0 - pitch_value_pid*1);
    }
	else{
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_right,      (values_test_sina[1]-1500)*11.25);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_left,     - (values_test_sina[1]-1500)*11.25);
	}
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
void Plane::stabilize_speed(float speed_scaler)
{	
	///Manul making less/more throttle for test
	if(values_test_sina[8] >= 1300 && values_test_sina[8] <= 1700){
		throttle_value_sina = throttle_value_sina - 0.1;
	}
	if(values_test_sina[8] >= 1700){
		throttle_value_sina = throttle_value_sina + 0.1;
	}
	throttle_value_sina = constrain_float(throttle_value_sina, 0, 100);
	
	///Making soft changing of throttle
	if((throttle_value_sina - old_comthrval_sina) > 5){
		comthrval_sina = old_comthrval_sina + 0.2;							
	}
	if((throttle_value_sina - old_comthrval_sina) < -5){
		comthrval_sina = old_comthrval_sina - 0.2;							
	}
	
	///Start Flight or urgently go to 100%
	if(values_test_sina[7] >= 1500 || control_mode == &mode_stabilize){
		comthrval_sina = 100;
		throttle_value_sina = 70;
	}
	
	///Sending throttle to ESC
    if(start_eng_sina == true && control_mode != &mode_manual){
        //hal.console->printf("\nCL WORK!!!!!!!!!!!!!!\n");
		SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, comthrval_sina);
		old_comthrval_sina	=	comthrval_sina;	
    }
	else{
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, (values_test_sina[2] - 1104)/8.2);
	}
}


/*
  this gives the user control of the aircraft in stabilization modes
 */
void Plane::stabilize_stick_mixing_direct()
{
    if (!stick_mixing_enabled() ||
        control_mode == &mode_acro ||
        control_mode == &mode_fbwa ||
        control_mode == &mode_autotune ||
        control_mode == &mode_fbwb ||
        control_mode == &mode_cruise ||
        control_mode == &mode_qstabilize ||
        control_mode == &mode_qhover ||
        control_mode == &mode_qloiter ||
        control_mode == &mode_qland ||
        control_mode == &mode_qrtl ||
        control_mode == &mode_qacro ||
        control_mode == &mode_training ||
        control_mode == &mode_qautotune) {
        return;
    }
    int16_t aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    aileron = channel_roll->stick_mixing(aileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);

    int16_t elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    elevator = channel_pitch->stick_mixing(elevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
}

/*
  this gives the user control of the aircraft in stabilization modes
  using FBW style controls
 */
void Plane::stabilize_stick_mixing_fbw()
{
    if (!stick_mixing_enabled() ||
        control_mode == &mode_acro ||
        control_mode == &mode_fbwa ||
        control_mode == &mode_autotune ||
        control_mode == &mode_fbwb ||
        control_mode == &mode_cruise ||
        control_mode == &mode_qstabilize ||
        control_mode == &mode_qhover ||
        control_mode == &mode_qloiter ||
        control_mode == &mode_qland ||
        control_mode == &mode_qrtl ||
        control_mode == &mode_qacro ||
        control_mode == &mode_training ||
        control_mode == &mode_qautotune ||
        (control_mode == &mode_auto && g.auto_fbw_steer == 42)) {
        return;
    }
    // do FBW style stick mixing. We don't treat it linearly
    // however. For inputs up to half the maximum, we use linear
    // addition to the nav_roll and nav_pitch. Above that it goes
    // non-linear and ends up as 2x the maximum, to ensure that
    // the user can direct the plane in any direction with stick
    // mixing.
    float roll_input = channel_roll->norm_input();
    if (roll_input > 0.5f) {
        roll_input = (3*roll_input - 1);
    } else if (roll_input < -0.5f) {
        roll_input = (3*roll_input + 1);
    }
    nav_roll_cd += roll_input * roll_limit_cd;
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
    
    float pitch_input = channel_pitch->norm_input();
    if (pitch_input > 0.5f) {
        pitch_input = (3*pitch_input - 1);
    } else if (pitch_input < -0.5f) {
        pitch_input = (3*pitch_input + 1);
    }
    if (fly_inverted()) {
        pitch_input = -pitch_input;
    }
    if (pitch_input > 0) {
        nav_pitch_cd += pitch_input * aparm.pitch_limit_max_cd;
    } else {
        nav_pitch_cd += -(pitch_input * pitch_limit_min_cd);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}


/*
  stabilize the yaw axis. There are 3 modes of operation:
    - hold a specific heading with ground steering
    - rate controlled with ground steering
    - yaw control for coordinated flight    
 */
void Plane::stabilize_yaw(float speed_scaler)
{
    if (landing.is_flaring()) {
        // in flaring then enable ground steering
        steering_control.ground_steering = true;
    } else {
        // otherwise use ground steering when no input control and we
        // are below the GROUND_STEER_ALT
        steering_control.ground_steering = (channel_roll->get_control_in() == 0 && 
                                            fabsf(relative_altitude) < g.ground_steer_alt);
        if (!landing.is_ground_steering_allowed()) {
            // don't use ground steering on landing approach
            steering_control.ground_steering = false;
        }
    }


    /*
      first calculate steering_control.steering for a nose or tail
      wheel. We use "course hold" mode for the rudder when either performing
      a flare (when the wings are held level) or when in course hold in
      FBWA mode (when we are below GROUND_STEER_ALT)
     */
    if (landing.is_flaring() ||
        (steer_state.hold_course_cd != -1 && steering_control.ground_steering)) {
        calc_nav_yaw_course();
    } else if (steering_control.ground_steering) {
        calc_nav_yaw_ground();
    }

    /*
      now calculate steering_control.rudder for the rudder
     */
    calc_nav_yaw_coordinated(speed_scaler);
}


/*
  a special stabilization function for training mode
 */
void Plane::stabilize_training(float speed_scaler)
{
    if (training_manual_roll) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, channel_roll->get_control_in());
    } else {
        // calculate what is needed to hold
        ///stabilize_roll(speed_scaler);
        if ((nav_roll_cd > 0 && channel_roll->get_control_in() < SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)) ||
            (nav_roll_cd < 0 && channel_roll->get_control_in() > SRV_Channels::get_output_scaled(SRV_Channel::k_aileron))) {
            // allow user to get out of the roll
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, channel_roll->get_control_in());
        }
    }

    if (training_manual_pitch) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, channel_pitch->get_control_in());
    } else {
        ///stabilize_pitch(speed_scaler);
        if ((nav_pitch_cd > 0 && channel_pitch->get_control_in() < SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)) ||
            (nav_pitch_cd < 0 && channel_pitch->get_control_in() > SRV_Channels::get_output_scaled(SRV_Channel::k_elevator))) {
            // allow user to get back to level
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, channel_pitch->get_control_in());
        }
    }

    stabilize_yaw(speed_scaler);
}


/*
  this is the ACRO mode stabilization function. It does rate
  stabilization on roll and pitch axes
 */
void Plane::stabilize_acro(float speed_scaler)
{
    float roll_rate = (channel_roll->get_control_in()/4500.0f) * g.acro_roll_rate;
    float pitch_rate = (channel_pitch->get_control_in()/4500.0f) * g.acro_pitch_rate;

    /*
      check for special roll handling near the pitch poles
     */
    if (g.acro_locking && is_zero(roll_rate)) {
        /*
          we have no roll stick input, so we will enter "roll locked"
          mode, and hold the roll we had when the stick was released
         */
        if (!acro_state.locked_roll) {
            acro_state.locked_roll = true;
            acro_state.locked_roll_err = 0;
        } else {
            acro_state.locked_roll_err += ahrs.get_gyro().x * G_Dt;
        }
        int32_t roll_error_cd = -ToDeg(acro_state.locked_roll_err)*100;
        nav_roll_cd = ahrs.roll_sensor + roll_error_cd;
        // try to reduce the integrated angular error to zero. We set
        // 'stabilze' to true, which disables the roll integrator
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rollController.get_servo_out(roll_error_cd,
                                                                                             speed_scaler,
                                                                                             true));
    } else {
        /*
          aileron stick is non-zero, use pure rate control until the
          user releases the stick
         */
        acro_state.locked_roll = false;
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rollController.get_rate_out(roll_rate,  speed_scaler));
    }

    if (g.acro_locking && is_zero(pitch_rate)) {
        /*
          user has zero pitch stick input, so we lock pitch at the
          point they release the stick
         */
        if (!acro_state.locked_pitch) {
            acro_state.locked_pitch = true;
            acro_state.locked_pitch_cd = ahrs.pitch_sensor;
        }
        // try to hold the locked pitch. Note that we have the pitch
        // integrator enabled, which helps with inverted flight
        nav_pitch_cd = acro_state.locked_pitch_cd;
        // SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitchController.get_servo_out(nav_pitch_cd - ahrs.pitch_sensor,
                                                                                               // speed_scaler,
                                                                                               // false));
		SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, data_for_stend.ctrl[1] = pitchController.get_servo_out_pitch(ahrs.pitch_sensor, 1));
    } else {
        /*
          user has non-zero pitch input, use a pure rate controller
         */
        acro_state.locked_pitch = false;
        //SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitchController.get_rate_out(pitch_rate, speed_scaler));
		SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, data_for_stend.ctrl[1] = pitchController.get_servo_out_pitch(ahrs.pitch_sensor, 1));
    }

    /*
      manual rudder for now
     */
    steering_control.steering = steering_control.rudder = rudder_input();
}



/////////////////////////////////////////////////////////////


/*
  main stabilization function for all 3 axes
 */
void Plane::stabilize()
{
    
    if(X_Plain == true){
        // elev1_Xplane    = pitchController.XPL_get_pid_pitch(pitch_Xplane, w_y_XPlane);
        elev1_Xplane  = pitchController.XPL_get_pid_alt(200 - alt_Xplane, alt_Xplane, pitch_Xplane, w_y_XPlane);//elev1_Xplane;
        elev1_2_Xplane  = pitchController.XPL_get_pid_pitch(pitch_Xplane, w_y_XPlane);//pitch_Xplane;//pitchController.XPL_get_pid_pitch(pitch_Xplane, w_y_XPlane);
        
        Lailn1_Xplane   = rollController.XPL_get_pid_roll(roll_Xplane, w_x_Xplane);
        Railn1_Xplane   = yawController.XPL_get_pid_speed(40 - trueSpd_Xplane);
        // hal.console->printf("\n\n\n\nPITCH = %f\n\n\n\n", pitch_Xplane);
        // hal.console->printf("\n\n\n\nROLL = %f\n\n\n\n", roll_Xplane);
        return;
    }
	
    // plane.pitch_Xplane  = -15/57.2958;
    // roll_Xplane         = -15/57.2958;    
    /// read joistick commands
    rc().get_radio_in(values_test_sina, ARRAY_SIZE(values_test_sina));
    
    // // Imitaition of launch
    // if(values_test_sina[7] >= 1500){
        // start_sina  = true;
        // start_cl_sina = 1;
        // start_eng_sina = true;
    // }
    // else{
        // delay_sina   = 0;
        // start_eng_sina = false;
        // start_cl_sina = 0;
    // }
    
    /// Registrate launch by accel
    if(((float)AP::ins().get_accel().x >= 100 || (float)AP::ins().get_accel().x <= -100) || start_sina == true){
        start_sina = true;
    }
    else{
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 		  (values_test_sina[2] - 1104)/8.2);
        SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_right,     - (values_test_sina[0]-1500)*11.25);
        SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_left,      - (values_test_sina[0]-1500)*11.25);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_right,      (values_test_sina[1]-1500)*11.25);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_left,     - (values_test_sina[1]-1500)*11.25); 
    }

    //Calculating delay
    //if delay = 300ms, start engine 100%, else wait delay
    if(start_sina == true && delay_sina <= 20){
        delay_sina ++;
    }
    if(delay_sina >= 10){
        start_cl_sina = 1;
    }
    if(delay_sina >= 15){
        start_eng_sina = true;
    }
    
    // manual      : CL(joistick commands)
    // stabilize   : CL pitch(10), roll(0)
    // others      : CL pitch(-5), roll(nav_roll_cd)
    if (control_mode == &mode_manual){
        //hal.console->printf("\nMODE MANUAL\n");
        // pitch_value_pid = pitchController.get_servo_out_pitch(ahrs.pitch_sensor*0.01f - ((values_test_sina[1] - 1500)/20), start_cl_sina);
        // roll_value_pid  = rollController.get_pid_roll(((values_test_sina[0] - 1500)/20) - ahrs.roll_sensor*0.01f, start_cl_sina);
        pitch_value_pid = (values_test_sina[1]-1500)*11.25;
        roll_value_pid  = (values_test_sina[0]-1500)*11.25;
    }
    else if(control_mode == &mode_stabilize){
        //hal.console->printf("\nMODE STABILIZE\n");
		if(elev_val_const <= 1100){
			elev_val_const = elev_val_const + 10;
		}
		if(ail_val_const <= 1200){
			ail_val_const = ail_val_const + 10;
		}
        pitch_value_pid = pitchController.get_servo_out_pitch(ahrs.pitch_sensor*0.01f - 15, start_cl_sina) + elev_val_const;
        roll_value_pid  = rollController.get_pid_roll( - ahrs.roll_sensor*0.01f, start_cl_sina) + ail_val_const;
    }
    else if(control_mode == &mode_auto){
        // hal.console->printf("\nMODE AUTO\n");
		if(elev_val_const >= 0){
			elev_val_const = elev_val_const - 10;
		}
		if(ail_val_const >= 900){
			ail_val_const = ail_val_const - 10;
		}
        if(values_test_sina[9] <= 1500){
			/// Auto mode
            //Make less atl for sina
            if(start_cl_sina == 1){
                if(values_test_sina[8] >= 1300 && values_test_sina[8] <= 1700){
                    throttle_value_sina = throttle_value_sina - 0.1;
					if(throttle_value_sina <= 0){
						throttle_value_sina = 0;
					}
                }
                if(values_test_sina[8] >= 1700){
                    throttle_value_sina = throttle_value_sina + 0.1;
					if(throttle_value_sina >= 100){
						throttle_value_sina = 100;
					}
                }
            }
            pitch_value_pid = pitchController.get_pid_alt(next_WP_loc.alt*0.01f - loc5.alt*0.01f, loc5.alt*0.01f, ahrs.pitch_sensor*0.01f, start_cl_sina) + elev_val_const;
            roll_value_pid  = rollController.get_pid_roll(nav_roll_cd*0.01f - ahrs.roll_sensor*0.01f, start_cl_sina) + ail_val_const;
        }
        else{
			///Dive mode
			//hal.console->printf("\nDIVE MODE!!!!!\n");
            Dist_to_target_point        = DistanceBetween2Points(loc5.lat*0.0000001f, loc5.lng*0.0000001f, target_point_lat_sina, target_point_lng_sina);
            dive_ang_sina               = atanf((loc5.alt*0.01f - target_point_alt_sina)/(Dist_to_target_point + 10))*180/3.1415926;
            yat                         = atanf(gps.velocity().z/gps.ground_speed())*180/3.1415926;    
            pitch_value_pid             = pitchController.get_pid_pitch_dive_gps(dive_ang_sina - yat, start_cl_sina, dive_ang_sina, yat, loc5.alt*0.01f) + elev_val_const;
            
            Course_to_target_point      = CourseToPointShortDis(loc5.lat*0.0000001, loc5.lng*0.0000001, target_point_lat_sina, target_point_lng_sina);
            course_target_error_        = Course_to_target_point - gps.ground_course();
            
            if(course_target_error_ < -180){
                course_target_error_ = course_target_error_ + 360;
			}
            if(course_target_error_ > 180){
                course_target_error_ = course_target_error_ - 360;
			}
            
			course_target_error_	= constrain_float(course_target_error_, -50, 50);
            roll_value_pid = rollController.get_pid_roll(course_target_error_ - ahrs.roll_sensor*0.01f, 1) + ail_val_const;      
        }
    }
    else{
        //hal.console->printf("\nMODE NONE/RTL\n");
        pitch_value_pid = pitchController.get_servo_out_pitch(ahrs.pitch_sensor*0.01f - ((values_test_sina[1] - 1500)/20), start_cl_sina);
        roll_value_pid  = rollController.get_pid_roll(nav_roll_cd*0.01f - ahrs.roll_sensor*0.01f, start_cl_sina);
    }


    // Full controll functions, start CL if delay 100ms after launch
    if(start_cl_sina == 1){
        if(start_eng_sina == true){
            if(control_mode != &mode_manual){
                //hal.console->printf("\nEngine 100!!!!!\n");
                if(control_mode == &mode_stabilize){
                    comthrval_sina = 100;
                }
                else{
					if(values_test_sina[7] >= 1500){
						comthrval_sina = 100;
						throttle_value_sina = 70;
					}
					else{
						comthrval_sina = throttle_value_sina;
						if((throttle_value_sina - old_comthrval_sina) > 5){
							comthrval_sina = old_comthrval_sina + 0.1;							
						}
						if((throttle_value_sina - old_comthrval_sina) < -5){
							comthrval_sina = old_comthrval_sina - 0.1;							
						}
					}
                }
            }
            else{
                comthrval_sina = (values_test_sina[2] - 1104)/8.2;
            }
        }
        else{
            comthrval_sina = 0.0f;
        }
        //hal.console->printf("\nCL WORK!!!!!!!!!!!!!!\n");
        SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_right,     - roll_value_pid*1 - pitch_value_pid*0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_left,      - roll_value_pid*1 + pitch_value_pid*0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_right,    - roll_value_pid*0 + pitch_value_pid*1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_left,     - roll_value_pid*0 - pitch_value_pid*1);
		SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 		comthrval_sina);
		old_comthrval_sina	=	comthrval_sina;	
    }

    // if mode = manual, return
    if (control_mode == &mode_manual) {
        // reset steering controls
        steer_state.locked_course = false;
        steer_state.locked_course_err = 0;
        return;
    }
    float speed_scaler = get_speed_scaler();

    if (quadplane.in_tailsitter_vtol_transition()) {
        /*
          during transition to vtol in a tailsitter try to raise the
          nose rapidly while keeping the wings level
         */
        nav_pitch_cd = constrain_float((quadplane.tailsitter.transition_angle+5)*100, 5500, 8500),
        nav_roll_cd = 0;
    }

    uint32_t now = AP_HAL::millis();
    if (now - last_stabilize_ms > 2000) {
        // if we haven't run the rate controllers for 2 seconds then
        // reset the integrators
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();

        // and reset steering controls
        steer_state.locked_course = false;
        steer_state.locked_course_err = 0;
    }
    last_stabilize_ms = now;

    if (control_mode == &mode_training) {
        stabilize_training(speed_scaler);
    } else if (control_mode == &mode_acro) {
        stabilize_acro(speed_scaler);
    } else if ((control_mode == &mode_qstabilize ||
                control_mode == &mode_qhover ||
                control_mode == &mode_qloiter ||
                control_mode == &mode_qland ||
                control_mode == &mode_qrtl ||
                control_mode == &mode_qacro ||
                control_mode == &mode_qautotune) &&
               !quadplane.in_tailsitter_vtol_transition()) {
        quadplane.control_run();
    } else {
        if (g.stick_mixing == STICK_MIXING_FBW && control_mode != &mode_stabilize) {
            stabilize_stick_mixing_fbw();
        }
        ///stabilize_roll(speed_scaler);
        ///stabilize_pitch(speed_scaler);
        if (g.stick_mixing == STICK_MIXING_DIRECT || control_mode == &mode_stabilize) {
            stabilize_stick_mixing_direct();
        }
        stabilize_yaw(speed_scaler);
    }

    /*
      see if we should zero the attitude controller integrators. 
     */
    if (get_throttle_input() == 0 &&
        fabsf(relative_altitude) < 5.0f && 
        fabsf(barometer.get_climb_rate()) < 0.5f &&
        gps.ground_speed() < 3) {
        // we are low, with no climb rate, and zero throttle, and very
        // low ground speed. Zero the attitude controller
        // integrators. This prevents integrator buildup pre-takeoff.
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();

        // if moving very slowly also zero the steering integrator
        if (gps.ground_speed() < 1) {
            steerController.reset_I();            
        }
    }
	//const AP_HAL::HAL& hal = AP_HAL::get_HAL();
}

void Plane::calc_throttle()
{
    if (aparm.throttle_cruise <= 1) {
        // user has asked for zero throttle - this may be done by a
        // mission which wants to turn off the engine for a parachute
        // landing
        //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        return;
    }

    int32_t commanded_throttle = SpdHgt_Controller->get_throttle_demand();

    // Received an external msg that guides throttle in the last 3 seconds?
    if ((control_mode == &mode_guided || control_mode == &mode_avoidADSB) &&
            plane.guided_state.last_forced_throttle_ms > 0 &&
            millis() - plane.guided_state.last_forced_throttle_ms < 3000) {
        commanded_throttle = plane.guided_state.forced_throttle;
    }
	data_for_stend.ctrl[3] = commanded_throttle;
    
    if(start_eng_sina == true){
        if(control_mode != &mode_manual){
            //hal.console->printf("\nEngine 100!!!!!\n");
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100);
        }
        else{
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, (values_test_sina[2] - 1104)/8.2);
        }
    }
    else{
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
    }
    

        //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}

/*****************************************
* Calculate desired roll/pitch/yaw angles (in medium freq loop)
*****************************************/

/*
  calculate yaw control for coordinated flight
 */
void Plane::calc_nav_yaw_coordinated(float speed_scaler)
{
    bool disable_integrator = false;
    int16_t rudder_in = rudder_input();

    int16_t commanded_rudder;

    // Received an external msg that guides yaw in the last 3 seconds?
    if ((control_mode == &mode_guided || control_mode == &mode_avoidADSB) &&
            plane.guided_state.last_forced_rpy_ms.z > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.z < 3000) {
        commanded_rudder = plane.guided_state.forced_rpy_cd.z;
    } else {
        if (control_mode == &mode_stabilize && rudder_in != 0) {
            disable_integrator = true;
        }

        commanded_rudder = yawController.get_servo_out(speed_scaler, disable_integrator);

        // add in rudder mixing from roll
        commanded_rudder += SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * g.kff_rudder_mix;
        commanded_rudder += rudder_in;
    }
	data_for_stend.ctrl[2] = constrain_int16(commanded_rudder, -4500, 4500);;
    steering_control.rudder = constrain_int16(commanded_rudder, -4500, 4500);
}

/*
  calculate yaw control for ground steering with specific course
 */
void Plane::calc_nav_yaw_course(void)
{
    // holding a specific navigation course on the ground. Used in
    // auto-takeoff and landing
    int32_t bearing_error_cd = nav_controller->bearing_error_cd();
    steering_control.steering = steerController.get_steering_out_angle_error(bearing_error_cd);
    if (stick_mixing_enabled()) {
        steering_control.steering = channel_rudder->stick_mixing(steering_control.steering);
    }
    steering_control.steering = constrain_int16(steering_control.steering, -4500, 4500);
}

/*
  calculate yaw control for ground steering
 */
void Plane::calc_nav_yaw_ground(void)
{
    if (gps.ground_speed() < 1 && 
        get_throttle_input() == 0 &&
        flight_stage != AP_Vehicle::FixedWing::FLIGHT_TAKEOFF &&
        flight_stage != AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        // manual rudder control while still
        steer_state.locked_course = false;
        steer_state.locked_course_err = 0;
        steering_control.steering = rudder_input();
        return;
    }

    float steer_rate = (rudder_input()/4500.0f) * g.ground_steer_dps;
    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF ||
        flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        steer_rate = 0;
    }
    if (!is_zero(steer_rate)) {
        // pilot is giving rudder input
        steer_state.locked_course = false;        
    } else if (!steer_state.locked_course) {
        // pilot has released the rudder stick or we are still - lock the course
        steer_state.locked_course = true;
        if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_TAKEOFF &&
            flight_stage != AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
            steer_state.locked_course_err = 0;
        }
    }
    if (!steer_state.locked_course) {
        // use a rate controller at the pilot specified rate
        steering_control.steering = steerController.get_steering_out_rate(steer_rate);
    } else {
        // use a error controller on the summed error
        int32_t yaw_error_cd = -ToDeg(steer_state.locked_course_err)*100;
        steering_control.steering = steerController.get_steering_out_angle_error(yaw_error_cd);
    }
    steering_control.steering = constrain_int16(steering_control.steering, -4500, 4500);
}


/*
  calculate a new nav_pitch_cd from the speed height controller
 */
void Plane::calc_nav_pitch()
{
    // Calculate the Pitch of the plane
    // --------------------------------
    int32_t commanded_pitch = SpdHgt_Controller->get_pitch_demand();

    // Received an external msg that guides roll in the last 3 seconds?
    if ((control_mode == &mode_guided || control_mode == &mode_avoidADSB) &&
            plane.guided_state.last_forced_rpy_ms.y > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.y < 3000) {
        commanded_pitch = plane.guided_state.forced_rpy_cd.y;
    }

    nav_pitch_cd = constrain_int32(commanded_pitch, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}


/*
  calculate a new nav_roll_cd from the navigation controller
 */
void Plane::calc_nav_roll()
{
    int32_t commanded_roll = nav_controller->nav_roll_cd();

    // Received an external msg that guides roll in the last 3 seconds?
    if ((control_mode == &mode_guided || control_mode == &mode_avoidADSB) &&
            plane.guided_state.last_forced_rpy_ms.x > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.x < 3000) {
        commanded_roll = plane.guided_state.forced_rpy_cd.x;
    }

    nav_roll_cd = constrain_int32(commanded_roll, -roll_limit_cd, roll_limit_cd);
    update_load_factor();
}

/*
  adjust nav_pitch_cd for STAB_PITCH_DOWN_CD. This is used to make
  keeping up good airspeed in FBWA mode easier, as the plane will
  automatically pitch down a little when at low throttle. It makes
  FBWA landings without stalling much easier.
 */
void Plane::adjust_nav_pitch_throttle(void)
{
    int8_t throttle = throttle_percentage();
    if (throttle >= 0 && throttle < aparm.throttle_cruise && flight_stage != AP_Vehicle::FixedWing::FLIGHT_VTOL) {
        float p = (aparm.throttle_cruise - throttle) / (float)aparm.throttle_cruise;
        nav_pitch_cd -= g.stab_pitch_down * 100.0f * p;
    }
}


/*
  calculate a new aerodynamic_load_factor and limit nav_roll_cd to
  ensure that the load factor does not take us below the sustainable
  airspeed
 */
void Plane::update_load_factor(void)
{
    float demanded_roll = fabsf(nav_roll_cd*0.01f);
    if (demanded_roll > 85) {
        // limit to 85 degrees to prevent numerical errors
        demanded_roll = 85;
    }
    aerodynamic_load_factor = 1.0f / safe_sqrt(cosf(radians(demanded_roll)));

    if (quadplane.in_transition() &&
        (quadplane.options & QuadPlane::OPTION_LEVEL_TRANSITION)) {
        // the user wants transitions to be kept level to within LEVEL_ROLL_LIMIT
        roll_limit_cd = MIN(roll_limit_cd, g.level_roll_limit*100);
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        return;
    }
    
    if (!aparm.stall_prevention) {
        // stall prevention is disabled
        return;
    }
    if (fly_inverted()) {
        // no roll limits when inverted
        return;
    }
    if (quadplane.tailsitter_active()) {
        // no limits while hovering
        return;
    }
       

    float max_load_factor = smoothed_airspeed / MAX(aparm.airspeed_min, 1);
    if (max_load_factor <= 1) {
        // our airspeed is below the minimum airspeed. Limit roll to
        // 25 degrees
        nav_roll_cd = constrain_int32(nav_roll_cd, -2500, 2500);
        roll_limit_cd = MIN(roll_limit_cd, 2500);
    } else if (max_load_factor < aerodynamic_load_factor) {
        // the demanded nav_roll would take us past the aerodymamic
        // load limit. Limit our roll to a bank angle that will keep
        // the load within what the airframe can handle. We always
        // allow at least 25 degrees of roll however, to ensure the
        // aircraft can be maneuvered with a bad airspeed estimate. At
        // 25 degrees the load factor is 1.1 (10%)
        int32_t roll_limit = degrees(acosf(sq(1.0f / max_load_factor)))*100;
        if (roll_limit < 2500) {
            roll_limit = 2500;
        }
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
        roll_limit_cd = MIN(roll_limit_cd, roll_limit);
    }    
}