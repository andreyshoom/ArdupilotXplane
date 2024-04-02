/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Initial Code by Jon Challinger
//  Modified by Paul Riseborough

#include <AP_HAL/AP_HAL.h>
#include "AP_PitchController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PitchController::var_info[] = {

	// @Param: TCONST
	// @DisplayName: Pitch Time Constant
	// @Description: Time constant in seconds from demanded to achieved pitch angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
	// @Range: 0.4 1.0
	// @Units: s
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_PitchController, gains.tau,       0.5f),

	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: Proportional gain from pitch angle demands to elevator. Higher values allow more servo response but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0.1 3.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("P",        1, AP_PitchController, gains.P,          1.0f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: Damping gain from pitch acceleration to elevator. Higher values reduce pitching in turbulence, but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.2
	// @Increment: 0.01
	// @User: User
    AP_GROUPINFO("D",        2, AP_PitchController, gains.D,        0.04f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: Integrator gain from long-term pitch angle offsets to elevator. Higher values "trim" out offsets faster but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.5
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("I",        3, AP_PitchController, gains.I,        0.3f),

	// @Param: RMAX_UP
	// @DisplayName: Pitch up max rate
	// @Description: Maximum pitch up rate that the pitch controller demands (degrees/sec) in ACRO mode.
	// @Range: 0 100
	// @Units: deg/s
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX_UP",     4, AP_PitchController, gains.rmax,   0.0f),

	// @Param: RMAX_DN
	// @DisplayName: Pitch down max rate
	// @Description: This sets the maximum nose down pitch rate that the controller will demand (degrees/sec). Setting it to zero disables the limit.
	// @Range: 0 100
	// @Units: deg/s
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX_DN",     5, AP_PitchController, _max_rate_neg,   0.0f),

	// @Param: RLL
	// @DisplayName: Roll compensation
	// @Description: Gain added to pitch to keep aircraft from descending or ascending in turns. Increase in increments of 0.05 to reduce altitude loss. Decrease for altitude gain.
	// @Range: 0.7 1.5
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("RLL",      6, AP_PitchController, _roll_ff,        1.0f),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of pitch integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 3000 allows trim of up to 2/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      7, AP_PitchController, gains.imax,     3000),

	// @Param: FF
	// @DisplayName: Feed forward Gain
	// @Description: Gain from demanded rate to elevator output.
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("FF",        8, AP_PitchController, gains.FF,       0.0f),

	AP_GROUPEND
};



// int32_t AP_PitchController::get_servo_out_alt(float pitch_ha, float alt_err_ha, float current_alt_ha)
// {
    // _out_ant = get_pid_alt(alt_err_ha, current_alt_ha, pitch_ha, 0);
  
     // return constrain_float(_out_ant*(-100), -4500, 4500);
// }

int32_t AP_PitchController::get_servo_out_pitch(float pitch_pe, float error_with_pitch)
{
	// Calculate offset to pitch rate demand required to maintain pitch angle whilst banking
	// Calculate ideal turn rate from bank angle and airspeed assuming a level coordinated turn
	// Pitch rate offset is the component of turn rate about the pitch axis
       
	_pid_info.actual = pitch_pe;
    _out_ant = get_pid_pitch(pitch_pe, error_with_pitch);
    
    if(error_with_pitch <= 0){
        _pid_info.P = 0;
        _pid_info.I = 0;
        _pid_info.D = 0;
        _integrator = 0;
    }

	_pid_info.FF = error_with_pitch;
	return constrain_float(_out_ant*(-100), -4500, 4500);
    	// return 0;

	// return constrain_float(pitch_pe*(-100), -4500, 4500);
}

void AP_PitchController::reset_I()
{
	//_pid_info.I = 0;
}



float AP_PitchController::get_pid_alt(float alt_err_ha, float current_alt_ha, float pitch_ha, int cl_start)
{
    //hal.console->printf("\n\n Alt_error = %f\n", alt_err_ha);
    uint32_t tnow       = AP_HAL::millis();
    uint32_t dt         = tnow - _last_t;
    float output        = 0;
    float delta_time;
    float alt_err_kp    = alt_err_ha;
    
    _pid_info.target    = alt_err_ha;
    
    float    kp         = 0.423;		//0.423;       	//0.27;
    float    ki         = 0.081;		//0.081;        //0.071;
    float    k_pitch    = 0.398;		//0.398;        //0.328;
    float    k_omega_y  = 0.02192;		//0.02192;      //0.05825; 
    float    imax       = 30;           //5				//5;
	
    alt_err_ha = constrain_float(alt_err_ha, -20, 20);
	alt_err_kp = constrain_float(alt_err_kp, -10, 10);

    // _pid_info.target    = alt_err_ha;
    _last_t             = tnow;

    delta_time          = (float)dt / 1000.0f;

    // Compute proportional component
    _pid_info.P         = constrain_float(alt_err_ha * kp, -45, 45);
    output              += _pid_info.P;

    // Compute integral component if time has elapsed
    if ((fabsf(ki) > 0) && (dt > 0)) {
        _integrator     += (alt_err_ha * ki) * delta_time;
        if (_integrator < -imax) {
            _integrator = -imax;
        } else if (_integrator > imax) {
            _integrator = imax;
        }
        _pid_info.I     = _integrator;
        output          += _integrator;
    }
    

	output              = constrain_float(output, -45, 45);
    float omega_y       = _ahrs.get_gyro().y;
    output              = output - (k_pitch*pitch_ha + k_omega_y*ToDeg(omega_y));
    _pid_info.D         = -k_omega_y*ToDeg(omega_y);
    _pid_info.FF        = -k_pitch*pitch_ha;
    
    if(cl_start == 0){
        _pid_info.P = 0;
        _pid_info.I = 0;
        _pid_info.D = 0;
        _integrator = 0;
    }
    
    return constrain_float(output*100, -4500, 4500);
}

float AP_PitchController::get_pid_pitch(float pitch_pe, float error_with_pitch)
{
    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t;
    float output            = 0;
    float main_output       = 0;
    float delta_time;

	
	//kd_pitch = 0.15594 - (float)airspeed.get_airspeed()*0.00093;
	
	/// Param for MSP ///							
	float    kp_pitch 			= 0.785;		//0.675;	//0.478;
    float    ki_pitch 			= 0.1187;		//0.0687;	//0.037;
    float    kd_pitch 			= 0.03825;		//0.02825;	//0.01825;
    float    imax 				= 30;
    
	_pitch_error_pe = pitch_pe;// - gains.imax * 0.01f + 15;
    
    _last_t = tnow;

    delta_time = (float)dt / 1000.0f;
    //delta_time = 0.025;

    // Compute proportional component
    _pid_info.P = constrain_float(_pitch_error_pe * kp_pitch, -45, 45);
    output += _pid_info.P;

    // Compute integral component if time has elapsed
    if ((fabsf(ki_pitch) > 0) && (dt > 0)) {
        _integrator             += (_pitch_error_pe * ki_pitch) * delta_time;
        if (_integrator < -imax) {
            _integrator = -imax;
        } else if (_integrator > imax) {
            _integrator = imax;
        }
    }
	_pid_info.I = _integrator;
	output                          += _integrator;

	output = constrain_float(output, -45, 45);
    main_output = constrain_float(output, -45, 45);
    float omega_y = _ahrs.get_gyro().y;
	_pid_info.D = -kd_pitch*ToDeg(omega_y);
    main_output += _pid_info.D;
    _pid_info.target = _pitch_error_pe;
	_pid_info.FF = pitch_pe;
	
    
	pid_pitch_P = _pid_info.P;
	pid_pitch_I = _integrator;
	pid_pitch_D = _pid_info.D;
	pid_pitch_error = pitch_pe;
	pid_pitch_elevator = main_output;
    
    
	
    return main_output;
}


float AP_PitchController::get_pid_pitch_dive_gps(float pitch_pe, float error_with_pitch, float dive_ang_sina, float yat, float altitude_now)
{
    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t;
    float output            = 0;
    float main_output       = 0;
    float delta_time;
	
	/// Param for MSP ///							
	float    kp_pitch 			= gains.P;  //0.478;   0.075
    float    ki_pitch 			= gains.I;  //0.037;
    float    kd_pitch 			= gains.D;  //0.01825;
    float    imax 				= 30;
	
	_pid_info.target 	= dive_ang_sina;
	_pid_info.actual 	= yat;
	_pid_info.FF 		= altitude_now;
	_pid_info.error     = pitch_pe;	
    
	_pitch_error_pe = constrain_float(pitch_pe, -15, 15);// - gains.imax * 0.01f + 15;
    
    _last_t = tnow;

    delta_time = (float)dt / 1000.0f;

    // Compute proportional component
    _pid_info.P = constrain_float(_pitch_error_pe * kp_pitch, -45, 45);
    output += _pid_info.P;

    // Compute integral component if time has elapsed
    if ((fabsf(ki_pitch) > 0) && (dt > 0)) {
        _integrator             += (_pitch_error_pe * ki_pitch) * delta_time;
        if (_integrator < -imax) {
            _integrator = -imax;
        } else if (_integrator > imax) {
            _integrator = imax;
        }
    }
	_pid_info.I = _integrator;
	output                          += _integrator;

	output = constrain_float(output, -45, 45);
    main_output = constrain_float(output, -45, 45);
    float omega_y = _ahrs.get_gyro().y;
	_pid_info.D = -kd_pitch*ToDeg(omega_y);
    main_output += _pid_info.D;
    //_pid_info.target = _pitch_error_pe;
	
    
	pid_pitch_P = _pid_info.P;
	pid_pitch_I = _integrator;
	pid_pitch_D = _pid_info.D;
	pid_pitch_error = pitch_pe;
	pid_pitch_elevator = main_output;
    
    
	
    return constrain_float(main_output*(-100), -4500, 4500);
	
    // return output;
}



float AP_PitchController::XPL_get_pid_pitch(float pitch_pe, float XPL_w_y)
{
    float output            = 0;
    float main_output       = 0;
    float delta_time;

	
	//kd_pitch = 0.15594 - (float)airspeed.get_airspeed()*0.00093;
	
	/// Param for MSP ///							
	float    kp_pitch 			= gains.P;  //0.885;		//0.675;	//0.478;
    float    ki_pitch 			= gains.I;  //0.0387;		//0.0687;	//0.037;
    float    kd_pitch 			= gains.D;  //0.03825;		//0.02825;	//0.01825;
    float    imax 				= 30;
    
	_pitch_error_pe = gains.imax*0.01f - pitch_pe;// - gains.imax * 0.01f + 15;
    

    // delta_time = (float)dt / 1000.0f;
    delta_time = 0.02;

    // Compute proportional component
    _pid_info.P = constrain_float(_pitch_error_pe * kp_pitch, -45, 45);
    output += _pid_info.P;

    // Compute integral component if time has elapsed

        _XPL_integrator             += (_pitch_error_pe * ki_pitch) * delta_time;
        if (_XPL_integrator < -imax) {
            _XPL_integrator = -imax;
        } else if (_XPL_integrator > imax) {
            _XPL_integrator = imax;
        }

	_pid_info.I = _XPL_integrator;
	output                          += _XPL_integrator;

	output = constrain_float(output, -45, 45);
    main_output = constrain_float(output, -45, 45);

	_pid_info.D = -kd_pitch*XPL_w_y;
    main_output += _pid_info.D;
    _pid_info.target = _pitch_error_pe;
	_pid_info.FF = pitch_pe;
	
    return constrain_float(main_output/90, -0.5, 0.5);
}


float AP_PitchController::XPL_get_pid_alt(float alt_err_ha, float current_alt_ha, float pitch_ha, float XPL_w_y)
{
    //hal.console->printf("\n\n Alt_error = %f\n", alt_err_ha);
    
    alt_err_ha = alt_err_ha + gains.imax;
    float output        = 0;
    float delta_time;
    float alt_err_kp    = alt_err_ha;
    
    _pid_info.target    = alt_err_ha;
    
    float    kp         = 0.923;		//0.423;       	//0.27;
    float    ki         = 0.0281;		//0.081;        //0.071;
    float    k_pitch    = 0.578;		//0.398;        //0.328;
    float    k_omega_y  = 0.03192;		//0.02192;      //0.05825; 
    float    imax       = 20;           //5				//5;
	
    alt_err_ha = constrain_float(alt_err_ha, -20, 20);
	alt_err_kp = constrain_float(alt_err_kp, -10, 10);

    // _pid_info.target    = alt_err_ha;

    delta_time = 0.02;

    // Compute proportional component
    _pid_info.P         = constrain_float(alt_err_ha * kp, -45, 45);
    output              += _pid_info.P;

    // Compute integral component if time has elapsed
        _XPL_integrator2     += (alt_err_ha * ki) * delta_time;
        if (_XPL_integrator2 < -imax) {
            _XPL_integrator2 = -imax;
        } else if (_XPL_integrator2 > imax) {
            _XPL_integrator2 = imax;
        }
        _pid_info.I     = _XPL_integrator2;
        output          += _XPL_integrator2;
    
    if(pitch_ha >= 10){
        k_pitch = k_pitch + pitch_ha*0.015;
    }
    if(pitch_ha <= -10){
        k_pitch = k_pitch - pitch_ha*0.015;
    }

	output              = constrain_float(output, -45, 45);
    output              = output - (k_pitch*pitch_ha + k_omega_y*XPL_w_y);
    _pid_info.D         = -k_omega_y*XPL_w_y;
    _pid_info.FF        = -k_pitch*pitch_ha;
    
    
    return constrain_float(output/90, -0.45, 0.45);
}


// float AP_PitchController::XPL_get_pid_pitch_dive_gps(float pitch_pe, float error_with_pitch, float dive_ang_sina, float yat, float altitude_now)
// {
    // uint32_t tnow = AP_HAL::millis();
    // uint32_t dt = tnow - _last_t;
    // float output            = 0;
    // float main_output       = 0;
    // float delta_time;
	
	// /// Param for MSP ///							
	// float    kp_pitch 			= gains.P;  //0.478;   0.075
    // float    ki_pitch 			= gains.I;  //0.037;
    // float    kd_pitch 			= gains.D;  //0.01825;
    // float    imax 				= 30;
	
	// _pid_info.target 	= dive_ang_sina;
	// _pid_info.actual 	= yat;
	// _pid_info.FF 		= altitude_now;
	// _pid_info.error     = pitch_pe;	
    
	// _pitch_error_pe = constrain_float(pitch_pe, -15, 15);// - gains.imax * 0.01f + 15;
    
    // _last_t = tnow;

    // delta_time = (float)dt / 1000.0f;

    // // Compute proportional component
    // _pid_info.P = constrain_float(_pitch_error_pe * kp_pitch, -45, 45);
    // output += _pid_info.P;

    // // Compute integral component if time has elapsed
    // if ((fabsf(ki_pitch) > 0) && (dt > 0)) {
        // _integrator             += (_pitch_error_pe * ki_pitch) * delta_time;
        // if (_integrator < -imax) {
            // _integrator = -imax;
        // } else if (_integrator > imax) {
            // _integrator = imax;
        // }
    // }
	// _pid_info.I = _integrator;
	// output                          += _integrator;

	// output = constrain_float(output, -45, 45);
    // main_output = constrain_float(output, -45, 45);
    // float omega_y = _ahrs.get_gyro().y;
	// _pid_info.D = -kd_pitch*ToDeg(omega_y);
    // main_output += _pid_info.D;
    // //_pid_info.target = _pitch_error_pe;
	
    
	// pid_pitch_P = _pid_info.P;
	// pid_pitch_I = _integrator;
	// pid_pitch_D = _pid_info.D;
	// pid_pitch_error = pitch_pe;
	// pid_pitch_elevator = main_output;
    
    
	
    // return constrain_float(main_output*(-100), -4500, 4500);
	
    // // return output;
// }



// float AP_PitchController::XPL_get_pid_alt(float alt_err_ha, float current_alt_ha, float pitch_ha, int cl_start)
// {
    // //hal.console->printf("\n\n Alt_error = %f\n", alt_err_ha);
    // uint32_t tnow       = AP_HAL::millis();
    // uint32_t dt         = tnow - _last_t;
    // float output        = 0;
    // float delta_time;
    // float alt_err_kp    = alt_err_ha;
    
    // _pid_info.target    = alt_err_ha;
    
    // float    kp         = 0.423;		//0.423;       	//0.27;
    // float    ki         = 0.081;		//0.081;        //0.071;
    // float    k_pitch    = 0.398;		//0.398;        //0.328;
    // float    k_omega_y  = 0.02192;		//0.02192;      //0.05825; 
    // float    imax       = 30;           //5				//5;
	
    // alt_err_ha = constrain_float(alt_err_ha, -20, 20);
	// alt_err_kp = constrain_float(alt_err_kp, -10, 10);

    // // _pid_info.target    = alt_err_ha;
    // _last_t             = tnow;

    // delta_time          = (float)dt / 1000.0f;

    // // Compute proportional component
    // _pid_info.P         = constrain_float(alt_err_ha * kp, -45, 45);
    // output              += _pid_info.P;

    // // Compute integral component if time has elapsed
    // if ((fabsf(ki) > 0) && (dt > 0)) {
        // _integrator     += (alt_err_ha * ki) * delta_time;
        // if (_integrator < -imax) {
            // _integrator = -imax;
        // } else if (_integrator > imax) {
            // _integrator = imax;
        // }
        // _pid_info.I     = _integrator;
        // output          += _integrator;
    // }
    

	// output              = constrain_float(output, -45, 45);
    // float omega_y       = _ahrs.get_gyro().y;
    // output              = output - (k_pitch*pitch_ha + k_omega_y*ToDeg(omega_y));
    // _pid_info.D         = -k_omega_y*ToDeg(omega_y);
    // _pid_info.FF        = -k_pitch*pitch_ha;
    
    // if(cl_start == 0){
        // _pid_info.P = 0;
        // _pid_info.I = 0;
        // _pid_info.D = 0;
        // _integrator = 0;
    // }
    
    // return constrain_float(output*100, -4500, 4500);
// }


