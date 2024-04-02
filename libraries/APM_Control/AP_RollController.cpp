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

//	Code by Jon Challinger
//  Modified by Paul Riseborough
//

#include <AP_HAL/AP_HAL.h>
#include "AP_RollController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] = {
	// @Param: TCONST
	// @DisplayName: Roll Time Constant
	// @Description: Time constant in seconds from demanded to achieved roll angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
	// @Range: 0.4 1.0
	// @Units: s
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_RollController, gains.tau,       0.5f),

	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: Proportional gain from roll angle demands to ailerons. Higher values allow more servo response but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("P",        1, AP_RollController, gains.P,        1.0f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: Damping gain from roll acceleration to ailerons. Higher values reduce rolling in turbulence, but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.2
	// @Increment: 0.01
	// @User: User
	AP_GROUPINFO("D",        2, AP_RollController, gains.D,        0.08f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: Integrator gain from long-term roll angle offsets to ailerons. Higher values "trim" out offsets faster but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 1.0
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("I",        3, AP_RollController, gains.I,        0.3f),

	// @Param: RMAX
	// @DisplayName: Maximum Roll Rate
	// @Description: Maximum roll rate that the roll controller demands (degrees/sec) in ACRO mode.
	// @Range: 0 180
	// @Units: deg/s
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX",   4, AP_RollController, gains.rmax,       0),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of roll integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 3000 allows trim of up to 2/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      5, AP_RollController, gains.imax,        3000),

	// @Param: FF
	// @DisplayName: Feed forward Gain
	// @Description: Gain from demanded rate to aileron output. 
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("FF",        6, AP_RollController, gains.FF,          0.0f),

	AP_GROUPEND
};


/*
  internal rate controller, called by attitude and rate controller
  public functions
*/
int32_t AP_RollController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator)
{
	/// Param for MSP ///
	// gains.P = 1.116;
	// gains.I = 0.175;
	// gains.D = 0.0812;
	
	/// Param for 155 3x tail no engine ///
	// gains.P = 1.33;
	// gains.I = 0.25;
	// gains.D = 0.156;
	
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	// Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    // No conversion is required for K_D
	float ki_rate = gains.I * gains.tau;
    float eas2tas = _ahrs.get_EAS2TAS();
	float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau  - gains.D , 0) / eas2tas;
    //float k_ff = gains.FF / eas2tas;
	// float delta_time    = (float)dt * 0.001f;
	float delta_time    = 0.05;
    // Get body rate vector (radians/sec)
	float omega_x = _ahrs.get_gyro().x;
	
	// Calculate the roll rate error (deg/sec) and apply gain scaler
    float achieved_rate = ToDeg(omega_x);
	float rate_error = (desired_rate - achieved_rate) * scaler;
	
	// Get an airspeed estimate - default to zero if none available
	float aspeed;
	if (!_ahrs.airspeed_estimate(aspeed)) {
        aspeed = 0.0f;
    }

	// // Multiply roll rate error by _ki_rate, apply scaler and integrate
	// // Scaler is applied before integrator so that integrator state relates directly to aileron deflection
	// // This means aileron trim offset doesn't change as the value of scaler changes with airspeed
	// // Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	// if (!disable_integrator && ki_rate > 0) {
		// //only integrate if gain and time step are positive and airspeed above min value.
		// if (dt > 0 && aspeed > float(aparm.airspeed_min)) {
		    // float integrator_delta = rate_error * ki_rate * delta_time * scaler;
			// // prevent the integrator from increasing if surface defln demand is above the upper limit
			// if (_last_out < -45) {
                // integrator_delta = MAX(integrator_delta , 0);
            // } else if (_last_out > 45) {
                // // prevent the integrator from decreasing if surface defln demand  is below the lower limit
                 // integrator_delta = MIN(integrator_delta, 0);
            // }
			// _pid_info.I += integrator_delta;
		// }
	// } else {
		// _pid_info.I = 0;
	// }
	
		// Multiply roll rate error by _ki_rate, apply scaler and integrate
	// Scaler is applied before integrator so that integrator state relates directly to aileron deflection
	// This means aileron trim offset doesn't change as the value of scaler changes with airspeed
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	// if (!disable_integrator && ki_rate > 0) {
		// only integrate if gain and time step are positive and airspeed above min value.
		// if (dt > 0 && aspeed > float(aparm.airspeed_min)) {
		    float integrator_delta = rate_error * ki_rate * delta_time * scaler;
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) {
                integrator_delta = MAX(integrator_delta , 0);
            } else if (_last_out > 45) {
                // prevent the integrator from decreasing if surface defln demand  is below the lower limit
                 integrator_delta = MIN(integrator_delta, 0);
            }
			
			if(disable_integrator){
				_pid_info.I = 0;
			}
			else{
				_pid_info.I += integrator_delta;
			}
		// }
	// } else {
		// _pid_info.I = 0;
	// }
	
	
    // Scale the integration limit
    // float intLimScaled = gains.imax * 0.01f;
	float intLimScaled = 3000 * 0.01f;

    // Constrain the integrator state
    _pid_info.I = constrain_float(_pid_info.I, -intLimScaled, intLimScaled);
	
	// Calculate the demanded control surface deflection
	// Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
	// path, but want a 1/speed^2 scaler applied to the rate error path. 
	// This is because acceleration scales with speed^2, but rate scales with speed.
    _pid_info.D = rate_error * gains.D * scaler;
    _pid_info.P = desired_rate * kp_ff * scaler;
    // _pid_info.FF = desired_rate * k_ff * scaler;
    _pid_info.target = desired_rate;
    _pid_info.actual = achieved_rate;

	// _last_out = _pid_info.FF + _pid_info.P + _pid_info.D;
	_last_out = _pid_info.P + _pid_info.D;

    if (autotune.running && aspeed > aparm.airspeed_min) {
        // let autotune have a go at the values 
        // Note that we don't pass the integrator component so we get
        // a better idea of how much the base PD controller
        // contributed
        autotune.update(desired_rate, achieved_rate, _last_out);
    }

	_last_out += _pid_info.I;
	
	pid_roll_P 			= _pid_info.P;
	pid_roll_I	 		= _pid_info.I;
	pid_roll_D 			= _pid_info.D;
	pid_roll_ailerons	= constrain_float(_last_out, -45, 45);
	
	//_pid_info.FF = _last_out;
	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}


/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) desired roll rate in degrees/sec
 2) control gain scaler = scaling_speed / aspeed
*/
int32_t AP_RollController::get_rate_out(float desired_rate, float scaler)
{
    return _get_rate_out(desired_rate, scaler, false);
}

/*
 Function returns an equivalent aileron deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded bank angle in centi-degrees
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
*/
int32_t AP_RollController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
	_pid_info.FF = angle_err;
	//angle_err = gains.imax - angle_err - 3000;
    if (gains.tau < 0.1f) {
        gains.tau.set(0.1f);
    }
	pid_roll_error = angle_err*0.01f;
	// Calculate the desired roll rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f / gains.tau;

    // Limit the demanded roll rate
    if (gains.rmax && desired_rate < -gains.rmax) {
        desired_rate = - gains.rmax;
    } else if (gains.rmax && desired_rate > gains.rmax) {
        desired_rate = gains.rmax;
    }

    return _get_rate_out(desired_rate, scaler, disable_integrator);
}

void AP_RollController::reset_I()
{
	//_pid_info.I = 0;
}


float AP_RollController::get_pid_roll(float roll_pe, int test_r)
{
    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t;
    float output            = 0;
    float delta_time;
	
	/// Param for MSP ///		  				//10.07		    //16.02     //15.02 2fl
	float    kp_roll 			= 0.427;		//0.327;    	//0.527;    //0.327     //0.177;//(float)gains.P;   //0.177;    //0.3573;		//1.232;		//1.42			//1.02;			//1.32;
    float    ki_roll 			= 0.1717;		//0.0717;    	//0.267;    //0.267     //0.167     //(float)gains.I;   //0.167;    //0.1242;		//0.122			//0.122			//0.162;		//0.162;
    float    kd_roll 			= 0.0257;		//0.0157;   	//0.0197;   //0.0097    //(float)gains.D;   //0.0087;   //0.00647;		//0.00872		//0.042			//0.042;		//0.072;
    float    imax 				= 30;       	//30;       	//30        //30;		//30			//30			//30;			//30;

	// _roll_error_pe = roll_pe	= constrain_float(roll_pe, -30, 30);// - gains.imax * 0.01f + 15;
    _roll_error_pe = roll_pe;//	= constrain_float(roll_pe, -30, 30);

    if (_last_t == 0 || dt > 1000) {
        dt = 0;

		// if this PID hasn't been used for a full second then zero
		// the intergator term. This prevents I buildup from a
		// previous fight mode from causing a massive return before
		// the integrator gets a chance to correct itself
		_integrator = 0;
		// we use NAN (Not A Number) to indicate that the last 
		// derivative value is not valid
		// _last_derivative = NAN;
		_pid_info.I = 0;
    }
    _last_t = tnow;

    delta_time = (float)dt / 1000.0f;
    // delta_time = 0.05;

    // Compute proportional component
    _pid_info.P = constrain_float(_roll_error_pe * kp_roll, -30, 45);
    output += _pid_info.P;

    // Compute integral component if time has elapsed
    if ((fabsf(ki_roll) > 0) && (dt > 0)) {
        _integrator             += (_roll_error_pe * ki_roll) * delta_time;
        if (_integrator < -imax) {
            _integrator = -imax;
        } else if (_integrator > imax) {
            _integrator = imax;
        }
        // _pid_info.I = _integrator;
        // output                          += _integrator;
    }
	_pid_info.I = _integrator;
	output                          += _integrator;

	output = constrain_float(output, -45, 45);
    float omega_y = _ahrs.get_gyro().x;
	_pid_info.D = -kd_roll*ToDeg(omega_y);
    output += _pid_info.D;
    _pid_info.target = _roll_error_pe;
	//_pid_info.FF = -output;
	//_pid_info.FF = roll_pe;
	

    pid_roll_error      = roll_pe;
    pid_roll_P 			= _pid_info.P;
	pid_roll_I	 		= _pid_info.I;
	pid_roll_D 			= _pid_info.D;
	pid_roll_ailerons	= constrain_float(output, -45, 45);
    
    if(test_r == 0){
        _pid_info.P = 0;
        _pid_info.I = 0;
        _pid_info.D = 0;
        _integrator = 0;
    }
	
    return output*100;
    // return 0;
}



float AP_RollController::XPL_get_pid_roll(float roll_pe, int XPL_w_x)
{

    float output            = 0;
    float delta_time;
	
	/// Param for MSP ///		  				//10.07		    //16.02     //15.02 2fl
	float    kp_roll 			= gains.P;  //0.627;		//0.327;    	//0.527;    //0.327     //0.177;//(float)gains.P;   //0.177;    //0.3573;		//1.232;		//1.42			//1.02;			//1.32;
    float    ki_roll 			= gains.I;  //0.0217;		//0.0717;    	//0.267;    //0.267     //0.167     //(float)gains.I;   //0.167;    //0.1242;		//0.122			//0.122			//0.162;		//0.162;
    float    kd_roll 			= gains.D;  //0.0257;		//0.0157;   	//0.0197;   //0.0097    //(float)gains.D;   //0.0087;   //0.00647;		//0.00872		//0.042			//0.042;		//0.072;
    float    imax 				= 30;       	//30;       	//30        //30;		//30			//30			//30;			//30;

	_roll_error_pe = constrain_float(gains.imax*0.01f - roll_pe, -10, 10);// - gains.imax * 0.01f + 15;


    delta_time = 0.02;

    // Compute proportional component
    _pid_info.P = constrain_float(_roll_error_pe * kp_roll, -45, 45);
    output += _pid_info.P;

    // Compute integral component if time has elapsed

        _XPL_integrator             += (_roll_error_pe * ki_roll) * delta_time;
        if (_XPL_integrator < -imax) {
            _XPL_integrator = -imax;
        } else if (_XPL_integrator > imax) {
            _XPL_integrator = imax;
        }
        // _pid_info.I = _XPL_integrator;
        // output                          += _integrator;

	_pid_info.I = _XPL_integrator;
	output                          += _XPL_integrator;

	output = constrain_float(output, -45, 45);
	_pid_info.D = -kd_roll*XPL_w_x;
    output += _pid_info.D;
    _pid_info.target = _roll_error_pe;
	//_pid_info.FF = -output;
	//_pid_info.FF = roll_pe;
	

    pid_roll_error      = roll_pe;
    pid_roll_P 			= _pid_info.P;
	pid_roll_I	 		= _pid_info.I;
	pid_roll_D 			= _pid_info.D;
	pid_roll_ailerons	= constrain_float(output, -45, 45);
    
	
    return constrain_float(output/90, -0.5, 0.5);
    // return 0;
}

