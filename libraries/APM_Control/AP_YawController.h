#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <cmath>

class AP_YawController {
public:
    AP_YawController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms)
        : aparm(parms)
        , _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _pid_info.target = 0;
        _pid_info.FF = 0;
        _pid_info.P = 0;
    }

    /* Do not allow copies */
    AP_YawController(const AP_YawController &other) = delete;
    AP_YawController &operator=(const AP_YawController&) = delete;
    
    float XPL_get_pid_speed(float XPL_pseed);

	int32_t get_servo_out(float scaler, bool disable_integrator);

	void reset_I();
	float get_pid_yaw(float pitch_pe, float yar_now);
	
	float pid_yaw_P 		= 0;
	float pid_yaw_I 		= 0;
	float pid_yaw_D 		= 0;
	float pid_yaw_error 	= 0;
	float pid_yaw_rudder 	= 0;

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I() {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }
    
	const AP_Logger::PID_Info& get_pid_info(void) const {return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_Vehicle::FixedWing &aparm;
	AP_Float _K_A;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Float _K_FF;
    AP_Int16 _imax;
	uint32_t _last_t;
	float _last_out;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _K_D_last;

	float _integrator;
    float _integrator_old;
    
    float _XPL_integrator;
    float _XPL_integrator_old;
	
	float _out_ant;
	float _yaw_error_pe;
	//float           _integrator;///< integrator value
	
	// float get_pid_yaw(float pitch_pe);

	AP_Logger::PID_Info _pid_info;

	AP_AHRS &_ahrs;
};
