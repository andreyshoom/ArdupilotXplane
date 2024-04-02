#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

class AP_PitchController {
public:
    AP_PitchController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms)
        : aparm(parms)
        , autotune(gains, AP_AutoTune::AUTOTUNE_PITCH, parms)
        , _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_PitchController(const AP_PitchController &other) = delete;
    AP_PitchController &operator=(const AP_PitchController&) = delete;

	int32_t get_rate_out(float desired_rate, float scaler);
	int32_t get_servo_out(int32_t angle_err, float scaler, bool disable_integrator);
	//int32_t get_servo_out_alt(float pitch_ha, float alt_err_ha, float current_alt_ha);
	int32_t get_servo_out_pitch(float pitch_pe, float XPL_w_y);
	
	float pid_pitch_P 		= 0;
	float pid_pitch_I 		= 0;
	float pid_pitch_D 		= 0;
	float pid_pitch_error 	= 0;
	float pid_pitch_elevator = 0;
    
    
    float get_pid_alt(float alt_err_ha, float current_alt_ha, float pitch_ha, int cl_start);
	float get_pid_pitch_dive_gps(float pitch_pe, float error_with_pitch, float dive_ang_sina, float yat, float altitude_now);
    
    // float XPL_get_pid_alt(float alt_err_ha, float current_alt_ha, float pitch_ha, int cl_start);
    // float XPL_get_pid_pitch_dive_gps(float pitch_pe, float error_with_pitch, float dive_ang_sina, float yat, float altitude_now);
    float XPL_get_pid_pitch(float pitch_pe, float error_with_pitch);
    float XPL_get_pid_alt(float alt_err_ha, float current_alt_ha, float pitch_ha, float XPL_w_y);
    
	void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I() {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }
    
    void autotune_start(void) { autotune.start(); }
    void autotune_restore(void) { autotune.stop(); }

    const AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

    AP_Float &kP(void) { return gains.P; }
    AP_Float &kI(void) { return gains.I; }
    AP_Float &kD(void) { return gains.D; }
    AP_Float &kFF(void) { return gains.FF; }

private:
    const AP_Vehicle::FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune autotune;
	AP_Int16 _max_rate_neg;
	AP_Float _roll_ff;
	uint32_t _last_t;
	float _last_out;
	
	float tttret = 0;
    float _out_ant;
    float _last_alt;
    float           _integrator;///< integrator value
    //float           _integrator_alt;///< integrator value
    //float           _integrator_dive_gps;///< integrator value
    float           _last_error;///< last error for derivative
    float           _last_derivative;///< last derivative for low-pass filter
    static const uint8_t        _fCut = 20;
	float _pitch_error_pe;
    
    float           _XPL_integrator = 0;///< integrator value
    float           _XPL_integrator2 = 0;
	
	
    AP_Logger::PID_Info _pid_info;

	int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed);
    float   _get_coordination_rate_offset(float &aspeed, bool &inverted) const;
	// float get_pid_alt(float alt_err_ha, float current_alt_ha, float pitch_ha);
	float get_pid_pitch(float pitch_pe, float error_with_pitch);
    
	
	AP_AHRS &_ahrs;
	
	//float get_present();
	
};
