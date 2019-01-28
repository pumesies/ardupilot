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

/*
    AP_LandingSafety.cpp

*/
#include <AP_HAL/AP_HAL.h>
#include "AP_LandingSafety.h"
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_LandingSafety::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Landing Safety Features
    // @Description: This enables the landing safety system. If this is set to zero (disable) then all the other options have no effect
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE",       0, AP_LandingSafety, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: LANDING_LIGHTS_SERVO
    // @DisplayName: Manual Pin
    // @Description: Servo number of the channel that turns the landing lights on or off
    // @User: Advanced
    AP_GROUPINFO("L_LGHT_SRV",     1, AP_LandingSafety, _lights_servo,    14),

    // @Param: CAMERA_RETRACT_SERVO
    // @DisplayName: Camera retract servo number
    // @Description: Servo number of the servo that retracts the camera ball.
    // @User: Advanced
    AP_GROUPINFO("CAM_RTR_SRV",      2, AP_LandingSafety, _retract_servo, 12),

    // @Param: CAMERA_RETRACT_PWM
    // @DisplayName: Camera retract pwm value
    // @Description: PWM value to output to retract the camera
    // @User: Advanced
    AP_GROUPINFO("CAM_RTR_PWM",    3, AP_LandingSafety, _retract_pwm, 2000),

    // @Param: LANDING_LIGHTS_PWM
    // @DisplayName: Landing lights pwm value
    // @Description: PWM value to output to switch the landing lights on
    // @User: Advanced
    AP_GROUPINFO("L_LGHT_PWM", 4, AP_LandingSafety, _lights_on_pwm, 2000),

    // @Param: AGL_LIMIT
    // @DisplayName: AGL limit
    // @Description: This sets the AGL (above ground level) altitude limit. Below this limit, the camera ball will automatically retract and the landing lights will switch on.
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("AGL_LIMIT",   5, AP_LandingSafety, _agl_limit,    10),

    AP_GROUPEND
};

// check for Failsafe conditions. This is called at 10Hz by the main
// ArduPlane code
void
AP_LandingSafety::check()
{    
    if (!_enable) {
        return;
    }
    
    // check for altitude breach
    if (check_altlimit()) {
        uint16_t value;
        if (SRV_Channels::get_output_pwm(SRV_Channel::k_cameraretract, value) && value !=_retract_pwm) {
            gcs().send_text(MAV_SEVERITY_INFO, "Altitude limit reached. Auto retract camera");
        }
        SRV_Channels::set_output_pwm(SRV_Channel::k_landinglights, _lights_on_pwm);
        SRV_Channels::set_output_pwm(SRV_Channel::k_cameraretract, _retract_pwm);
        
    }
    
    // check for landing mode
    //enum control_mode mode = afs_mode();
    
    // check if RC termination is enabled
    // check for RC failure in manual mode or RC failure when AFS_RC_MANUAL is 0
    if (_landing.is_on_approach() || _landing.is_flaring()) { //TODO replace mode
        uint16_t value;
        if (SRV_Channels::get_output_pwm(SRV_Channel::k_cameraretract, value) && value !=_retract_pwm) {
            gcs().send_text(MAV_SEVERITY_INFO, "Landing mode detected. Auto retract camera");
        }
        SRV_Channels::set_output_pwm(SRV_Channel::k_landinglights, _lights_on_pwm);
        SRV_Channels::set_output_pwm(SRV_Channel::k_cameraretract, _retract_pwm);
    }
    
}


// check for altitude limit breach
bool
AP_LandingSafety::check_altlimit(void)
{    
    if (!_enable) {
        return false;
    }
    if (_agl_limit == 0 ) {
        // no limit set
        return false;
    }

    float veh_alt = get_ground_altitude();

    //_ahrs.get_relative_position_D_home(veh_alt);
    
    if (veh_alt < _agl_limit) {
        // ahrs altitude breach
        return true;
    }
    

    // exit immediately if no rangefinder object
    //const RangeFinder *rngfnd = frontend.get_rangefinder();
    //if (rngfnd == nullptr) {
    //    return false;
    //}

    

    // look through all rangefinders
    /*for (uint8_t i=0; i < rngfnd->num_sensors(); i++) {
        AP_RangeFinder_Backend *sensor = rngfnd->get_backend(i);
        if (sensor == nullptr) {
            continue;
        }
        if (sensor->has_data()) {
            // check for horizontal range finders
            if ((sensor->orientation() == ROTATION_PITCH_270) && (sensor->status() == RangeFinder::RangeFinder_Good)) {
                if (sensor->distance_cm() * 0.01f < _agl_limit) {
                    return true;
                }
                
            }

        }
    }*/

    // all OK
    return false;
}


