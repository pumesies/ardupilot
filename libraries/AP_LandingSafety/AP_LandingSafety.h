#pragma once

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
  Outback Challenge Failsafe module

  Andrew Tridgell and CanberraUAV, August 2012
*/

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <inttypes.h>
#include <AP_Landing/AP_Landing.h>
#include <AP_AHRS/AP_AHRS.h>


class AP_LandingSafety
{
public:   
    // Constructor
    AP_LandingSafety(const AP_AHRS &ahrs, const AP_Landing &landing)
     :_ahrs(ahrs)
     , _landing(landing)
    {
        AP_Param::setup_object_defaults(this, var_info);        
    }

    virtual float get_ground_altitude(void) = 0;

    // check that everything is OK
    void check();
    

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];
        
protected:

    // reference to const AP_Landing to access it's params
    const AP_Landing &_landing;

    // reference to the AHRS object
    const AP_AHRS &_ahrs;
    
    AP_Int8 _enable;
    
    AP_Int32 _agl_limit;
    AP_Int32  _lights_on_pwm;
    AP_Int32  _retract_pwm;
    AP_Int8  _lights_servo;
    AP_Int8  _retract_servo;   
    
    bool check_altlimit(void);
};
