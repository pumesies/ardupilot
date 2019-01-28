/*
  plane specific AP_LandingSafety class
 */

#include "Plane.h"

// Constructor
AP_LandingSafety_Plane::AP_LandingSafety_Plane(const AP_AHRS &ahrs, const AP_Landing &landing) :
    AP_LandingSafety(ahrs, landing)
{}

float AP_LandingSafety_Plane::get_ground_altitude()
{
	return plane.relative_ground_altitude(true);
}


