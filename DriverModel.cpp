/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that simply sends back VISSIM's suggestions to VISSIM.    */
/*                                                                          */
/*  Version of 2010-03-02                                   Lukas Kautzsch  */
/*==========================================================================*/

#include "DriverModel.h"
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <math.h>
#include <set>
#include <stdio.h>
#include <string>


/*==========================================================================*/
/*Default Parameters*/

double  desired_acceleration = 3.5;
double  desired_lane_angle   = 0.0;
long    active_lane_change   = 0;
long    rel_target_lane      = 0;
double  desired_velocity     = 13.9;
long    turning_indicator    = 0;
long    vehicle_color        = RGB(225,225,225);

/*==========================================================================*/
/*Create Variables*/

double current_time;
long current_vehicle;
double  current_velocity;
double  current_acceleration;
double  vehicle_length;
double  vehicle_x_coord;
double  vehicle_y_coord;
long  vehicle_type;
int vehicle_current_link;
double  gross_distance;
double  speed_difference;
double  ahead_vehicle_acceleration;
double  signal_distance;
double  s_star;
double  distance = 0;
double stopline_x_coord;
double stopline_y_coord;
double car_following_acceleration;
double ahead_vehicle;
double phase;
//long category;


static int vehicle_count = 0;
static int veh = -1;
static int vehicle_to_stop = 0;

/*==========================================================================*/
/* IDM parameters and equation*/

double a = 1.5;                            // maximum acceleration, constant a (m/s^2)
double b = 3;                            // maximum deceleration, constant b (m/s^2)
double IDM_desired_velocity = 15;        // desired velocity uesd in IDM (m/s)
double v0 = IDM_desired_velocity;
double jam_distance = 0.6;                 // minimum gross distance (m)
double S0 = jam_distance;
double T = 0.6;                            // safe-time headway (s)

double S_star(double v, double v_delta)
{
	double S_star = S0 + (v * T) + ((v * v_delta) / (float)(2 * sqrt(a*b)));
	return S_star;
}



double euclid_dist(double x1, double y1, double x2, double y2)
{
	double x_diff = x2 - x1;
	double y_diff = y2 - y1;
	double euclid_dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
	return euclid_dist;
}


/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule, 
                       DWORD   ul_reason_for_call, 
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }
  return TRUE;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, to <long_value>, <double_value> or            */
  /* <*string_value> (object and value selection depending on <type>).    */
  /* Return value is 1 on success, otherwise 0.                           */
	
  switch (type) {
    case DRIVER_DATA_PATH                   :
    case DRIVER_DATA_TIMESTEP               :
    case DRIVER_DATA_TIME                   :
		current_time = double_value;
		return 1;
    case DRIVER_DATA_VEH_ID                 :
		current_vehicle = long_value;
		return 1;
    case DRIVER_DATA_VEH_LANE               :
    case DRIVER_DATA_VEH_ODOMETER           :
    case DRIVER_DATA_VEH_LANE_ANGLE         :
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
    case DRIVER_DATA_VEH_VELOCITY           :
		current_velocity = double_value;
		return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
		current_acceleration = double_value;
		return 1;
    case DRIVER_DATA_VEH_LENGTH             :
		vehicle_length = double_value;
		return 1;
    case DRIVER_DATA_VEH_WIDTH              :
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
		return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
		turning_indicator = long_value;
		return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
		//category = long_value;
		//return 1;
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
		return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
		desired_velocity = double_value;
		return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
		vehicle_x_coord = double_value;
		return 1;
    case DRIVER_DATA_VEH_Y_COORDINATE       :
		vehicle_y_coord = double_value;
		return 1;
    case DRIVER_DATA_VEH_TYPE               :
		vehicle_type = long_value;
		return 1;
    case DRIVER_DATA_VEH_COLOR              :
		vehicle_color = long_value;
		return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
		vehicle_current_link = long_value;
		return 1; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
                /* Must return 1 if these messages are to be sent from VISSIM!         */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
    case DRIVER_DATA_NVEH_ID                :
		if (index1 == 0 && index2 == 1) {
			ahead_vehicle = long_value;
		}
		//adj_vehicle[0][1] = long_value;
		return 1;
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
    case DRIVER_DATA_NVEH_DISTANCE          :
		if (index1 == 0 && index2 == 1)
		{
			gross_distance = double_value;
		}
		return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
		if (index1 == 0 && index2 == 1)
		{
			speed_difference = double_value;
		}
		return 1;
    case DRIVER_DATA_NVEH_ACCELERATION      :
		if (index1 == 0 && index2 == 1)
		{
			ahead_vehicle_acceleration = double_value;
		}
		return 1;
    case DRIVER_DATA_NVEH_LENGTH            :
    case DRIVER_DATA_NVEH_WIDTH             :
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
    case DRIVER_DATA_NO_OF_LANES            :
    case DRIVER_DATA_LANE_WIDTH             :
    case DRIVER_DATA_LANE_END_DISTANCE      :
    case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
    case DRIVER_DATA_SIGNAL_DISTANCE        :
		signal_distance = double_value;
		return 1;
    case DRIVER_DATA_SIGNAL_STATE           :
		phase = long_value;
		return 1;
    case DRIVER_DATA_SIGNAL_STATE_START     :
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
		return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
		desired_acceleration = double_value;
		return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
		desired_lane_angle = double_value;
		return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
		active_lane_change = long_value;
		return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
		rel_target_lane = long_value;
		return 1;
    default :
      return 0;
  }
}


/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   *long_value,
                                           double *double_value,
                                           char   **string_value)
{
  /* Gets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, and writes that value to <*double_value>,     */
  /* <*float_value> or <**string_value> (object and value selection       */
  /* depending on <type>).                                                */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_STATUS :
      *long_value = 0;
      return 1;
	case DRIVER_DATA_TIME:
		*double_value = current_time;
		 return 1;
	case DRIVER_DATA_VEH_ID:
		*long_value = current_vehicle;
		return 1;
	case DRIVER_DATA_VEH_VELOCITY:
		*double_value = current_velocity;
		return 1;
	case DRIVER_DATA_VEH_ACCELERATION:
		*double_value = current_acceleration;
		return 1;
	case DRIVER_DATA_VEH_LENGTH:
		*double_value = vehicle_length;
		return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
      *long_value = turning_indicator;
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      *double_value = desired_velocity;
      return 1;
	case DRIVER_DATA_VEH_X_COORDINATE:
		*double_value = vehicle_x_coord;
		return 1;
	case DRIVER_DATA_VEH_Y_COORDINATE:
		*double_value = vehicle_y_coord;
		return 1;
	case DRIVER_DATA_VEH_TYPE:
		*long_value = vehicle_type;
		return 1;
    case DRIVER_DATA_VEH_COLOR :
      *long_value = vehicle_color;
      return 1;
	case DRIVER_DATA_VEH_CURRENT_LINK:
		*long_value = vehicle_current_link;
		return 1;
	//case DRIVER_DATA_NVEH_ID:
		//*long_value = adj_vehicle[0][1];
		//return 1;
	case DRIVER_DATA_NVEH_DISTANCE:
		*double_value = gross_distance;
		return 1;
	case DRIVER_DATA_NVEH_REL_VELOCITY:
		*double_value = speed_difference;
		return 1;
	//case DRIVER_DATA_NVEH_ACCELERATION:
	//	*double_value = ahead_vehicle_acceleration;
	//	return 1;
	case DRIVER_DATA_SIGNAL_DISTANCE:
		*double_value = signal_distance;
		return 1;
    case DRIVER_DATA_WANTS_SUGGESTION :
      *long_value = 1;
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
      *double_value = desired_acceleration;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
      *double_value = desired_lane_angle;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      *long_value = active_lane_change;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
      *long_value = rel_target_lane;
      return 1;
    case DRIVER_DATA_SIMPLE_LANECHANGE :
      *long_value = 1;
      return 1;
    default :
      return 0;
  }
}



/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (long number)
{
  /* Executes the command <number> if that is available in the driver */
  /* module. Return value is 1 on success, otherwise 0.               */
  
  switch (number) {
    case DRIVER_COMMAND_INIT :
      return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
      return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
      return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
		if (vehicle_type == 700) 
		{
			if(current_vehicle != vehicle_to_stop ){ 

				double S = gross_distance - vehicle_length;
				double v = current_velocity;
				double v_delta = speed_difference;
				double s_star = S_star(current_velocity, speed_difference);
				
				car_following_acceleration = a * (1 - pow((v / (float)(v0)), 4) - pow((s_star / (float)S), 2));

				if (gross_distance > jam_distance)
				{
					desired_acceleration = car_following_acceleration;// setting the IDM acceleration to the desired acceleration if gross distance is greater than the min gross distance
				}
			}

			if (ahead_vehicle < 0) {
				vehicle_to_stop = current_vehicle;
			}
		}
      return 1;
    default :
      return 0;
  }
}

/*==========================================================================*/
/*  Ende of DriverModel.cpp                                                 */
/*==========================================================================*/
