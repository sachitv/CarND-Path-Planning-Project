//
// Created by Sachit Vithaldas on 29/07/17.
//
#include "SensorFusionData.h"

#include <vector>
#include <array>

#pragma once
#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

typedef std::vector<double> T_PathPoints;
typedef std::vector<double> T_MapPoints;
typedef std::vector<S_SensorFusionData> T_SensorFusionDatas;
typedef std::vector<double> T_PolynomialParameters;
typedef std::array<double, 3> T_JMTParams;

struct S_PlannerResult
{
	T_PathPoints m_X;
	T_PathPoints m_Y;
};

class Planner
{
public:
	Planner( double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
	         double car_acceleration,
	         const T_PathPoints &previous_path_x, const T_PathPoints &previous_path_y, double end_path_s,
	         double end_path_d, const T_SensorFusionDatas &sensor_fusion )
			: m_Car_x( car_x )
			  , m_Car_y( car_y )
			  , m_Car_s( car_s )
			  , m_Car_d( car_d )
			  , m_Car_yaw( car_yaw )
			  , m_Car_speed( car_speed )
			  , m_Car_acceleration( car_acceleration )
			  , m_Previous_path_x( previous_path_x )
			  , m_Previous_path_y( previous_path_y )
			  , m_End_path_s( end_path_s )
			  , m_End_path_d( end_path_d )
			  , m_Sensor_fusion( sensor_fusion ) {}

	S_PlannerResult
	GetPath( T_MapPoints const &map_waypoints_x, T_MapPoints const &map_waypoints_y, T_MapPoints const &map_waypoints_s,
	         T_MapPoints const &map_waypoints_dx, T_MapPoints const &map_waypoints_dy );

	T_PolynomialParameters JMT( T_JMTParams const &start, T_JMTParams const &end, double const T );

private:
	double m_Car_x;
	double m_Car_y;
	double m_Car_s;
	double m_Car_d;
	double m_Car_yaw;
	double m_Car_speed;
	double m_Car_acceleration;
	T_PathPoints m_Previous_path_x;
	T_PathPoints m_Previous_path_y;
	double m_End_path_s;
	double m_End_path_d;
	T_SensorFusionDatas m_Sensor_fusion;


};


#endif //PATH_PLANNING_PLANNER_H
