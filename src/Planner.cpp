//
// Created by Sachit Vithaldas on 29/07/17.
//

#include "Planner.h"
#include "util.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

static double const MPH2MS = 1600.f / 3600.f;

S_PlannerResult Planner::GetPath( T_MapPoints const &map_waypoints_x, T_MapPoints const &map_waypoints_y,
                                  T_MapPoints const &map_waypoints_s, T_MapPoints const &map_waypoints_dx,
                                  T_MapPoints const &map_waypoints_dy )
{
	S_PlannerResult retVal;

	double const dist_inc = MPH2MS * 49 * 0.02; // 50 MPH speed in 0.02 sec intervals
	for ( int i = 0; i < 50; i++ )
	{
		double const nextS = m_Car_s + dist_inc;
		double const nextD = m_Car_d;
		auto const XY = getXY( nextS, nextD, map_waypoints_s, map_waypoints_x, map_waypoints_y );
		retVal.m_X.emplace_back( XY[ 0 ] );
		retVal.m_Y.emplace_back( XY[ 1 ] );
	}

//	static double const MAX_SPEED = 50.f * MPH2MS;
//	static double const MAX_ACCEL = 10.f * MPH2MS;
//
//	double const carSpeedMS = m_Car_speed * MPH2MS;
//	double const T = 1; // second
//	double const target_speed = min(MAX_SPEED, carSpeedMS + MAX_ACCEL);
//
//	//from v = u + at
//	double const resultAccel = (target_speed - carSpeedMS) / T;
//
//	auto const size =  m_Previous_path_x.size();
//	retVal.m_X = m_Previous_path_x;
//	retVal.m_Y = m_Previous_path_y;
//
//	double lastS(m_Car_s);
//	if(size > 0)
//	{
//		auto const lastSD = getFrenet(m_Previous_path_x.back(), m_Previous_path_y.back(), m_Car_yaw, map_waypoints_x, map_waypoints_y);
//		lastS = lastSD[0];
//	}
//
//	for(auto i = 0; i <= (51 - size); i++)
//	{
//		double const t = 0.02 * i;
//		double const nextS = lastS + carSpeedMS * t + 0.5 * resultAccel * t * t;
//		double const nextD = m_Car_d;
//		auto const XY = getXY( nextS, nextD, map_waypoints_s, map_waypoints_x, map_waypoints_y );
//		retVal.m_X.emplace_back( XY[ 0 ] );
//		retVal.m_Y.emplace_back( XY[ 1 ] );
//	}


	T_JMTParams start{m_Car_s, m_Car_speed * MPH2MS, m_Car_acceleration * MPH2MS};
	T_JMTParams end{m_Car_s + 200.f, 50.f * MPH2MS, 0.0};
	double const T(10);

	auto const &PolynomialParameters = JMT( start, end, T );

	for ( int i = 0; i < 50; i++ )
	{
		double const deltaTime = i * 0.02;
		double const nextS = PolynomialParameters[ 0 ] +
		                     PolynomialParameters[ 1 ] * deltaTime +
		                     PolynomialParameters[ 2 ] * pow( deltaTime, 2 ) +
		                     PolynomialParameters[ 3 ] * pow( deltaTime, 3 ) +
		                     PolynomialParameters[ 4 ] * pow( deltaTime, 4 ) +
		                     PolynomialParameters[ 5 ] * pow( deltaTime, 5 );
		double const nextD = m_Car_d;
		auto const XY = getXY( nextS, nextD, map_waypoints_s, map_waypoints_x, map_waypoints_y );
		retVal.m_X.emplace_back( XY[ 0 ] );
		retVal.m_Y.emplace_back( XY[ 1 ] );
	}


	return retVal;
}

T_PolynomialParameters Planner::JMT(T_JMTParams const& start, T_JMTParams const& end, double const T)
{
	using namespace Eigen;
	/*
	Calculate the Jerk Minimizing Trajectory that connects the initial state
	to the final state in time T.

	INPUTS

	start - the vehicles start location given as a length three array
		corresponding to initial values of [s, s_dot, s_double_dot]

	end   - the desired end state for vehicle. Like "start" this is a
		length three array.

	T     - The duration, in seconds, over which this maneuver should occur.

	OUTPUT
	an array of length 6, each value corresponding to a coefficent in the polynomial
	s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

	EXAMPLE

	> JMT( [0, 10, 0], [10, 10, 0], 1)
	[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
	*/
	double const a1 = start[ 0 ];
	double const a2 = start[ 1 ];
	double const a3 = start[ 2 ] / 2;

	double const T2 = T * T;
	double const T3 = T2 * T;
	double const T4 = T3 * T;
	double const T5 = T4 * T;

	double const X1 = end[ 0 ] - (start[ 0 ] + (start[ 1 ] * T) + (start[ 2 ] * T2 / 2.0));
	double const X2 = end[ 1 ] - (start[ 1 ] + (start[ 2 ] * T));
	double const X3 = end[ 2 ] - start[ 2 ];

	MatrixXd orig( 3, 3 );
	orig << T3, T4, T5,
			3 * T2, 4 * T3, 5 * T4,
			6 * T, 12 * T2, 20 * T3;

	MatrixXd const inv = orig.inverse();
	VectorXd val( 3 );
	val << X1, X2, X3;

	VectorXd const result = inv * val;
	double const a4 = result[ 0 ];
	double const a5 = result[ 1 ];
	double const a6 = result[ 2 ];

	return {a1, a2, a3, a4, a5, a6};
}
