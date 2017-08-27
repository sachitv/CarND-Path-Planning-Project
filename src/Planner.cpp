//
// Created by Sachit Vithaldas on 29/07/17.
//

#include "Planner.h"
#include "util.h"
#include "spline.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include <iostream>

static double const MPH2MS = 1600.f / 3600.f;

static double const MAX_VEL = 49.5f * MPH2MS;
static int const MAX_PATH_POINTS = 50;
static double const TIME_INTERVAL = 0.02;//sec
static double const MAX_DBL = numeric_limits<double>::max();

struct Point
{
	Point()
	{
		x = 0;
		y = 0;
	}

	Point( double const inX, double const inY )
	{
		x = inX;
		y = inY;
	}

	void print() const
	{
		cout << x << "," << y << endl;
	}

	double x;
	double y;
};

S_PlannerResult Planner::GetPath( T_MapPoints const &map_waypoints_x, T_MapPoints const &map_waypoints_y,
                                  T_MapPoints const &map_waypoints_s, T_MapPoints const &map_waypoints_dx,
                                  T_MapPoints const &map_waypoints_dy )
{
	S_PlannerResult retVal;

	size_t const previous_path_len = m_Previous_path_y.size(); //or x, whatever.

	//Checking other vehicles on the road
	static double targetSpeed = 0;
	bool slowDown( false );
	int const currentCarLane = (m_Car_d) / 4;
	static int targetLane = currentCarLane;
	static bool changingLane( false );
	double minSpeed = MAX_VEL;
	double minProjectedDistance = MAX_DBL;
	double minCurrentDistance = MAX_DBL;
	static double changeD = 0;
	static double targetD = currentCarLane * 4 + 2.f;

	double mainCarPredictedPosition = m_Car_s;
	if ( previous_path_len > 0 )
	{
		auto const lastPoint = getFrenet( m_Previous_path_x.back(), m_Previous_path_y.back(),
		                                  deg2rad( m_Car_yaw ), map_waypoints_x, map_waypoints_y );
		mainCarPredictedPosition = lastPoint[ 0 ];
	}

	//Check if it is safe to enter any of these lanes. By Default true.
	array<bool, 3> laneSafety = {true, true, true};
	array<double, 3> laneDistance = {MAX_DBL, MAX_DBL, MAX_DBL};

	for ( auto const &vehicleData : m_Sensor_fusion )
	{
		int const thisCarLane = (vehicleData.m_D / 4);
		double const thisCarS = vehicleData.m_S;
		double const thisCarSpeed = sqrt( pow( vehicleData.m_Vy, 2 ) + pow( vehicleData.m_Vx, 2 ));

		double const thisCarProjectedDistance = previous_path_len * TIME_INTERVAL * thisCarSpeed;

		double const projectedDistance = (thisCarS + thisCarProjectedDistance) - mainCarPredictedPosition;
		double const currentDistance = thisCarS - m_Car_s;

		if ( thisCarLane == currentCarLane )
		{
			//Car is in our lane

			static double const MIN_DIST = 30.0;
			if ( projectedDistance > 0 && projectedDistance < MIN_DIST ) //meters
			{
				//This car is ahead of us, slow our speed
				if ( thisCarSpeed < MAX_VEL )
				{
					slowDown = true;
					minSpeed = min( thisCarSpeed, minSpeed );
					minProjectedDistance = min( minProjectedDistance, projectedDistance );
					minCurrentDistance = min( minCurrentDistance, currentDistance );
				}
			}
		}
		else
		{
			static double const MIN_BACK_DIST = -10.0;
			static double const MIN_FORWARD_DIST = 50.0;

			if ((currentDistance > MIN_BACK_DIST) && (currentDistance < MIN_FORWARD_DIST) &&
			    (projectedDistance < MIN_FORWARD_DIST))
			{
				laneSafety[ thisCarLane ] = false;
			}
			laneDistance[ thisCarLane ] = min( laneDistance[ thisCarLane ], projectedDistance );

			//There is an edge case I suppose when s cycles back to 0. I am doing this to fix that.
			double const thisCarX = vehicleData.m_X;
			double const thisCarY = vehicleData.m_Y;

			double const XDiff = m_Car_x - thisCarX;
			double const YDiff = m_Car_y - thisCarY;

			double const xydistance = sqrt(pow(XDiff, 2) + pow(YDiff, 2));

			if(xydistance < MIN_BACK_DIST)
			{
				laneSafety[ thisCarLane ] = false;
				laneDistance[ thisCarLane ] = min( laneDistance[ thisCarLane ], xydistance );
			}
		}
	}

	double const minPermittedSpeedInLane = minSpeed - 5.0;
	if ( slowDown && !changingLane )
	{
		//Check available lanes on the sides
		int const leftLane = currentCarLane - 1;
		int const rightLane = currentCarLane + 1;
		if ( leftLane >= 0 && laneSafety[ leftLane ] )
		{
			//If the left lane is empty, choose it.
			targetLane = leftLane;
		}
		if ( rightLane <= 2 && laneSafety[ rightLane ] )
		{
			//If the right lane is also empty, then check to see which is more open.
			if ( targetLane == leftLane )
			{
				//Go in the right lane if it is more open than the left lane
				if ( laneDistance[ rightLane ] > laneDistance[ leftLane ] )
				{
					targetLane = rightLane;
				}
				//else, go to the left lane itself.
			}
			else
			{
				targetLane = rightLane;
			}
		}

		//If there's less than NO_CHANGE_DISTANCE meters to the car ahead, don't change lanes
		static double const NO_CHANGE_DISTANCE = 20;
		static double const SLOW_DOWN_DIST = 30;

		if ( minProjectedDistance < NO_CHANGE_DISTANCE )
		{
			targetLane = currentCarLane;

			if ( targetSpeed > minPermittedSpeedInLane )
			{
				static double const HEAVY_BRAKE_AMOUNT = 0.3 * MPH2MS;
				targetSpeed -= HEAVY_BRAKE_AMOUNT;
			}
		}
		else if ( minProjectedDistance < SLOW_DOWN_DIST )
		{
			if ( targetSpeed > minPermittedSpeedInLane )
			{
				static double const LIGHT_BRAKE_AMOUNT = 0.12 * MPH2MS;
				targetSpeed -= LIGHT_BRAKE_AMOUNT;
			}
		}

		if ( targetLane != currentCarLane )
		{
			changingLane = true;
			changeD = ((double) (targetLane - currentCarLane)) / 10.f;
		}
	}
	else if ( slowDown && changingLane )
	{
		static double const SLOW_DOWN_DIST = 5;

		if ( minCurrentDistance > 0 && minCurrentDistance < SLOW_DOWN_DIST )
		{
			targetLane = currentCarLane;

			if ( targetSpeed > minPermittedSpeedInLane )
			{
				static double const HEAVY_BRAKE_AMOUNT = 0.2 * MPH2MS;
				targetSpeed -= HEAVY_BRAKE_AMOUNT;
			}
		}
	}
	else if ( targetSpeed < MAX_VEL )
	{
		static double const HEAVY_ACCEL_AMOUNT = 0.2 * MPH2MS;
		targetSpeed += HEAVY_ACCEL_AMOUNT;
	}

	double const targetLaneCenter = targetLane * 4 + 2.f;
	static double const THRESHOLD = 0.1;

	targetD = targetD + changeD;
	if ( changeD > 0 )
	{
		targetD = min( targetLaneCenter, targetD );
	}
	else
	{
		targetD = max( targetLaneCenter, targetD );
	}

	if ( abs( m_Car_d - targetLaneCenter ) < THRESHOLD )
	{
		changingLane = false;
		targetD = targetLaneCenter;
		changeD = 0.0;
	}

	targetSpeed = max( 0.0, min( MAX_VEL, targetSpeed ));

	vector<Point> pts;

	Point ref;
	double ref_yaw;

	if ( previous_path_len < 2 )
	{
		ref = Point( m_Car_x, m_Car_y );
		ref_yaw = deg2rad( m_Car_yaw );

		//If it's almost empty
		double const prev_x = m_Car_x - cos( ref_yaw );
		double const prev_y = m_Car_y - sin( ref_yaw );

		Point const prev( prev_x, prev_y );

		pts.emplace_back( prev );
		pts.emplace_back( ref );
	}
	else
	{
		double const prev_x = m_Previous_path_x[ previous_path_len - 2 ];
		double const prev_y = m_Previous_path_y[ previous_path_len - 2 ];

		Point const prev( prev_x, prev_y );

		double const ref_x = m_Previous_path_x[ previous_path_len - 1 ];
		double const ref_y = m_Previous_path_y[ previous_path_len - 1 ];

		ref = Point( ref_x, ref_y );

		ref_yaw = atan2((ref_y - prev_y), (ref_x - prev_x));

		pts.emplace_back( prev );
		pts.emplace_back( ref );
	}


	for ( int i = 0; i < 3; ++i )
	{
		static int const OFFSET = 30;

		vector<double> const nextwp = getXY( m_Car_s + (OFFSET * (i + 1)), targetD, map_waypoints_s,
		                                     map_waypoints_x, map_waypoints_y );
		Point const next( nextwp[ 0 ], nextwp[ 1 ] );
		pts.emplace_back( next );
	}

	for ( int i = 0; i < pts.size(); ++i )
	{
		double const shift_x = pts[ i ].x - ref.x;
		double const shift_y = pts[ i ].y - ref.y;

		pts[ i ].x = (shift_x * cos( -ref_yaw ) - shift_y * sin( -ref_yaw ));
		pts[ i ].y = (shift_x * sin( -ref_yaw ) + shift_y * cos( -ref_yaw ));

	}

	tk::spline s;
	sort( pts.begin(), pts.end(), []( Point const &first, Point const &second )
	{
		return first.x < second.x;
	} );

	{
		vector<double> ptsx;
		ptsx.reserve( pts.size());
		vector<double> ptsy;
		ptsy.reserve( pts.size());

		for ( auto const &point : pts )
		{
			ptsx.emplace_back( point.x );
			ptsy.emplace_back( point.y );
		}

		s.set_points( ptsx, ptsy );
	}

	assert( m_Previous_path_y.size() == m_Previous_path_x.size());

	for ( int i = 0; i < m_Previous_path_y.size(); ++i )
	{
		retVal.m_X.emplace_back( m_Previous_path_x[ i ] );
		retVal.m_Y.emplace_back( m_Previous_path_y[ i ] );
	}

	double const target_x = 30.0;
	double const target_y = s( target_x );
	double const target_dist = sqrt((target_x * target_x) + (target_y * target_y));

	double x_addon( 0.0 );
	double const N = (target_dist / (TIME_INTERVAL * targetSpeed));
	double const incFactor = (target_x) / N;

	for ( int i = 0; i < MAX_PATH_POINTS - previous_path_len; ++i )
	{
		double x_point = x_addon + incFactor;
		double y_point = s( x_point );

		x_addon = x_point;

		double const x_temp = x_point;
		double const y_temp = y_point;

		x_point = (x_temp * cos( ref_yaw ) - y_temp * sin( ref_yaw ));
		y_point = (x_temp * sin( ref_yaw ) + y_temp * cos( ref_yaw ));

		x_point += ref.x;
		y_point += ref.y;

		retVal.m_X.emplace_back( x_point );
		retVal.m_Y.emplace_back( y_point );
	}

	return retVal;
}

T_PolynomialParameters Planner::JMT( T_JMTParams const &start, T_JMTParams const &end, double const T )
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
