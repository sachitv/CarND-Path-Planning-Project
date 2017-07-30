//
// Created by Sachit Vithaldas on 29/07/17.
//
#pragma once
#ifndef PATH_PLANNING_SENSORFUSIONDATA_H
#define PATH_PLANNING_SENSORFUSIONDATA_H

#include <cstdint>

struct S_SensorFusionData
{
	int m_Id;
	double m_X;
	double m_Y;
	double m_Vx;
	double m_Vy;
	double m_S;
	double m_D;

	S_SensorFusionData( int const id, int const x, int const y, int const vx, int const vy, int const s, int const d )
			: m_Id( id )
			  , m_X( x )
			  , m_Y( y )
			  , m_Vx( vx )
			  , m_Vy( vy )
			  , m_S( s )
			  , m_D( d )
	{
	}
};

#endif //PATH_PLANNING_SENSORFUSIONDATA_H
