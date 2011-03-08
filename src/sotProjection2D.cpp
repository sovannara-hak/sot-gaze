/*
 * =====================================================================================
 *
 *       Filename:  sotProjection2D.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  08/22/2010 11:35:36 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Hak Sovannara (), shak@laas.fr
 *        Company:  
 *
 * =====================================================================================
 */


#include "sotProjection2D.h"
#include <dynamic-graph/factory.h>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(sotProjection2D,"Projection2D");    

const double sotProjection2D::
MINZ = .001;

sotProjection2D::
sotProjection2D( const std::string& name )
	:Entity(name)
	 ,pointSIN(NULL,"sotProjection2D("+name+")::input(vector3d)::point")
	 ,transformSIN(NULL,"sotProjection2D("+name+")::input(matrixHomo)::transform")
	 ,imageSOUT( boost::bind(&sotProjection2D::projection,this, _1, _2),
			 pointSIN << transformSIN,
			 "sotProjection2D("+name+")::output(vector2d)::image")
{
	dg::Entity::signalRegistration( pointSIN << transformSIN << imageSOUT );
}

void sotProjection2D::
display( std::ostream& os ) const
{
  os << "projection2D "<<getName();
  try{
    os <<"image = "<<imageSOUT; 
  }
  catch (dg::ExceptionSignal e) {}
}

ml::Vector& sotProjection2D::
projection( ml::Vector& point2d, int t){

	point2d.resize(2);

	const ml::Vector& point3d = pointSIN(t);
	const ml::Matrix& transform = transformSIN(t);

	ml::Vector point3dCamera(3);

	point3dCamera = transform.inverse() * point3d;
	
	if(point3dCamera(2) * point3dCamera(2) < MINZ*MINZ)
	{
		point3dCamera(2) = MINZ;
	}

	point2d(0) = point3dCamera(0)/point3dCamera(2);
	point2d(1) = point3dCamera(1)/point3dCamera(2);

	return point2d;
}

