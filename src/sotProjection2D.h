/*
 * =====================================================================================
 *
 *       Filename:  sotProjection2D.h
 *
 *    Description:  projection of a point from a 3Dpoint into an image point
 *    		    ct = Mt*wt = [X, Y, Z]; x=X/Z, y=Y/Z
 *
 *        Version:  1.0
 *        Created:  08/22/2010 11:08:26 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Hak Sovannara (), shak@laas.fr
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __SOT_PROJECTION_2D_HH__
#define __SOT_PROJECTION_2D_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <boost/bind.hpp>
#include <boost/format.hpp>

/* SOT */
#include <jrl/mal/boost.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <sot-dynamic/dynamic.h>
namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;
namespace sot = ::dynamicgraph::sot;

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class sotProjection2D
: public dg::Entity
{
	public:
		sotProjection2D( const std::string & name );

      	public:  /* --- SIGNALS --- */   
		
		dg::SignalPtr<ml::Vector,int> pointSIN;
		dg::SignalPtr<ml::Matrix,int> transformSIN;
		dg::SignalTimeDependent<ml::Vector,int> imageSOUT; 

	public: /* --- ENTITY INHERITANCE --- */
		static const std::string CLASS_NAME;
		virtual void display( std::ostream& os ) const; 
		virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

	protected:
		static const double MINZ;
	protected:
		ml::Vector& projection(ml::Vector& point2d, int t);
};
#endif  
