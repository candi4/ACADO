/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


 /**
 *    \file examples/process/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 25.08.2008
 *
 *     Simple example for getting started with using the Process of the ACADO Toolkit.
 */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main( )
{
	USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------
	DifferentialState xB;
	DifferentialState xW;
	DifferentialState vB;
	DifferentialState vW;

	Disturbance R;
	Control F;

	Parameter mB;
	double mW = 50.0;
	double kS = 20000.0;
	double kT = 200000.0;


    // DEFINE A DYNAMIC SYSTEM:
    // ------------------------
    DifferentialEquation f;

	f << dot(xB) == vB;
	f << dot(xW) == vW;
	f << dot(vB) == ( -kS*xB + kS*xW + F ) / mB;
	f << dot(vW) == (  kS*xB - (kT+kS)*xW + kT*R - F ) / mW;

	OutputFcn g;
	g << xB;
	g << 500.0*vB + F;

    DynamicSystem dynSys( f,g );


    // SETUP THE PROCESS:
    // ------------------
	DVector mean( 1 ), amplitude( 1 );
	mean.setZero( );
	amplitude.setAll( 50.0 );

	GaussianNoise myNoise( mean,amplitude );

	Actuator myActuator( 1,1 );

	myActuator.setControlNoise( myNoise,0.1 );
	myActuator.setControlDeadTimes( 0.1 );
	
	myActuator.setParameterDeadTimes( 0.2 );
// 	myActuator.setParameterNoise( myNoise,0.1 );


	mean.setZero( );
	amplitude.setAll( 0.005 );
	UniformNoise myOutputNoise1( mean,amplitude );
	
	mean.setAll( 10.0 );
	amplitude.setAll( 50.0 );
	GaussianNoise myOutputNoise2( mean,amplitude );
	
	Sensor mySensor( 2 );
	mySensor.setOutputNoise( 0,myOutputNoise1,0.1 );
	mySensor.setOutputNoise( 1,myOutputNoise2,0.1 );
	mySensor.setOutputDeadTimes( 0.2 );


	Process myProcess;
	
	myProcess.setDynamicSystem( dynSys,INT_RK45 );
	myProcess.set( ABSOLUTE_TOLERANCE,1.0e-8 );
	
	myProcess.setActuator( myActuator );
	myProcess.setSensor( mySensor );

	DVector x0( 4 );
	x0.setZero( );
	x0( 0 ) = 0.01;

	myProcess.initializeStartValues( x0 );
	myProcess.setProcessDisturbance( "road.txt" );
	/*
	myProcess.set( PLOT_RESOLUTION,HIGH );
// 	myProcess.set( CONTROL_PLOTTING,PLOT_NOMINAL );
// 	myProcess.set( PARAMETER_PLOTTING,PLOT_NOMINAL );
	myProcess.set( OUTPUT_PLOTTING,PLOT_REAL );

	GnuplotWindow window;
	  window.addSubplot( xB, "Body Position [m]" );
	  window.addSubplot( xW, "Wheel Position [m]" );
	  window.addSubplot( vB, "Body Velocity [m/s]" );
	  window.addSubplot( vW, "Wheel Velocity [m/s]" );

	  window.addSubplot( F,"Damping Force [N]" );
	  window.addSubplot( mB,"Body Mass [kg]" );
	  window.addSubplot( R, "Road Disturbance" );
	  window.addSubplot( g(0),"Output 1" );
	  window.addSubplot( g(1),"Output 2" );

	myProcess << window;
	*/

    // SIMULATE AND GET THE RESULTS:
    // -----------------------------
	VariablesGrid u( 1,0.0,1.0,6 );

	u( 0,0 ) = 10.0;
	u( 1,0 ) = -200.0;
	u( 2,0 ) = 200.0;
	u( 3,0 ) = 0.0;
	u( 4,0 ) = 0.0;
	u( 5,0 ) = 0.0;

	DVector p( 1 );
	p(0) = 350.0;

	DVector pInit( 1 );
	pInit(0) = 300.0;

	myProcess.init( 0.0,x0,u.getFirstVector(),pInit );
	myProcess.run( u,p );

	VariablesGrid xSim, ySim, pSim, cSim, dSim;
	std::vector<float> x_t_sequence, y_t_sequence, p_t_sequence, c_t_sequence, d_t_sequence;
	std::vector<float> xB_sequence, xW_sequence, vB_sequence, vW_sequence;
	std::vector<float> F_sequence, mB_sequence, R_sequence, output1_sequence, output2_sequence;

	myProcess.getLast( LOG_SIMULATED_DIFFERENTIAL_STATES,xSim );
	myProcess.getLast( LOG_PROCESS_OUTPUT,ySim );
	myProcess.getLast( LOG_SIMULATED_PARAMETERS,pSim );
	myProcess.getLast( LOG_SIMULATED_CONTROLS,cSim );
	myProcess.getLast( LOG_SIMULATED_DISTURBANCES,dSim );

	int rols, cols;

    rols = xSim.getNumRows();
    cols = xSim.getDim() / rols;
    for (int i = 0; i < cols; ++i) {
		x_t_sequence.push_back(xSim[i].getTime(0));
		xB_sequence.push_back(xSim(i, 0));
		xW_sequence.push_back(xSim(i, 1));
		vB_sequence.push_back(xSim(i, 2));
		vW_sequence.push_back(xSim(i, 3));
    }

    rols = ySim.getNumRows();
    cols = ySim.getDim() / rols;
    for (int i = 0; i < cols; ++i) {
		y_t_sequence.push_back(ySim[i].getTime(0));
		output1_sequence.push_back(ySim(i, 0));
		output2_sequence.push_back(ySim(i, 1));
    }

    rols = pSim.getNumRows();
    cols = pSim.getDim() / rols;
    for (int i = 0; i < cols; ++i) {
		p_t_sequence.push_back(pSim[i].getTime(0));
		mB_sequence.push_back(pSim(i, 0));
    }

    rols = cSim.getNumRows();
    cols = cSim.getDim() / rols;
    for (int i = 0; i < cols; ++i) {
		c_t_sequence.push_back(cSim[i].getTime(0));
		F_sequence.push_back(cSim(i, 0));
    }

    rols = dSim.getNumRows();
    cols = dSim.getDim() / rols;
    for (int i = 0; i < cols; ++i) {
		d_t_sequence.push_back(dSim[i].getTime(0));
		R_sequence.push_back(dSim(i, 0));
    }

	plt::figure();
    plt::subplot(3,3,1);
    plt::title("Body Position [m]");
    plt::plot(x_t_sequence, xB_sequence);


    plt::subplot(3,3,2);
    plt::title("Wheel Position [m]");
    plt::plot(x_t_sequence, xW_sequence);


	plt::subplot(3,3,3);
    plt::title("Body Velocity [m/s]");
    plt::plot(x_t_sequence, vB_sequence);


	plt::subplot(3,3,4);
    plt::title("Body Velocity [m/s]");
    plt::plot(x_t_sequence, vW_sequence);

	plt::subplot(3,3,5);
    plt::title("Damping Force [N]");
    plt::plot(c_t_sequence, F_sequence);

	plt::subplot(3,3,6);
    plt::title("Body Mass [kg]");
    plt::plot(p_t_sequence, mB_sequence);

	plt::subplot(3,3,7);
    plt::title("Road Disturbance");
    plt::plot(d_t_sequence, R_sequence);

	plt::subplot(3,3,8);
    plt::title("Output 1");
    plt::plot(y_t_sequence, output1_sequence);

	plt::subplot(3,3,9);
    plt::title("Output 2");
    plt::plot(y_t_sequence, output2_sequence);

	plt::show();

	return 0;
}



