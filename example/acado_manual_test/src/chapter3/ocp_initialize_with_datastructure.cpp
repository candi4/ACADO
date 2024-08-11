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
 *    \file   examples/getting_started/simple_ocp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    DifferentialState        s,v,m      ;     // the differential states
    Control                  u          ;     // the control input u
    Parameter                T          ;     // the time horizon T
    DifferentialEquation     f( 0.0, T );     // the differential equation

//  -------------------------------------
    OCP ocp( 0.0, T );                        // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T );               // the time T should be optimized

    f << dot(s) == v;                         // an implementation
    f << dot(v) == (u-0.2*v*v)/m;             // of the model equations
    f << dot(m) == -0.01*u*u;                 // for the rocket.

    ocp.subjectTo( f                   );     // minimize T s.t. the model,
    ocp.subjectTo( AT_START, s ==  0.0 );     // the initial values for s,
    ocp.subjectTo( AT_START, v ==  0.0 );     // v,
    ocp.subjectTo( AT_START, m ==  1.0 );     // and m,

    ocp.subjectTo( AT_END  , s == 10.0 );     // the terminal constraints for s
    ocp.subjectTo( AT_END  , v ==  0.0 );     // and v,

    ocp.subjectTo( -0.1 <= v <=  1.7   );     // as well as the bounds on v
    ocp.subjectTo( -1.1 <= u <=  1.1   );     // the control input u,
    ocp.subjectTo(  5.0 <= T <= 15.0   );     // and the time horizon T.
//  -------------------------------------

    OptimizationAlgorithm algorithm(ocp);     // the optimization algorithm
    

	// initialize using data structures.
	Grid timeGrid(0.0, 1.0, 11);
	VariablesGrid xinit(3, timeGrid);
	VariablesGrid uinit(1, timeGrid);
	VariablesGrid pinit(1, timeGrid);

	xinit(0, 0) = 0.00e+00; xinit(0, 1) = 0.00e+00; xinit(0, 2) = 1.00e+00;
	xinit(1, 0) = 2.99e-01; xinit(1, 1) = 7.90e-01; xinit(1, 2) = 9.90e-01;
	xinit(2, 0) = 1.13e+00; xinit(2, 1) = 1.42e+00; xinit(2, 2) = 9.81e-01;
	xinit(3, 0) = 2.33e+00; xinit(3, 1) = 1.69e+00; xinit(3, 2) = 9.75e-01;
	xinit(4, 0) = 3.60e+00; xinit(4, 1) = 1.70e+00; xinit(4, 2) = 9.73e-01;
	xinit(5, 0) = 4.86e+00; xinit(5, 1) = 1.70e+00; xinit(5, 2) = 9.70e-01;
	xinit(6, 0) = 6.13e+00; xinit(6, 1) = 1.70e+00; xinit(6, 2) = 9.68e-01;
	xinit(7, 0) = 7.39e+00; xinit(7, 1) = 1.70e+00; xinit(7, 2) = 9.65e-01;
	xinit(8, 0) = 8.66e+00; xinit(8, 1) = 1.70e+00; xinit(8, 2) = 9.63e-01;
	xinit(9, 0) = 9.67e+00; xinit(9, 1) = 8.98e-01; xinit(9, 2) = 9.58e-01;
	xinit(10, 0) = 1.00e+01; xinit(10, 1) = 0.00e+00; xinit(10, 2) = 9.49e-01;

	uinit(0, 0) = 1.10e+00;
	uinit(1, 0) = 1.10e+00;
	uinit(2, 0) = 1.10e+00;
	uinit(3, 0) = 5.78e-01;
	uinit(4, 0) = 5.78e-01;
	uinit(5, 0) = 5.78e-01;
	uinit(6, 0) = 5.78e-01;
	uinit(7, 0) = 5.78e-01;
	uinit(8, 0) = -2.12e-01;
	uinit(9, 0) = -1.10e+00;
	uinit(10, 0) = -1.10e+00;

	pinit(0, 0) = 7.44e+00;

	algorithm.initializeDifferentialStates(xinit);
	algorithm.initializeControls(uinit);
	algorithm.initializeParameters(pinit);
	
    algorithm.solve();                        // solves the problem.


    return 0;
}
