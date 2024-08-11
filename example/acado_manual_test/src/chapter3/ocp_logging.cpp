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
#include <iostream>
#include <fstream>

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

	// set up a logging object and flush it into the algorithm
	LogRecord logRecord(LOG_AT_EACH_ITERATION);
	logRecord << LOG_KKT_TOLERANCE;
	algorithm << logRecord;

    algorithm.solve();                        // solves the problem.

	// get the logging object back and print it
	algorithm.getLogRecord(logRecord);

    std::ofstream file("log.txt");
    if (file.is_open()) {
        logRecord.print(file);
        file.close();
        std::cout << "Log has been saved to log.txt" << std::endl;
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }

    return 0;
}
