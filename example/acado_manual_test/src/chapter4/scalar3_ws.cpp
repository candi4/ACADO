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
  *    \file   examples/multi_objective/scalar3_nnc.cpp
  *    \author Boris Houska, Filip Logist, Hans Joachim Ferreau
  *    \date   2009
  */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------
    Parameter y1,y2,y3;


    // DEFINE AN OPTIMIZATION PROBLEM:
    // -------------------------------
    NLP nlp;
    nlp.minimize( 0, y1 );
    nlp.minimize( 1, y2 );
    nlp.minimize( 2, y3 );

    nlp.subjectTo( -5.0 <= y1 <= 5.0 );
    nlp.subjectTo( -5.0 <= y2 <= 5.0 );
    nlp.subjectTo( -5.0 <= y3 <= 5.0 );

    nlp.subjectTo( y1*y1+y2*y2+y3*y3 <= 4.0 );


    // DEFINE A MULTI-OBJECTIVE ALGORITHM AND SOLVE THE NLP:
    // -----------------------------------------------------
    MultiObjectiveAlgorithm algorithm(nlp);

    algorithm.set( PARETO_FRONT_GENERATION, PFG_WEIGHTED_SUM );
    algorithm.set( PARETO_FRONT_DISCRETIZATION, 11 );

    // Generate Pareto set 
    algorithm.solve();

    algorithm.getWeights("scalar3_ws_weights.txt");


    // GET THE RESULT FOR THE PARETO FRONT AND PLOT IT:
    // ------------------------------------------------
    VariablesGrid paretoFront;
    // algorithm.getParetoFrontWithFilter( paretoFront );
    algorithm.getParetoFront( paretoFront );
    paretoFront.print();

    //GnuplotWindow window;
    //window.addSubplot3D( paretoFront, "Pareto Front y1 vs y2 vs y3","y1","y2", PM_POINTS );
    //window.plot( );

    std::vector<float> y1_sequence, y2_sequence, y3_sequence;

    int rols, cols;

    rols = paretoFront.getNumRows();
    cols = paretoFront.getDim() / rols;
    for (int i = 0; i < cols; ++i) {
      y3_sequence.push_back(paretoFront[i].getTime(0));
      y1_sequence.push_back(paretoFront(i, 0));
      y2_sequence.push_back(paretoFront(i, 1));
    }

    plt::scatter(y1_sequence, y2_sequence, y3_sequence);
    plt::title("Pareto Front y1 vs y2 vs y3");
    plt::xlabel("y1");
    plt::ylabel("y2");
    plt::set_zlabel("y3");
    plt::show();
    // PRINT INFORMATION ABOUT THE ALGORITHM:
    // --------------------------------------
    algorithm.printInfo();

    return 0;
}
/* <<< end tutorial code <<< */

