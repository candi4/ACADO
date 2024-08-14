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

#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>
#include <matrix_vector/vector.hpp>

USING_NAMESPACE_ACADO

int main( ){

    const bool CODE_GEN = true;

    DifferentialState        x,y,theta      ;     // the differential states
    Control                  v,w          ;     // the control input u
    DifferentialEquation     f;     // the differential equation

    double time_step = 0.02;
    int N = 20;

//  -------------------------------------
    OCP ocp(0, time_step*N, N);                        // time horizon of the OCP: [0,T]

    f << dot(x)     == v*cos(theta);                         // an implementation
    f << dot(y)     == v*sin(theta);             // of the model equations
    f << dot(theta) == w;                 // for the rocket.

    ocp.subjectTo( f                   );     // minimize T s.t. the model,
    ocp.subjectTo( AT_START, x     ==  -1 );     // the initial values for s,
    ocp.subjectTo( AT_START, y     ==  -1 );     // v,
    ocp.subjectTo( AT_START, theta ==  -M_PI/4 );     // and m,

    ocp.subjectTo( -2 <= v <=  2   );     // as well as the bounds on v
    ocp.subjectTo( -2 <= w <=  2   );     // the control input u,
    ocp.subjectTo(  -M_PI/4 <= theta <= M_PI/2   );     // and the time horizon T.
//  -------------------------------------

    Function h;
    h << x << y << theta;
    h << v << w;

    Function hN;
    hN << x << y << theta;

    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ(Q_sparse, h);
    ocp.minimizeLSQEndTerm(QN_sparse, hN);

    OCPexport mpc(ocp);
    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON); // Hessian approximation method
    mpc.set(INTEGRATOR_TYPE, INT_RK4); // Integrate method (RK4) *
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING); // Shooting technique
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2);
    mpc.set(NUM_INTEGRATOR_STEPS, N); // Number of integrator step
    mpc.set(QP_SOLVER, QP_QPOASES); // QP solver type
    mpc.set(HOTSTART_QP, YES); // Initial guess *
    mpc.set(LEVENBERG_MARQUARDT, 1e-10);
    mpc.set(LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    mpc.set(CG_USE_OPENMP, YES); // Parallelization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO); // Set constraint on runtime *
    mpc.set(USE_SINGLE_PRECISION, YES); 
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES); // Use different matrix on each shooting node

    mpc.set( GENERATE_TEST_FILE,          NO);
    mpc.set( GENERATE_MAKE_FILE,          NO);
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

    if (mpc.exportCode( "../include/prob2_mpc_export" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP();


    return EXIT_SUCCESS;
}
