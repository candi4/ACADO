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

    DifferentialState x, y, theta;
    DifferentialState x_dot, y_dot, theta_dot;

    Control u1, u2;

    OnlineData m, l, J;
    OnlineData xc, yc;

    float Ts = 0.02;
    float N = 20;
    float g = 9.81;

    DifferentialEquation f;
    f << dot(x) == x_dot;
    f << dot(y) == y_dot;
    f << dot(theta) == theta_dot;
    f << dot(x_dot) == -(u1+u2)*sin(theta)/m;
    f << dot(y_dot) == (u1+u2)*cos(theta)/m - g;
    f << dot(theta_dot) == l*(-u1+u2)/J;

    Function h;
    h << x << y << theta;
    h << x_dot << y_dot << theta_dot;
    h << u1 << u2;

    Function hN;
    hN << x << y << theta;
    hN << x_dot << y_dot << theta_dot;


    // Create Optimal Control Problem
    OCP ocp(0.0, N*Ts, N);

    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ(Q_sparse, h);
    ocp.minimizeLSQEndTerm(QN_sparse, hN);


    // Dynamics constraint
    ocp.subjectTo(f);

    // State constraints
    ocp.subjectTo(-10 <= x <= 10);
    ocp.subjectTo(-10 <= y <= 10);
    ocp.subjectTo(-M_PI/2 <= theta <= M_PI/2);

    ocp.subjectTo(-10 <= x_dot <= 10);
    ocp.subjectTo(-10 <= y_dot <= 10);
    ocp.subjectTo(-10 <= theta_dot <= 10);

    // Control constraints
    ocp.subjectTo(-10 <= u1 <= 10);
    ocp.subjectTo(-10 <= u2 <= 10);

    // Other constraints


    // Online Data
    ocp.setNOD(3);

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

    if (mpc.exportCode( "planar_drone_mpc_export" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP();


    return EXIT_SUCCESS;
}
