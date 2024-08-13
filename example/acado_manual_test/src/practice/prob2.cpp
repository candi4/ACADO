


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <iostream>
#include <fstream>

int main( ){

    USING_NAMESPACE_ACADO


    DifferentialState        x,y,theta      ;     // the differential states
    Control                  v,w          ;     // the control input u
    DifferentialEquation     f;     // the differential equation

//  -------------------------------------
    OCP ocp( 0, 10, 100 );                        // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm(v*v+w*w);               // the time T should be optimized

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

    OptimizationAlgorithm algorithm(ocp);     // the optimization algorithm

    algorithm.solve();                        // solves the problem.

	algorithm.getDifferentialStates("prob1_states.txt");
	algorithm.getControls("prob1_controls.txt");





    return 0;
}


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState      phi;    // the angle phi
    DifferentialState    omega;    // the first derivative of phi w.r.t. time

    Parameter                l;    // the length of the pendulum
    Parameter            alpha;    // frictional constant
    Parameter                g;    // the gravitational constant


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    DifferentialEquation f;

    f << dot(phi ) == omega;
    f << dot(omega) == -(g/l)*sin(phi) - alpha*omega;

    // REMARK: Note that the parameters g, and l are not independent.
    //         Only one of these parameters can be estimated from
    //         the measurements of the dynamic motion.
    // -----------------------------------------------------------------


    // DEFINE A MEASUREMENT FUNCTION:
    // ------------------------------

    Function h;
    h <<   phi;  // The state phi is being measured.


   // DEFINE THE INVERSE OF THE VARIANCE-COVARIANCE MATRIX OF THE MEASUREMENTS:
   // -------------------------------------------------------------------------
    DMatrix S(1,1);
    S(0,0) = 1.0/pow(0.1,2);  // (1 over the variance of the measurement)  HERE: the standard deviation of the measurement is assumed to be 0.1, thus S = 1/(0.1)^2.


    // READ THE MEASUREMENT FROM A DATA FILE:
    // --------------------------------------
    // Note: I fill nan to real values. Because there is some error if data contains nan.
    VariablesGrid measurements;
    measurements.read( "parameter_estimation_data.txt" );

    if( measurements.isEmpty() == BT_TRUE )
        printf("The file \"parameter_estimation_data.txt\" can't be opened.");


    // DEFINE A PARAMETER ESTIMATION PROBLEM:
    // --------------------------------------
    OCP ocp( measurements.getTimePoints() );

    ocp.minimizeLSQ(S, h, measurements );

    ocp.subjectTo( f );

    ocp.subjectTo( 0.0 <= alpha <= 4.0  );
    ocp.subjectTo( 0.0 <=   l   <= 2.0  );

    ocp.subjectTo( g == 9.81 );


    // SETUP AN PLOT WINDOW:
    // ---------------------------------------------------
	/*
    GnuplotWindow window( PLOT_NEVER );

	window.addSubplot( phi,   "The angle phi", "time [s]", "angle [rad]" );
	window.addSubplot( omega, "The angular velocity omega" );
	window.addSubplot( l,     "The length of the pendulum l" );
	window.addSubplot( alpha, "Frictional constant alpha" );
    */

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    ParameterEstimationAlgorithm algorithm(ocp);

//	algorithm << window;
//     algorithm.initializeDifferentialStates( "parameter_estimation_data2.txt" );

    algorithm.solve();


    // GET THE OPTIMAL PARAMETERS:
    // -----------------------------------
    VariablesGrid parameters;
    VariablesGrid differentialstates;
    VariablesGrid controls;
    algorithm.getParameters( parameters );
    algorithm.getDifferentialStates( differentialstates );
    //algorithm.getControls( controls );

    VariablesGrid phi_states, omega_states, l_params, alpha_params;
	std::vector<float> state_t_sequence, param_t_sequence, control_t_sequence, measurement_t_sequence;
    std::vector<float> phi_sequence, omega_sequence, l_sequence, alpha_sequence, measurement_sequence;
	
    int rols, cols;

    rols = differentialstates.getNumRows();
    cols = differentialstates.getDim() / rols;
    for (int i = 0; i < cols; ++i) {
		state_t_sequence.push_back(differentialstates[i].getTime(0));
		phi_sequence.push_back(differentialstates(i, 0));
		omega_sequence.push_back(differentialstates(i, 1));
    }

    rols = parameters.getNumRows();
    cols = parameters.getDim() / rols;
    for (int i = 0; i < cols; ++i) {
		param_t_sequence.push_back(parameters[i].getTime(0));
		l_sequence.push_back(parameters(i, 0));
		alpha_sequence.push_back(parameters(i, 1));
    }

    rols = measurements.getNumRows();
    cols = measurements.getDim() / rols;
    for (int i = 0; i < cols; ++i) {
		measurement_t_sequence.push_back(measurements[i].getTime(0));
		measurement_sequence.push_back(measurements(i, 0));
    }


    plt::figure();
    plt::subplot(2,2,1);
    plt::title("The angle phi");
    plt::named_plot("time [s]", state_t_sequence, phi_sequence);
    plt::scatter(measurement_t_sequence, measurement_sequence);
    plt::ylabel("angle [rad]");

    plt::subplot(2,2,2);
    plt::title("The angular velocity omega");
    plt::plot(state_t_sequence, omega_sequence);

    plt::subplot(2,2,3);
    plt::title("The length of the pendulum l");
    plt::plot(param_t_sequence, l_sequence);

    plt::subplot(2,2,4);
    plt::title("Frictional constant alpha");
    plt::plot(param_t_sequence, alpha_sequence);


// 	return 0;

	// GET THE VARIANCE COVARIANCE IN THE SOLUTION:
	// ---------------------------------------------
	DMatrix var;
	algorithm.getParameterVarianceCovariance( var );

// 	return 0;

	// PRINT THE RESULT ON THE TERMINAL:
	// -----------------------------------------------------------------------

	printf("\n\nResults for the parameters: \n");
	printf("-----------------------------------------------\n");
	printf("   l      =  %.3e  +/-  %.3e \n", parameters(0,0), sqrt( var(0,0) )  );
	printf("   alpha  =  %.3e  +/-  %.3e \n", parameters(0,1), sqrt( var(1,1) )  );
	printf("   g      =  %.3e  +/-  %.3e \n", parameters(0,2), sqrt( var(2,2) )  );
	printf("-----------------------------------------------\n\n\n");


    // PLOT THE RESULT:
    // ---------------------------------------------------
    //algorithm.getPlotWindow( window );
    /*
	window.addData( 0, measurements(0) );
	window.addLine( 2, parameters(0,0) + sqrt( var(0,0) ) );
	window.addLine( 2, parameters(0,0) - sqrt( var(0,0) ) );
	window.addLine( 3, parameters(0,1) + sqrt( var(1,1) ) );
	window.addLine( 3, parameters(0,1) - sqrt( var(1,1) ) );
    */
	//window.plot( );
    
    plt::show();

    return 0;
}
