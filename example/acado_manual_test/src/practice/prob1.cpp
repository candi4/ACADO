


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <iostream>
#include <fstream>

// namespace plt = matplotlibcpp;

int main( ){

    USING_NAMESPACE_ACADO


    DifferentialState        x,y,theta      ;     // the differential states
    Control                  v,w          ;     // the control input u
    DifferentialEquation     f;     // the differential equation

//  -------------------------------------
    OCP ocp( 0, 10, 100 );                        // time horizon of the OCP: [0,T]
    ocp.minimizeLagrangeTerm(v*v+w*w);               // the time T should be optimized

    f << dot(x)     == v*cos(theta);                         // an implementation
    f << dot(y)     == v*sin(theta);             // of the model equations
    f << dot(theta) == w;                 // for the rocket.

    ocp.subjectTo( f                   );     // minimize T s.t. the model,
    ocp.subjectTo( AT_START, x     ==  0 );     // the initial values for s,
    ocp.subjectTo( AT_START, y     ==  0 );     // v,
    ocp.subjectTo( AT_START, theta ==  0 );     // and m,

    ocp.subjectTo( AT_END  , x     == 10 );     // the terminal constraints for s
    ocp.subjectTo( AT_END  , y     ==  5 );     // and v,
    ocp.subjectTo( AT_END  , theta ==  0 );     // and v,

    ocp.subjectTo( -2 <= v <=  2   );     // as well as the bounds on v
    ocp.subjectTo( -2 <= w <=  2   );     // the control input u,
    ocp.subjectTo(  -M_PI/4 <= theta <= M_PI/2   );     // and the time horizon T.
//  -------------------------------------

    OptimizationAlgorithm algorithm(ocp);     // the optimization algorithm
    algorithm.set(HESSIAN_APPROXIMATION, EXACT_HESSIAN);
    algorithm.set(KKT_TOLERANCE, 1e-10);
    algorithm.solve();                        // solves the problem.

	algorithm.getDifferentialStates("prob1_states.txt");
	algorithm.getControls("prob1_controls.txt");

    VariablesGrid state, input;
    algorithm.getDifferentialStates(state);
    algorithm.getControls(input);

    GnuplotWindow window;
    window.addSubplot(state(0), "x");
    window.addSubplot(state(1), "y");
    window.addSubplot(state(2), "theta");
    window.addSubplot(input(0), "v");
    window.addSubplot(input(1), "w");
    window.plot();

    return 0;
}
