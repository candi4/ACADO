#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <math.h>
#include <vector>
#include <random>
#include <chrono>
#include "matplotlibcpp.h"


#include "MPC_SOLVER/acado_qpoases_interface.hpp"
extern "C"{
    #include "MPC_SOLVER/acado_common.h"
    #include "MPC_SOLVER/acado_auxiliary_functions.h"
}

#define NX		ACADO_NX	// Number of states	
#define NXA		ACADO_NXA	// Number of algebraic variables
#define NU		ACADO_NU	// Number of control inputs
#define NP		ACADO_NP	// Number of parameters
#define NY		ACADO_NY	// Number of reference on 0->N-1
#define NYN		ACADO_NYN	// Number of reference on N
#define NH		ACADO_N		// Number of horizon
#define NOD		ACADO_NOD	// Number of online data
#define N_STEPS	10
#define VERBOSE	0

ACADOvariables	acadoVariables;
ACADOworkspace	acadoWorkspace;


namespace plt = matplotlibcpp;



float g = 9.81;
float m = 1.5;
float l = 1;
float I = 1.0/12.0*m*(2*l)*(2*l);

float xc = 1;
float yc = 1;
float rc = 0;

float Ts = 0.001;
float Tc = 0.02;
int hold = Tc/Ts;
float warm_start_state[NX*(NH+1)];
float warm_start_control[NU*(NH)];

int T_final = 20/Tc;

// Initial state and control
Eigen::Matrix<float, 6, 1> x = {10, -10, -1.0, 1, 0.5, 0.2};
Eigen::Matrix<float, 6, 1> x_ref = {0, 0, 0, 0, 0, 0};

Eigen::Matrix<float, 2, 1> u;
Eigen::Matrix<float, 2, 1> u_ref = {m*g/2, m*g/2};


Eigen::Matrix<float, 6, 1> planar_drone_dnm(Eigen::Matrix<float, 6, 1> x, Eigen::Matrix<float, 2, 1> u)
{
    Eigen::Matrix<float, 6, 1> x_dot;

    x_dot(0) = x(3);
    x_dot(1) = x(4);
    x_dot(2) = x(5);

    x_dot(3) = -(u(0)+u(1))*sin(x(2))/m;
    x_dot(4) = (u(0)+u(1))*cos(x(2))/m - g;
    x_dot(5) = l*(-u(0)+u(1))/I;

    return x_dot;

}


Eigen::Matrix<float, 6, 1> update_RK4(Eigen::Matrix<float, 6, 1> x, Eigen::Matrix<float, 2, 1> u)
{

    // https://www.youtube.com/watch?v=OOL5Mo_jYGY
    Eigen::Matrix<float, 6, 1> x_next;
    Eigen::Matrix<float, 6, 1> coeff1, coeff2, coeff3, coeff4;

    coeff1 << planar_drone_dnm(x, u);
    coeff2 << planar_drone_dnm(x + 0.5*Ts*coeff1, u);
    coeff3 << planar_drone_dnm(x + 0.5*Ts*coeff2, u);
    coeff4 << planar_drone_dnm(x + Ts*coeff3, u);

    x_next = x + Ts*0.167*(coeff1 + 2*coeff2 + 2*coeff3 + coeff4);

    return x_next;

}


Eigen::Matrix<float, 2, 1> runAcado(Eigen::Matrix<float, 6, 1> current_state, Eigen::Matrix<float, 6, 1> reference_state, Eigen::Matrix<float, 2, 1> reference_control)
{
	int NY_2 = NY*NY;
    for (int i = 0; i < NH; i++) {

		// {x, theta, x_dot, theta_dot}
        acadoVariables.W[(NY+1) * 0 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 1 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 2 + NY_2 * i] = 10;

        acadoVariables.W[(NY+1) * 3 + NY_2 * i] = 1;
        acadoVariables.W[(NY+1) * 4 + NY_2 * i] = 1;
        acadoVariables.W[(NY+1) * 5 + NY_2 * i] = 1;

		// {u}
        acadoVariables.W[(NY+1) * 6 + NY_2 * i] = 0.1;
        acadoVariables.W[(NY+1) * 7 + NY_2 * i] = 0.1;

    }


    acadoVariables.WN[0] = acadoVariables.W[0];
    acadoVariables.WN[7] = acadoVariables.W[9];
    acadoVariables.WN[14] = acadoVariables.W[18];
    acadoVariables.WN[21] = acadoVariables.W[27];
    acadoVariables.WN[28] = acadoVariables.W[36];
    acadoVariables.WN[35] = acadoVariables.W[45];


	// Initial state
	acadoVariables.x0[0] = current_state(0);
    acadoVariables.x0[1] = current_state(1);
    acadoVariables.x0[2] = current_state(2);

    acadoVariables.x0[3] = current_state(3);
    acadoVariables.x0[4] = current_state(4);
    acadoVariables.x0[5] = current_state(5);

	// Set reference
	for (int i = 0; i < NH; ++i) {
		acadoVariables.y[i * NY + 0] = reference_state(0);
		acadoVariables.y[i * NY + 1] = reference_state(1);
		acadoVariables.y[i * NY + 2] = reference_state(2);
		acadoVariables.y[i * NY + 3] = reference_state(3);
		acadoVariables.y[i * NY + 4] = reference_state(4);
        acadoVariables.y[i * NY + 5] = reference_state(5);

        acadoVariables.y[i * NY + 6] = reference_control(0);
        acadoVariables.y[i * NY + 7] = reference_control(1);

	}

	acadoVariables.yN[0] = reference_state(0);
	acadoVariables.yN[1] = reference_state(1);
	acadoVariables.yN[2] = reference_state(2);
    acadoVariables.yN[3] = reference_state(3);
    acadoVariables.yN[3] = reference_state(4);
    acadoVariables.yN[3] = reference_state(5);  

	// bound value
	for (int i=0; i<NH; i++){
        // Control constraint
        // u = {u1, u2}
        // u_min <= u <= umax
		acadoVariables.lbValues[i*NU + 0] = 1;
        acadoVariables.ubValues[i*NU + 0] = 30;

        acadoVariables.lbValues[i*NU + 1] = 1;
        acadoVariables.ubValues[i*NU + 1] = 30;

        // State constraint
        // x = {x,y,theta,x_dot,y_dot,theta_dot}
        // x_min <= x <= x_max
        acadoVariables.lbAValues[i*6 + 0] = -20;
        acadoVariables.ubAValues[i*6 + 0] = 20;

        acadoVariables.lbAValues[i*6 + 1] = -20;
        acadoVariables.ubAValues[i*6 + 1] = 20;

        acadoVariables.lbAValues[i*6 + 2] = -M_PI/2;
        acadoVariables.ubAValues[i*6 + 2] = M_PI/2;

        acadoVariables.lbAValues[i*6 + 3] = -3;
        acadoVariables.ubAValues[i*6 + 3] = 3;

        acadoVariables.lbAValues[i*6 + 4] = -3;
        acadoVariables.ubAValues[i*6 + 4] = 3;

        acadoVariables.lbAValues[i*6 + 5] = -3;
        acadoVariables.ubAValues[i*6 + 5] = 3;
	}


    // Online data (m, l, I)
    for (int i=0; i<NH; i++){

        acadoVariables.od[i*NOD + 0] = m;
        acadoVariables.od[i*NOD + 1] = l;
        acadoVariables.od[i*NOD + 2] = I;
    }

	acado_preparationStep();
    acado_feedbackStep();

	// for (int i=0; i<(NH)*NX; i++){
	// 	warm_start_state[i] = acadoVariables.x[i];
	// }
	// for (int i=0; i<(NH-1)*NU; i++){
	// 	warm_start_control[i] = acadoVariables.u[i];
	// }


    // acado_printDifferentialVariables();
    // acado_printControlVariables();

    // for (int i=0; i<NX; i++){
    //     std::cout << "x0: " << acadoVariables.x0[i] << std::endl;
    //     std::cout << "x: " << acadoVariables.x[i] << std::endl;
    //     std::cout << "x: " << acadoVariables.x[i+NX] << std::endl;
    // }


    Eigen::Matrix<float, 2, 1> control_input;
    control_input << acadoVariables.u[0],
                    acadoVariables.u[1];

    return control_input;

}




void plot(std::vector<float> time, std::vector<float> x1, std::vector<float> x2, std::vector<float> x3, 
        std::vector<float> x4, std::vector<float> x5, std::vector<float> x6, std::vector<float> u1, std::vector<float> u2, 
        std::vector<float> computation_time)
{

    plt::figure();
    plt::subplot(3,1,1);
    plt::title("Position");
    plt::named_plot("x", time, x1);
    plt::ylabel("X");

    plt::subplot(3,1,2);
    plt::named_plot("y", time, x2);
    plt::ylabel("Y");

    plt::subplot(3,1,3);
    plt::named_plot("theta", time, x3);
    plt::ylabel("Theta");
    plt::xlabel("Time (s)");

    plt::figure();
    plt::subplot(3,1,1);
    plt::title("Velocity");
    plt::named_plot("x_dot", time, x4);
    plt::ylabel("x_dot");

    plt::subplot(3,1,2);
    plt::named_plot("y_dot", time, x5);
    plt::ylabel("y_dot");

    plt::subplot(3,1,3);
    plt::named_plot("theta_dot", time, x6);
    plt::ylabel("theta_dot");
    plt::xlabel("Time (s)");

    plt::figure();
    plt::subplot(2,1,1);
    plt::title("Control input");
    plt::named_plot("u1", time, u1);
    plt::ylabel("u1");

    plt::subplot(2,1,2);
    plt::named_plot("u2", time, u2);
    plt::ylabel("u2");

    plt::figure();
    plt::hist(computation_time, 5);
    plt::title("Computation time");
    plt::named_plot("Time", time, u1);
    plt::xlabel("Time (microsecond)");
    plt::ylabel("Frequency");


    plt::show();
}



int  main()
{
    std::vector<float> time;
    std::vector<float> x1;
    std::vector<float> x2;
    std::vector<float> x3;

    std::vector<float> x4;
    std::vector<float> x5;
    std::vector<float> x6;

    std::vector<float> control1;
    std::vector<float> control2;

    std::vector<float> computation_time;
    float computation_time_mean;
    float computation_time_max;

    acado_initializeSolver();

    int NY_2 = NY*NY;
    for (int i = 0; i < NH; i++) {

		// {x, theta, x_dot, theta_dot}
        acadoVariables.W[(NY+1) * 0 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 1 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 2 + NY_2 * i] = 10;

        acadoVariables.W[(NY+1) * 3 + NY_2 * i] = 1;
        acadoVariables.W[(NY+1) * 4 + NY_2 * i] = 1;
        acadoVariables.W[(NY+1) * 5 + NY_2 * i] = 1;

		// {u}
        acadoVariables.W[(NY+1) * 6 + NY_2 * i] = 0.01;
        acadoVariables.W[(NY+1) * 7 + NY_2 * i] = 0.01;

    }


    acadoVariables.WN[0] = acadoVariables.W[0];
    acadoVariables.WN[7] = acadoVariables.W[9];
    acadoVariables.WN[14] = acadoVariables.W[18];
    acadoVariables.WN[21] = acadoVariables.W[27];
    acadoVariables.WN[28] = acadoVariables.W[36];
    acadoVariables.WN[35] = acadoVariables.W[45];

    for (int t=0; t<T_final; t++)
    {

	    uint64_t start_test = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        u = runAcado(x, x_ref, u_ref);
        uint64_t stop_test = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        computation_time.push_back(stop_test - start_test); 
        computation_time_mean += stop_test - start_test;

        if (stop_test - start_test > computation_time_max)
        {
            computation_time_max = stop_test - start_test;
        }

        std::cout << "u: " << t << std::endl;

        for (int i=0; i<hold; i++){
            x = update_RK4(x,u);
        }
        time.push_back(t*Tc);
        x1.push_back(x(0));
        x2.push_back(x(1));
        x3.push_back(x(2));
        x4.push_back(x(3));
        x5.push_back(x(4));
        x6.push_back(x(5));
        control1.push_back(u(0));
        control2.push_back(u(1));
    }

    float T_final_float = T_final;
    std::cout << "mean: " << computation_time_mean/T_final_float << std::endl;
    std::cout << "max: " << computation_time_max << std::endl;

    plot(time, x1, x2, x3, x4, x5, x6, control1, control2, computation_time);


    return 0;

}

