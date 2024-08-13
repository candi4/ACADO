#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <math.h>
#include <vector>
#include <random>
#include <chrono>
#include "matplotlibcpp.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

namespace plt = matplotlibcpp;

#include "prob2_mpc_export/acado_qpoases_interface.hpp"
extern "C"{
    #include "prob2_mpc_export/acado_common.h"
    #include "prob2_mpc_export/acado_auxiliary_functions.h"
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


float Ts = 0.001;
float Tc = 0.02;
int hold = Tc/Ts;
int T_final = 10/Tc;



// Initial state and control
Eigen::Matrix<float, 3, 1> x = {-1, -1, -M_PI/4}; // x, y, theta
Eigen::Matrix<float, 3, 1> x_ref = {0,0,0};

Eigen::Matrix<float, 2, 1> u; // v,w
Eigen::Matrix<float, 2, 1> u_ref = {0, 0};


Eigen::Matrix<float, 3, 1> planar_drone_dnm(Eigen::Matrix<float, 3, 1> x, Eigen::Matrix<float, 2, 1> u)
{
    Eigen::Matrix<float, 3, 1> x_dot;

    x_dot(0) = u(0)*cos(x(2));
    x_dot(1) = u(0)*sin(x(2));
    x_dot(2) = u(1);

    return x_dot;

}


Eigen::Matrix<float, 3, 1> update_RK4(Eigen::Matrix<float, 3, 1> x, Eigen::Matrix<float, 2, 1> u)
{

    // https://www.youtube.com/watch?v=OOL5Mo_jYGY
    Eigen::Matrix<float, 3, 1> x_next;
    Eigen::Matrix<float, 3, 1> coeff1, coeff2, coeff3, coeff4;

    coeff1 << planar_drone_dnm(x, u);
    coeff2 << planar_drone_dnm(x + 0.5*Ts*coeff1, u);
    coeff3 << planar_drone_dnm(x + 0.5*Ts*coeff2, u);
    coeff4 << planar_drone_dnm(x + Ts*coeff3, u);

    x_next = x + Ts*0.167*(coeff1 + 2*coeff2 + 2*coeff3 + coeff4);

    return x_next;

}

Eigen::Matrix<float, 2, 1> runAcado(Eigen::Matrix<float, 3, 1> current_state, Eigen::Matrix<float, 3, 1> reference_state, Eigen::Matrix<float, 2, 1> reference_control)
{

	int NY_2 = NY*NY;
    for (int i = 0; i < NH; i++) {

		// {x}
        acadoVariables.W[(NY+1) * 0 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 1 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 2 + NY_2 * i] = 10;

		// {u}
        acadoVariables.W[(NY+1) * 3 + NY_2 * i] = 1;
        acadoVariables.W[(NY+1) * 4 + NY_2 * i] = 1;

    }


    acadoVariables.WN[0] = 10; // 10
    acadoVariables.WN[4] = 10; // 10
    acadoVariables.WN[8] = 10; // 10


	// Initial state
	acadoVariables.x0[0] = current_state(0);
    acadoVariables.x0[1] = current_state(1);
    acadoVariables.x0[2] = current_state(2);


	// Set reference
	for (int i = 0; i < NH; ++i) {
		acadoVariables.y[i * NY + 0] = reference_state(0);
		acadoVariables.y[i * NY + 1] = reference_state(1);
		acadoVariables.y[i * NY + 2] = reference_state(2);

        acadoVariables.y[i * NY + 3] = reference_control(0);
        acadoVariables.y[i * NY + 4] = reference_control(1);

	}

	acadoVariables.yN[0] = reference_state(0);
	acadoVariables.yN[1] = reference_state(1);
	acadoVariables.yN[2] = reference_state(2);

	// bound value
	for (int i=0; i<NH; i++){
		acadoVariables.lbValues[i*NU + 0] = -2;
        acadoVariables.ubValues[i*NU + 0] = 2;

        acadoVariables.lbValues[i*NU + 1] = -2;
        acadoVariables.ubValues[i*NU + 1] = 2;

	}


	for (int i=0; i<20; i++){
        acadoVariables.lbAValues[i] = -M_PI/2;
        acadoVariables.ubAValues[i] = M_PI/2;
    }

	acado_preparationStep();
    acado_feedbackStep();


    Eigen::Matrix<float, 2, 1> control_input;
    control_input << acadoVariables.u[0],
                    acadoVariables.u[1];

    return control_input;

}

void plot(std::vector<float> time, std::vector<float> x1, std::vector<float> x2, std::vector<float> x3, std::vector<float> u1, std::vector<float> u2, 
        std::vector<float> computation_time)
{

    plt::figure();
    plt::subplot(3,1,1);
    plt::title("State");
    plt::named_plot("x", time, x1);
    plt::ylabel("x");

    plt::subplot(3,1,2);
    plt::named_plot("y", time, x2);
    plt::ylabel("y");

    plt::subplot(3,1,3);
    plt::named_plot("theta", time, x3);
    plt::ylabel("theta");
    plt::xlabel("Time (s)");


    plt::figure();
    plt::subplot(2,1,1);
    plt::title("Control input");
    plt::named_plot("v", time, u1);
    plt::ylabel("v");

    plt::subplot(2,1,2);
    plt::named_plot("w", time, u2);
    plt::ylabel("w");
    plt::xlabel("Time (s)");

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

    std::ifstream inputFile("prob1_states.txt"); 
    if (!inputFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    std::vector<double> timesteps;
    std::vector<double> x_values;
    std::vector<double> y_values;
    std::vector<double> theta_values;

    std::string line;
    while (std::getline(inputFile, line)) {
        line.erase(std::remove(line.begin(), line.end(), '['), line.end());
        line.erase(std::remove(line.begin(), line.end(), ']'), line.end());

        std::istringstream iss(line);
        double timestep, x, y, theta;
        
        iss >> timestep >> x >> y >> theta;
        
        timesteps.push_back(timestep);
        x_values.push_back(x);
        y_values.push_back(y);
        theta_values.push_back(theta);
    }

    inputFile.close();    


    std::ifstream inputFile2("prob1_controls.txt"); 
    if (!inputFile2.is_open()) {
        std::cerr << "Error opening file!2" << std::endl;
        return 1;
    }

    std::vector<double> v_values;
    std::vector<double> w_values;

    std::string line2;
    while (std::getline(inputFile2, line2)) {
        line2.erase(std::remove(line2.begin(), line2.end(), '['), line2.end());
        line2.erase(std::remove(line2.begin(), line2.end(), ']'), line2.end());

        std::istringstream iss(line2);
        double first, v, w;
        
        iss  >> first >> v >> w ;
        
        v_values.push_back(v);
        w_values.push_back(w);
    }

    inputFile.close();    

    std::vector<float> time;
    std::vector<float> x1;
    std::vector<float> x2;
    std::vector<float> x3;


    std::vector<float> control1;
    std::vector<float> control2;

    std::vector<float> computation_time;
    float computation_time_mean;
    float computation_time_max;

    acado_initializeSolver();

    int NY_2 = NY*NY;
    for (int i = 0; i < NH; i++) {

		// {x}
        acadoVariables.W[(NY+1) * 0 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 1 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 2 + NY_2 * i] = 10;

		// {u}
        acadoVariables.W[(NY+1) * 3 + NY_2 * i] = 0.01;
        acadoVariables.W[(NY+1) * 4 + NY_2 * i] = 0.01;

    }


    acadoVariables.WN[0] = 10;
    acadoVariables.WN[4] = 10;
    acadoVariables.WN[8] = 10;

    int r = 0;

    for (int t=0; t<T_final; t++)
    {
	    uint64_t start_test = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        
        x_ref = {x_values[r], y_values[r], theta_values[r]};
        u_ref = (Eigen::Matrix<float, 2, 1>() << v_values[r], w_values[r]).finished();

        if (t%5 == 0)
        {
           r++;
           x_ref = (Eigen::Matrix<float, 3, 1>() << x_values[r], y_values[r], theta_values[r]).finished();
           u_ref = (Eigen::Matrix<float, 2, 1>() << v_values[r], w_values[r]).finished();
        }
        
        u = runAcado(x, x_ref, u_ref);
        uint64_t stop_test = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        computation_time.push_back(stop_test - start_test); 
        computation_time_mean += stop_test - start_test;

        if (stop_test - start_test > computation_time_max)
        {
            computation_time_max = stop_test - start_test;
        }

        for (int i=0; i<hold; i++){
            x = update_RK4(x,u);
        }
        time.push_back(t*Tc);
        x1.push_back(x(0));
        x2.push_back(x(1));
        x3.push_back(x(2));
        control1.push_back(u(0));
        control2.push_back(u(1));
    }

    float T_final_float = T_final;

    plot(time, x1, x2, x3, control1, control2, computation_time);


    return 0;

}

