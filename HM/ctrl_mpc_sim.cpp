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

#include "include/prob2_mpc_export/acado_qpoases_interface.hpp"
extern "C"{
    #include "include/prob2_mpc_export/acado_common.h"
    #include "include/prob2_mpc_export/acado_auxiliary_functions.h"
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

    x_next = x + Ts*(coeff1 + 2*coeff2 + 2*coeff3 + coeff4)/6.0;

    return x_next;

}

Eigen::Matrix<float, 2, 1> runAcado(Eigen::Matrix<float, 3, 1> current_state, Eigen::Matrix<float, 3, 1> reference_state, Eigen::Matrix<float, 2, 1> reference_control,
                                    Eigen::Matrix<float, 3, 1> reference_state_next, Eigen::Matrix<float, 2, 1> reference_control_next)
{

	int NY_2 = NY*NY;
    for (int i = 0; i < NH; i++) {

		// {x}
        acadoVariables.W[(NY+1) * 0 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 1 + NY_2 * i] = 10;
        acadoVariables.W[(NY+1) * 2 + NY_2 * i] = 10;

		// {u}
        acadoVariables.W[(NY+1) * 3 + NY_2 * i] = 0.1;
        acadoVariables.W[(NY+1) * 4 + NY_2 * i] = 0.1;

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
        float alpha = i / NH;
        Eigen::Matrix<float, 3, 1> reference_state_interp = alpha*reference_state_next + (1-alpha)*reference_state;
        Eigen::Matrix<float, 2, 1> reference_control_interp = alpha*reference_control_next + (1-alpha)*reference_control;
		acadoVariables.y[i * NY + 0] = reference_state_interp(0);
		acadoVariables.y[i * NY + 1] = reference_state_interp(1);
		acadoVariables.y[i * NY + 2] = reference_state_interp(2);

        acadoVariables.y[i * NY + 3] = reference_control_interp(0);
        acadoVariables.y[i * NY + 4] = reference_control_interp(1);

	}

	acadoVariables.yN[0] = reference_state_next(0);
	acadoVariables.yN[1] = reference_state_next(1);
	acadoVariables.yN[2] = reference_state_next(2);

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



class Reference{
public:
    std::vector<double> timesteps;
    std::vector<double> x_values;
    std::vector<double> y_values;
    std::vector<double> theta_values;
    std::vector<double> v_values;
    std::vector<double> w_values;
    void init(std::vector<double> _timesteps, std::vector<double> _x_values, std::vector<double> _y_values, std::vector<double> _theta_values, 
              std::vector<double> _v_values, std::vector<double> _w_values)
    {
        timesteps = _timesteps;
        x_values = _x_values;
        y_values = _y_values;
        theta_values = _theta_values;
        v_values = _v_values;
        w_values = _w_values;
        double dt = timesteps[1] - timesteps[0];
        for (int i=0; i < NH; i++){
            timesteps.push_back(timesteps.back()+dt);
            x_values.push_back(x_values.back());
            y_values.push_back(y_values.back());
            theta_values.push_back(theta_values.back());
            v_values.push_back(v_values.back());
            w_values.push_back(w_values.back());
        }
    }

};



void plot(std::vector<float> time, std::vector<float> x1, std::vector<float> x2, std::vector<float> x3, std::vector<float> u1, std::vector<float> u2, 
        std::vector<float> computation_time,
        std::vector<double> timesteps, std::vector<double>  x_values, std::vector<double>  y_values, std::vector<double>  theta_values, std::vector<double>  v_values, std::vector<double>  w_values
        )
{

    plt::figure();
    plt::subplot(3,1,1);
    plt::title("State");
    plt::named_plot("x", time, x1);
    plt::named_plot("x_refer", timesteps, x_values);
    plt::ylabel("x");
    plt::legend();

    plt::subplot(3,1,2);
    plt::named_plot("y", time, x2);
    plt::named_plot("y_refer", timesteps, y_values);
    plt::ylabel("y");
    plt::legend();

    plt::subplot(3,1,3);
    plt::named_plot("theta", time, x3);
    plt::named_plot("theta_refer", timesteps, theta_values);
    plt::ylabel("theta");
    plt::xlabel("Time (s)");
    plt::legend();


    plt::figure();
    plt::subplot(2,1,1);
    plt::title("Control input");
    plt::named_plot("v", time, u1);
    plt::named_plot("v_refer", timesteps, v_values);
    plt::ylabel("v");
    plt::legend();

    plt::subplot(2,1,2);
    plt::named_plot("w", time, u2);
    plt::named_plot("w_refer", timesteps, w_values);
    plt::ylabel("w");
    plt::xlabel("Time (s)");
    plt::legend();

    plt::figure();
    plt::hist(computation_time, 5);
    plt::title("Computation time");
    plt::named_plot("Time", time, u1);
    plt::xlabel("Time (microsecond)");
    plt::ylabel("Frequency");


}








// plot x1 - x2
void calculate_error(std::vector<double> time1, std::vector<double> x1, 
                     std::vector<float> time2, std::vector<float> x2)
{
    std::vector<float> time;
    std::vector<float> x_diff;
    float t = -1; // for plot
    
    int idx1 = 1; // for time1
    int idx2 = 1; // for time2
    int size1 = time1.size();
    int size2 = time2.size();

    float t_previous = -1;
    float t1 = time1[0];
    float t2 = time2[0];
    float x1_t = x1[0];
    float x2_t = x2[0];

    
    while (idx1 < size1 && idx2 < size2){
        // Get information
        float t1 = time1[idx1];
        float t2 = time2[idx2];
        float x1_t = x1[idx1];
        float x2_t = x2[idx2];
        // Arrange via time
        if (t1 < t2){
            // Use time1
            t = t1;
            idx1++;
        }
        else if (t1 > t2){
            // Use time2
            t = t2;
            idx2++;
        }
        else{
            // Use time1=time2
            t = t1;
            idx1++;
            idx2++;
        }
        // Stack information for plotting
        time.push_back(t);
        x_diff.push_back(x1_t - x2_t);
    }
    // Use remaining elements
    while (idx1 < size1)
    {
        idx2--;
        // Get information
        float t1 = time1[idx1];
        float t2 = time2[idx2];
        float x1_t = x1[idx1];
        float x2_t = x2[idx2];
        t = t1;
        idx1++;
        // Stack information for plotting
        time.push_back(t);
        x_diff.push_back(x1_t - x2_t);
    }
    while (idx2 < size2)
    {
        idx1--;
        // Get information
        float t1 = time1[idx1];
        float t2 = time2[idx2];
        float x1_t = x1[idx1];
        float x2_t = x2[idx2];
        t = t2;
        idx2++;
        // Stack information for plotting
        time.push_back(t);
        x_diff.push_back(x1_t - x2_t);
    }
    


    plt::named_plot("x_diff", time, x_diff);
}

// plot refer - real
void plot_error(std::vector<double> timesteps, std::vector<double> x_values, std::vector<double> y_values, std::vector<double> theta_values, std::vector<double> v_values, std::vector<double> w_values,
                std::vector<float> time,      std::vector<float> x1, std::vector<float> x2, std::vector<float> x3, std::vector<float> control1, std::vector<float> control2)
{
    
    plt::figure();
    plt::subplot(3,1,1);
    plt::title("State trajectory error");
    calculate_error(timesteps, x_values, time, x1);
    plt::ylabel("x");

    plt::subplot(3,1,2);
    calculate_error(timesteps, y_values, time, x2);
    plt::ylabel("y");

    plt::subplot(3,1,3);
    calculate_error(timesteps, theta_values, time, x3);
    plt::ylabel("theta");
    plt::xlabel("Time (s)");


    plt::figure();
    plt::subplot(2,1,1);
    plt::title("Control input trajectory error");
    calculate_error(timesteps, v_values, time, control1);
    plt::ylabel("v");

    plt::subplot(2,1,2);
    calculate_error(timesteps, w_values, time, control2);
    plt::ylabel("w");
    plt::xlabel("Time (s)");
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


    Reference reference;
    reference.init(timesteps, x_values, y_values, theta_values, v_values, w_values);

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
        acadoVariables.W[(NY+1) * 3 + NY_2 * i] = 0.;
        acadoVariables.W[(NY+1) * 4 + NY_2 * i] = 0.;

    }


    acadoVariables.WN[0] = 10;
    acadoVariables.WN[4] = 10;
    acadoVariables.WN[8] = 10;

    int r = 1;
    Eigen::Matrix<float, 3, 1> x_ref_previous;
    Eigen::Matrix<float, 2, 1> u_ref_previous;
    Eigen::Matrix<float, 3, 1> x_ref_interp, x_ref_interp_previous;
    Eigen::Matrix<float, 2, 1> u_ref_interp, u_ref_interp_previous;
    x_ref_previous = (Eigen::Matrix<float, 3, 1>() << x_values[0], y_values[0], theta_values[0]).finished();
    u_ref_previous = (Eigen::Matrix<float, 2, 1>() << v_values[0], w_values[0]).finished();
    x_ref = (Eigen::Matrix<float, 3, 1>() << x_values[1], y_values[1], theta_values[1]).finished();
    u_ref = (Eigen::Matrix<float, 2, 1>() << v_values[1], w_values[1]).finished();
    x_ref_interp = x_ref_previous;
    u_ref_interp = u_ref_previous;


    for (int t=0; t<T_final; t++)
    {
	    uint64_t start_test = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        


        // interploation
        float alpha = t%5/5.0;
        x_ref_interp_previous = x_ref_interp;
        u_ref_interp_previous = u_ref_interp;
        x_ref_interp = alpha*x_ref + (1-alpha)*x_ref_previous;
        u_ref_interp = alpha*u_ref + (1-alpha)*u_ref_previous;
        if (alpha == 0){
            r++;
            x_ref_previous = x_ref;
            u_ref_previous = u_ref;
            if (T_final-t > 8){
            x_ref = (Eigen::Matrix<float, 3, 1>() << x_values[r], y_values[r], theta_values[r]).finished();
            u_ref = (Eigen::Matrix<float, 2, 1>() << v_values[r], w_values[r]).finished();
            }
        }
        std::cout << "alpha=" << alpha << "\n";
        
        u = runAcado(x, x_ref_interp_previous, u_ref_interp_previous, x_ref_interp, u_ref_interp);
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


    plot(time, x1, x2, x3, control1, control2, computation_time,
         timesteps, x_values, y_values, theta_values, v_values, w_values);

    plot_error(timesteps, x_values, y_values, theta_values, v_values, w_values,
               time,      x1,       x2,       x3,           control1, control2);

    plt::show();

    return 0;

}

