from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from planar_drone_model import export_planar_drone_ode_model
import numpy as np
from utils import plot_planar
from casadi import vertcat
import scipy.linalg
from math import sin, cos


m = 1.5
l = 1
Inertia = 1/12*m*(2*l)**2
g = 9.81

def planar_drone_dnm(x,u):
    x_dot = np.zeros((6))
    x_dot[0] = x[3]
    x_dot[1] = x[4]
    x_dot[2] = x[5]
    x_dot[3] = -(u[0]+u[1])*sin(x[2])/m
    x_dot[4] = (u[0]+u[1])*cos(x[2])/m - g
    x_dot[5] = l*(-u[0]+u[1])/Inertia

    return x_dot

def update_RK4(x,u,Ts):

    coeff1 = planar_drone_dnm(x, u)
    coeff2 = planar_drone_dnm(x + 0.5*Ts*coeff1, u)
    coeff3 = planar_drone_dnm(x + 0.5*Ts*coeff2, u)
    coeff4 = planar_drone_dnm(x + Ts*coeff3, u)

    x_next = x + Ts*0.16667*(coeff1 + 2*coeff2 + 2*coeff3 + coeff4)

    return x_next

def setup(x0, Tf,N,RTI=False):

    ocp = AcadosOcp()
    model = export_planar_drone_ode_model(m,l,Inertia)
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx # terminal

    ocp.dims.N = N

    ocp.cost.cost_type = 'NONLINEAR_LS' # Running
    ocp.cost.cost_type_e = 'NONLINEAR_LS' # Terminal
    Q_mat = np.diag([10, 10, 10, 1, 1, 1])
    R_mat =  np.diag([0.1, 0.1])
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.yref = np.array([0,0,0,0,0,0,m*g/2,m*g/2])
    ocp.cost.yref_e = np.array([0,0,0,0,0,0])

    ocp.constraints.lbu = np.array([1,1])
    ocp.constraints.ubu = np.array([30,30])
    
    ocp.constraints.x0 = x0
    ocp.constraints.idxbu = np.array([0,1])

    ocp.constraints.lbx = np.array([-20,-20,-np.pi/2, -2,-2,-2])
    ocp.constraints.ubx = np.array([20,20,np.pi/2, 2,2,2])
    ocp.constraints.idxbx   = np.array([0,1,2,3,4,5])

    ocp.model.con_h_expr = vertcat((model.x[0]-1)**2 + (model.x[1]+1)**2, (model.x[0]-4)**2 + (model.x[1]+2)**2)
    ocp.constraints.lh = np.array([(0.5+l)**2, (0.5+l)**2])
    ocp.constraints.uh = np.array([100000, 100000])

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = 'EXACT'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_newton_iter = 10

    if RTI:
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    else:
        ocp.solver_options.nlp_solver_type = 'SQP'

    ocp.solver_options.qp_solver_cond_N = N

    ocp.solver_options.tf = Tf

    solver_json = 'acados_ocp_' + model.name + '.json'
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json)
    acados_integrator = AcadosSimSolver(ocp, json_file = solver_json)

    return acados_ocp_solver, acados_integrator

def main(use_RTI=False):

    x0 = np.array([5, -5, -0.8, 0.0, 0.0, 0.0])
    N_horizon = 20
    Tc = 0.02
    Tdnm = 0.001
    Tf = N_horizon*Tc
    
    ocp_solver,  integrator = setup(x0, Tf, N_horizon, use_RTI)

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu

    N_sim = 1000
    simX = np.zeros((N_sim+1, nx))
    simU = np.zeros((N_sim, nu))

    guessX = np.zeros((N_horizon+1, nx))
    guessU = np.zeros((N_horizon, nu))

    simX[0,:] = x0

    if use_RTI:
        t_preparation = np.zeros((N_sim))
        t_feedback = np.zeros((N_sim))
    else:
        t = np.zeros((N_sim))

    num_iter_initial = 100
    for _ in range(num_iter_initial):
        ocp_solver.solve_for_x0(x0_bar=x0)

    
    for i in range(N_sim):

        if use_RTI:
            # preparation
            ocp_solver.options_set('rti_phase', 1)
            status = ocp_solver.solve()
            t_preparation[i] = ocp_solver.get_stats('time_tot')

            # initial state
            noise = np.random.normal(0,0.01,6)
            ocp_solver.set(0,'lbx', simX[i,:])
            ocp_solver.set(0,'ubx', simX[i,:])

            # initial guess
            # for j in range(N_horizon-1):
            #     ocp_solver.set(j,'x', guessX[j+1,:])
            #     ocp_solver.set(j,'u', guessU[j+1,:])
            # ocp_solver.set(N_horizon,'x', np.array([0,0,0,0,0,0]))
            # ocp_solver.set(N_horizon-1,'u', np.array([m*g/2, m*g/2,]))


            # feedback
            ocp_solver.options_set('rti_phase', 2)
            status = ocp_solver.solve()
            t_feedback[i] = ocp_solver.get_stats('time_tot')

            simU[i, :] = ocp_solver.get(0, "u")

            for j in range(N_horizon):
                guessU[j, :] = ocp_solver.get(j, "u")
                guessX[j, :] = ocp_solver.get(j, "x")
            guessX[N_horizon, :] = ocp_solver.get(N_horizon, "x")


        else:
            simU[i, :] = ocp_solver.solve_for_x0(x0_bar = simX[i, :])
            t[i] = ocp_solver.get_stats('time_tot')
        
        # simX[i+1,:] = integrator.simulate(x=simX[i,:], u=simU[i,:])
        x_dnm = simX[i,:]
        for j in range(int(Tc/Tdnm)):
            x_dnm = update_RK4(x=x_dnm, u=simU[i,:], Ts=Tdnm)
        simX[i+1,:] = x_dnm

    if use_RTI:
        t_preparation *= 1000
        t_feedback *= 1000
        print(f'Computation time in preparation phase in ms: \
                min {np.min(t_preparation):.3f} median {np.median(t_preparation):.3f} max {np.max(t_preparation):.3f}')
        print(f'Computation time in feedback phase in ms:    \
                min {np.min(t_feedback):.3f} median {np.median(t_feedback):.3f} max {np.max(t_feedback):.3f}')

    else:
        # scale to milliseconds
        t *= 1000
        print(f'Computation time in ms: min {np.min(t):.3f} median {np.median(t):.3f} max {np.max(t):.3f}')

    # plot results
    plot_planar(np.linspace(0, (Tf/N_horizon)*N_sim, N_sim+1), simU, simX)

    ocp_solver = None


if __name__ == '__main__':
    main(use_RTI=True)






