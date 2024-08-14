from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_planar_drone_ode_model(mass, length, Inertia) -> AcadosModel:

    model_name = 'planar_drone'

    m = mass
    g = 9.81
    l = length
    I = Inertia

    x = SX.sym('x')
    y = SX.sym('y')
    theta = SX.sym('theta')
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    vtheta = SX.sym('vtheta')


    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    theta_dot = SX.sym('theta_dot')
    vx_dot = SX.sym('vx_dot')
    vy_dot = SX.sym('vy_dot')
    vtheta_dot = SX.sym('vtheta_dot')

    u1 = SX.sym('u1')
    u2 = SX.sym('u2')


    # State:  x_
    x_ = vertcat(x,y,theta,vx,vy,vtheta)
    x_dot_ = vertcat(x_dot,y_dot,theta_dot,vx_dot,vy_dot,vtheta_dot)
    u_ = vertcat(u1,u2)

    f_expl =  vertcat(vx,
                vy,
                vtheta,
                -(u1+u2)*sin(theta)/m,
                (u1+u2)*cos(theta)/m - g,
                l*(-u1+u2)/I
                )
    
    f_impl = x_dot_ - f_expl
    
    model = AcadosModel()

    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    model.x = x_
    model.xdot = x_dot_
    model.u = u_
    model.name = model_name

    return model

