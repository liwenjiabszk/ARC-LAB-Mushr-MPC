#include <acado_code_generation.hpp>

int main(){
    
    USING_NAMESPACE_ACADO

    DifferentialState                x,y,theta;
    Control                          v,delta;

    const double t_start =    0.0;
    const double t_end   =   2.5;
    const double l       =   0.25;
    
    DifferentialEquation  f;

    f << dot(x) == v*cos(theta);
    f << dot(y) == v*sin(theta);
    f << dot(theta) == v*tan(delta)/l;

    DMatrix P(3,3);
    P.setZero();
    	P(0,0) = 1;
    	P(1,1) = 1;
	P(2,2) = 1;

    DVector sr(3);
    sr.setAll(0.0);

    DVector ur(2);
    ur.setAll(0.0);
    ur(0) = 0.5;


    DMatrix W(5,5);
    W.setZero();
        W(0,0)=1000;
        W(1,1)=1000;
        W(2,2)=1;
        W(3,3)=1;
        W(4,4)=1;
        
    Function state;
    Function control;
    Function state_control;
    
    state << x;
    state << y;   
    state << theta;
    control << v;
    control << delta;
    state_control << x << y << theta << v << delta;

    OCP ocp( t_start, t_end, 10);

    ocp.minimizeLSQEndTerm( P, state);
    ocp.minimizeLSQ( W, state_control);
    ocp.subjectTo( f );

    ocp.subjectTo( 0 <= v <= 0.5);
    ocp.subjectTo( -0.34 <= delta <= 0.34);

    OCPexport mpc( ocp );
    
    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_RK4);
    mpc.set(NUM_INTEGRATOR_STEPS, 10);

    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(HOTSTART_QP, YES);
    mpc.set(GENERATE_TEST_FILE, YES);
    mpc.set(GENERATE_MAKE_FILE, YES);
    mpc.set(GENERATE_MATLAB_INTERFACE, YES);
    mpc.set(GENERATE_SIMULINK_INTERFACE, YES);

    if (mpc.exportCode( "mushr_mpc_continuous" ) != SUCCESSFUL_RETURN)
	    exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}
