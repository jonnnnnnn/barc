#!/usr/bin/env julia

using RobotOS
@rosimport std_msgs.msg: Float32
@rosimport barc.msg: ECU, Encoder, Ultrasound, Z_KinBkMdl
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using std_msgs.msg
using JuMP
using Ipopt
using Polyhedra
using CDDLib
using QHull
using Conda
using PyCall

# -------------------------------------------------------------------
#                           MPC PARAMETERS
# -------------------------------------------------------------------

# Number of states / Number of inputs
nx = 4
nu = 2

# Horizons
n = 7           # MPC Horizon (CFTOC Horizon)

# Cost matrices
Q = [10 0 0 0; 0 10 0 0; 0 0 0 0; 0 0 0 10]
R = [0.1 0; 0 0.1]

# Hardware parameters
dt = 0.2
lr = 1.738
lf = 1.738

# Reference trajectory (sample trajectory)
t = linspace(0,1.570796,n)
xref = zeros(nx,n)
xref[1,:] = 10*sin.(t)
xref[2,:] = 10*cos.(t)
xref[3,:] = ones(1,n)
xref[4,:] = -1.570796*sin.(t)

# Box constraints on input, A*u <= B
Au = [1 0; -1 0; 0 1; 0 -1]
Bu = [10; 10; 0.524; 0.524]

# Allowable points (1's from image processing)
pts = [15 15; -15 15; -15 -15; 15 -15]
hull = chull(pts)                   # Convex hull of allowable points
H = SimpleHRepresentation(polyhedron(SimpleVRepresentation(hull.points),CDDLibrary()))         # H-representation of convex hull
Ax = H.A                            # A-matrix for A*x <= B
Bx = H.b                            # B-vector for A*x <= B

# -------------------------------------------------------------------
#                         SET UP CFTOC MODEL
# -------------------------------------------------------------------

m = Model(solver = IpoptSolver(print_level=0))

# Variables
@variable(m,x[1:nx,1:n+1])              # State vectors (nx x n+1)
@variable(m,u[1:nu,1:n])                # Input vectors (nu x n)

# Objective Function
@objective(m, :Min, sum(Q[1,1]*(x[1,i+1]-xref[1,i])^2 +             Q[2,2]*(x[2,i+1]-xref[2,i])^2 + Q[3,3]*(x[3,i+1]-xref[3,i])^2 + Q[4,4]*(x[4,i+1]-xref[4,i])^2 + R[1,1]*u[1,i]^2 + R[2,2]*u[2,i]^2 for i in n:n))

# Constraints
for i in 1:n+1
    @constraint(m, Ax*x[1:2,i] .<= Bx)
end

for i in 1:n
    @constraint(m, Au*u[:,i] .<= Bu)
end

@NLparameter(m, x0     == 0); @NLconstraint(m, x[1,1]     == x0);
@NLparameter(m, y0     == 0); @NLconstraint(m, x[2,1]     == y0);
@NLparameter(m, psi0   == 0); @NLconstraint(m, x[3,1]     == psi0);
@NLparameter(m, v0     == 0); @NLconstraint(m, x[4,1]     == v0);

for i in 1:n                            # Dynamics constraint
    @NLconstraint(m, x[1,i+1] == x[1,i] + dt*x[3,i]*cos(x[4,i]+atan(lr*tan(u[2,i])/(lf+lr))))
    @NLconstraint(m, x[2,i+1] == x[2,i] + dt*x[3,i]*sin(x[4,i]+atan(lr*tan(u[2,i])/(lf+lr))))
    @NLconstraint(m, x[3,i+1] == x[3,i] + dt*u[1,i])
    @NLconstraint(m, x[4,i+1] == x[4,i] + dt*x[3,i]/lr*sin(atan(lr*tan(u[2,i])/(lf+lr))))
end

# -------------------------------------------------------------------
#                             INITIAL SOLVE
# -------------------------------------------------------------------

println("Initial solve...")
solve(m)
println("Initial solve complete.")

# -------------------------------------------------------------------
#                             ROS FUNCTIONS
# -------------------------------------------------------------------

function callback(msg::Z_KinBkMdl)
    # Assign new initial condition for CFTOC 
    setValue(x0,    msg.x)
    setValue(y0,    msg.y)
    setValue(psi0,  msg.psi)
    setValue(v0,    msg.v)
end

function loop(pub_obj)
    loop_rate = Rate(10)
    while ! is_shutdown()

        # -----------------------------------------------------------
        #                      RUNNING MPC
        # -----------------------------------------------------------

        solve(m)                                # Solve CFTOC
        xOpt = getvalue(x)
        uOpt = getvalue(u)
        JOpt = getobjectivevalue(m)

        cmdOpt = ECU(uOpt[1,1], uOpt[2,1])      # Command to publish

        publish(pub_obj,cmdOpt)                 # Publish command to ECU
        rossleep(loop_rate)

    end
end

function main()
    init_node("MPC_node")
    pub = Publisher("ecu", ECU, queue_size=10)
    sub = Subscriber("state_estimate", Z_KinBkMdl, callback, queue_size=10)
    loop(pub)
end

if ! isinteractive()
    main()
end