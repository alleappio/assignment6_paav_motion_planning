import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib
from simulation import Simulation
import pid
import purepursuit
import stanley
import mpc
import cubic_spline_planner
import math
import frenet_optimal_trajectory as fp 
from parameters import SimulationParameters as sim_params
from parameters import VehicleParameters as vehicle_params
from parameters import PIDParameters as PID_params
from parameters import PurepursuitParameters as PP_params
from parameters import StanleyParameters as stanley_params
from parameters import MpcParameters as MPC_params
from parameters import FrenetParameters as frenet_params

matplotlib.use('Qt5Agg')  # Or 'Agg', 'Qt5Agg', etc.

# Create instance of PID for Longitudinal Control
long_control_pid = pid.PIDController(kp=PID_params.kp, ki=PID_params.ki, kd=PID_params.kd, output_limits=PID_params.output_limits)

pp_controller = purepursuit.PurePursuitController(vehicle_params.wheelbase, vehicle_params.max_steer)
stanley_controller = stanley.StanleyController(stanley_params.k_stanley, vehicle_params.lf, vehicle_params.max_steer, stanley_params.k_he, stanley_params.k_ctc)
mpc_controller = mpc.MPC(MPC_params.T, MPC_params.dt, MPC_params.N, vehicle_params.max_steer, vehicle_params.min_steer, MPC_params.gain_mult)

#Planning
frenet_planner = fp.FrenetPlanner(
    frenet_params.MAX_SPEED,
    frenet_params.MAX_ACCEL,
    frenet_params.MAX_CURVATURE, 
    frenet_params.MAX_ROAD_WIDTH,
    frenet_params.D_ROAD_W, 
    frenet_params.DT,
    frenet_params.MAX_T,
    frenet_params.MIN_T,
    frenet_params.TARGET_SPEED,
    frenet_params.D_T_S,
    frenet_params.N_S_SAMPLE,
    frenet_params.ROBOT_RADIUS,
    frenet_params.K_J,
    frenet_params.K_T,
    frenet_params.K_D,
    frenet_params.K_LAT,
    frenet_params.K_LON,
    True
)

# obstacle lists
ob = np.array([[100.0, -0.5],
                [400.0, 0.5],
                [570.0, 29.0],
                [600.0,100.0],
                [120.0, 200.0],
                [33.0, 200.0],
                [-70.0, 171.0],
                [-100.0, 100.0]
                ])

def load_path(file_path):
    file = open(file_path, "r")
    
    xs = []
    ys = []

    while(file.readline()):
        line = file.readline()
        xs.append( float(line.split(",")[0]) )
        ys.append( float(line.split(",")[1]) )
    return xs, ys

# Load path and create a spline
xs, ys = load_path("oval_trj.txt")
path_spline = cubic_spline_planner.Spline2D(xs, ys)

def point_transform(trg, pose, yaw):

    local_trg = [trg[0] - pose[0], trg[1] - pose[1]]

    return local_trg

def plot_comparison(results, labels, title, xlabel, ylabel, show = False):
    """ Plot comparison of results for a specific state variable. """
    plt.figure(figsize=(10, 6))
    for i, result in enumerate(results):
        plt.plot(result, label=labels[i])
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.savefig(f"{sim_params.figures_path}/{title}.png")
    if(show):
        plt.show()
    else:
        plt.close()

def plot_trajectory(x_vals, y_vals, labels, path_spline, frenet_x_results, frenet_y_results, show=False):
    """ Plot 2D trajectory (x vs y) for all simulation configurations and path_spline trajectory. """
    plt.figure(figsize=(10, 6))
    
    # Plot the simulation trajectories
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])

    # Plot the frenet planner trajectory
    for i in range(len(frenet_x_results)):
        plt.plot(frenet_x_results[i], frenet_y_results[i], linestyle="--", color="green")

    # Plot the path_spline trajectory
    spline_x = [path_spline.calc_position(s)[0] for s in np.linspace(0, path_spline.s[-1], 1000)]
    spline_y = [path_spline.calc_position(s)[1] for s in np.linspace(0, path_spline.s[-1], 1000)]
    plt.plot(spline_x, spline_y, label="Path Spline", linestyle="--", color="red")

    # Plot obstacles
    if(len(ob[0]) is not 0):
        plt.scatter(ob[:, 0], ob[:, 1], c='black', label="Obstacles", marker='x')
    
    # Customize plot
    plt.title("2D Trajectory Comparison")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    #plt.show()
    plt.savefig(f"{sim_params.figures_path}/trajectory.png")
    if(show):
        plt.show()

def run_simulation(ax, steer, dt, integrator, model, steps=500):
    """ Run a simulation with the given parameters and return all states. """

    # Initialize the simulation
    sim = Simulation(vehicle_params.lf, vehicle_params.lr, vehicle_params.mass, vehicle_params.Iz, dt, integrator=integrator, model=model)

    # Storage for state variables and slip angles
    x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals = [], [], [], [], [], []
    alpha_f_vals, alpha_r_vals = [], []  # Slip angles
    frenet_x, frenet_y = [], []

    mpc_controller.casadi_model()

    # states for Frenet-planner
    c_speed = 0.0  # current speed [m/s]
    c_accel = 0.0  # current acceleration [m/ss]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    for step in range(steps):
    
        # Print time
        print("Time:", step*dt)

        # Calculate ax to track speed
        ax = long_control_pid.compute(sim_params.target_speed, sim.vx, dt)

        ############# Frenet-planner

        frenet_path = frenet_planner.frenet_optimal_planning(
            path_spline, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)
        
        if(frenet_path is None):
            if(sim_params.verbose):
                print("None available paths found from Frenet...")
            break

        frenetpath_spline = cubic_spline_planner.Spline2D(frenet_path.x, frenet_path.y)

        # Update actual frenet-frame position in the spline
        # aka longitudinal position and actual lateral error
        actual_position = sim.x, sim.y
        actual_pose = sim.x, sim.y, sim.theta
        path_spline.update_current_s(actual_position)
        frenetpath_spline.update_current_s(actual_position)

        # get actual position projected on the path/spline
        global_position_projected = path_spline.calc_position(path_spline.cur_s)
        prj = [ global_position_projected[0], global_position_projected[1] ]
        local_error = point_transform(prj, actual_position, sim.theta)

        local_position_projected = frenetpath_spline.calc_position(frenetpath_spline.cur_s)
        prj = [ local_position_projected[0], local_position_projected[1] ]
        frenetlocal_error = point_transform(prj, actual_position, sim.theta)

        nearest_idx = 0
        nearest_distance = abs(path_spline.cur_s - frenet_path.s[0])
        for i in range(len(frenet_path.s)):
            dist = abs(path_spline.cur_s - frenet_path.s[i])
            if(dist < nearest_distance):
                nearest_distance = dist
                nearest_idx = i

        s0 = frenet_path.s[nearest_idx]
        c_d = frenet_path.d[nearest_idx]
        c_d_d = frenet_path.d_d[nearest_idx]
        c_d_dd = frenet_path.d_dd[nearest_idx]
        c_speed = frenet_path.s_d[nearest_idx]
        c_accel = frenet_path.s_dd[nearest_idx]

        ################

        if(abs(local_error[1]) > 4.0):
            if(sim_params.verbose):
                print("Lateral error is higher than 4.0... ending the simulation")
                print("Lateral error: ", local_error[1])
            break

        # get target pose
        Lf = PP_params.k_v * sim.vx + PP_params.look_ahead 
        if(abs(path_spline.calc_curvature(path_spline.cur_s)) > PP_params.limit_curvature):
            Lf += PP_params.k_c / abs(path_spline.calc_curvature(path_spline.cur_s))

        s_pos = frenetpath_spline.cur_s + Lf 

        trg = frenetpath_spline.calc_position(s_pos)
        trg = [ trg[0], trg[1] ]
        pp_position = actual_position
        # Adjust CoG position to the rear axle position for PP
        pp_position = actual_position[0] + vehicle_params.lr * math.cos(sim.theta), actual_position[1] + vehicle_params.lr * math.sin(sim.theta)
        loc_trg = point_transform(trg, pp_position, sim.theta)

        # Calculate steer to track path
        
        ####### Pure Pursuit
        # steer = 0
        # Compute the look-ahead distance
        if(sim_params.controller == 'purepursuit'):
            steer = pp_controller.compute_steering_angle(loc_trg, sim.theta, Lf)
        
        ###### Stanley
        #TO-DO: Move actual position (CoG) to the front axle for stanley
        # Adjust CoG position to the front axle position
        if(sim_params.controller == 'stanley'):
            px_front = local_position_projected[0] + vehicle_params.lf * math.cos(sim.theta)
            py_front = local_position_projected[1] + vehicle_params.lf * math.sin(sim.theta)
            actual_pose = px_front, py_front, sim.theta
            stanley_target = px_front, py_front, frenetpath_spline.calc_yaw(frenetpath_spline.cur_s)
            steer = stanley_controller.compute_steering_angle(actual_pose, stanley_target, sim.vx)

        ###### MPC
        if(sim_params.controller == 'mpc'):
            # get future horizon targets pose
            targets = [ ]
            s_pos = frenetpath_spline.cur_s
            for i in range(mpc_controller.N):
                step_increment = (sim.vx)*dt
                trg = frenetpath_spline.calc_position(s_pos)
                t_yaw = frenetpath_spline.calc_yaw(s_pos)
                trg = [ trg[0], trg[1], t_yaw ]
                targets.append(trg)
                s_pos += step_increment
            steer = mpc_controller.opt_step(targets, sim)

        # Make one step simulation via model integration
        sim.integrate(ax, float(steer))
        
        # Append each state to corresponding list
        x_vals.append(sim.x)
        y_vals.append(sim.y)
        theta_vals.append(sim.theta)
        vx_vals.append(sim.vx)
        vy_vals.append(sim.vy)
        r_vals.append(sim.r)

        # Calculate slip angles for front and rear tires
        alpha_f = steer - np.arctan((sim.vy + sim.l_f * sim.r) / max(0.5, sim.vx))  # Front tire slip angle
        alpha_r = -(np.arctan(sim.vy - sim.l_r * sim.r) / max(0.5, sim.vx))         # Rear tire slip angle

        alpha_f_vals.append(alpha_f)
        alpha_r_vals.append(alpha_r)

        frenet_x.append(frenet_path.x[0])
        frenet_y.append(frenet_path.y[0])

    return x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals, alpha_f_vals, alpha_r_vals, frenet_x, frenet_y

def main():
    if(not os.path.exists(sim_params.figures_path)):
        os.makedirs(sim_params.figures_path)
    # List of configurations
    configs = [
        ("rk4", "nonlinear"),
    ]

    # Run each simulation and store the results
    all_results = []
    actual_state = []
    labels = []
    for integrator, model in sim_params.vehicle_model:
        actual_state = run_simulation(sim_params.ax, sim_params.steer, sim_params.dt, integrator, model, sim_params.steps)
        all_results.append(actual_state)
        labels.append(f"{integrator.capitalize()} - {model.capitalize()}")

    # Separate each state for plotting
    x_results = [result[0] for result in all_results]
    y_results = [result[1] for result in all_results]
    theta_results = [result[2] for result in all_results]
    vx_results = [result[3] for result in all_results]
    vy_results = [result[4] for result in all_results]
    r_results = [result[5] for result in all_results]
    alpha_f_results = [result[6] for result in all_results]
    alpha_r_results = [result[7] for result in all_results]
    frenet_x_results = [result[8] for result in all_results]
    frenet_y_results = [result[9] for result in all_results]

    # Plot comparisons for each state variable
    plot_trajectory(x_results, y_results, labels, path_spline, frenet_x_results, frenet_y_results, show=True)
    plot_comparison(theta_results, labels, "Heading Angle Comparison", "Time Step", "Heading Angle (rad)", show=True)
    plot_comparison(vx_results, labels, "Longitudinal Velocity Comparison", "Time Step", "Velocity (m/s)", show=True)
    plot_comparison(vy_results, labels, "Lateral Velocity Comparison", "Time Step", "Lateral Velocity (m/s)", show=True)
    plot_comparison(r_results, labels, "Yaw Rate Comparison", "Time Step", "Yaw Rate (rad/s)", show=True)
    plot_comparison(alpha_f_results, labels, "Front Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Front", show=True)
    plot_comparison(alpha_r_results, labels, "Rear Slip Angle Comparison", "Time Step", "Slip Angle (rad) - Rear", show=True)

if __name__ == "__main__":
    main()
