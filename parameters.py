class SimulationParameters:
    dt = 0.05                   # Time step (s)
    ax = 0.0                    # Constant longitudinal acceleration (m/s^2)
    steer = 0.0                 # Constant steering angle (rad)
    sim_time = 90.0             # Simulation duration in seconds
    steps = int(sim_time / dt)  # Simulation steps (30 seconds)
    target_speed = 25.0
    controller = 'purepursuit'
    figures_path = 'figures/general'
    vehicle_model = [
        ("rk4", "nonlinear"),
    ]
    verbose = False


class VehicleParameters:
    lf = 1.156                # Distance from COG to front axle (m)
    lr = 1.42                 # Distance from COG to rear axle (m)
    wheelbase = lf + lr       # Wheelbase of vehicle
    mass = 1200               # Vehicle mass (kg)
    Iz = 1792                 # Yaw moment of inertia (kg*m^2)
    max_steer = 3.14          # Maximum steering angle in radians
    min_steer = -3.14         # Maximum steering angle in radians


class PIDParameters:
    kp = 2.3                  # Proportional gain
    ki = 1.7                  # Integrative gain
    kd = 0.01                  # Derivative gain
    output_limits = (-2, 2)   # Saturation limits


class PurepursuitParameters:
    k_v = 0.7                # Speed proportional gain for Pure Pursuit
    k_c = 0.04                # Curve proportional gain for Pure Pursuit
    limit_curvature = 0.001    # Minimum heading angle for adding curve proportional gain
    look_ahead = 1.0          # Minimum look-ahead distance for Pure Pursuit


class StanleyParameters:
    k_stanley = 2.9           # Gain for cross-track error for Stanley
    k_he = 1.1                # Gain for heading error
    k_ctc = 2.9               # Gain for cross-trac correction 


class MpcParameters:
    gain_mult = 1
    T =  1                    # Horizon length in seconds
    dt = 0.05                 # Horizon timesteps
    N = int(T/dt)             # Horizon total points

class FrenetParameters:
    SIM_LOOP = 500

    # Parameter
    MAX_SPEED = SimulationParameters.target_speed       # maximum speed [m/s]
    MAX_ACCEL = 10.0                                    # maximum acceleration [m/ss]
    MAX_CURVATURE = 2.0                                 # maximum curvature [1/m]
    MAX_ROAD_WIDTH = 5.0                                # maximum road width [m]
    D_ROAD_W = 0.5                                      # road width sampling length [m]
    DT = 0.2                                            # time tick [s]
    MAX_T = 5.0                                         # max prediction time [s]
    MIN_T = 4.5                                         # min prediction time [s]
    TARGET_SPEED = SimulationParameters.target_speed    # target speed [m/s]
    D_T_S = 0.5                                         # target speed sampling length [m/s]
    N_S_SAMPLE = 1                                      # sampling number of target speed
    ROBOT_RADIUS = 3.0                                  # robot radius [m]

    # cost weights
    K_J = 0.5
    K_T = 0.1
    K_D = 1.0
    K_LAT = 0.5
    K_LON = 1.5

    show_animation = True
