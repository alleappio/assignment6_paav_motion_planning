class SimulationParameters:
    dt = 0.05                   # Time step (s)
    ax = 0.0                    # Constant longitudinal acceleration (m/s^2)
    steer = 0.0                 # Constant steering angle (rad)
    sim_time = 180.0             # Simulation duration in seconds
    steps = int(sim_time / dt)  # Simulation steps (30 seconds)
    target_speed = 25.0
    controller = 'mpc'
    figures_path = 'figures/exercise2/speed2/mpc'
    vehicle_model = [
        ("rk4", "kinematic"),
    ]
    verbose = True
    frenet_verbose = False


class VehicleParameters:
    lf = 1.156                # Distance from COG to front axle (m)
    lr = 1.42                 # Distance from COG to rear axle (m)
    wheelbase = lf + lr       # Wheelbase of vehicle
    mass = 1200               # Vehicle mass (kg)
    Iz = 1792                 # Yaw moment of inertia (kg*m^2)
    max_steer = 3.14          # Maximum steering angle in radians
    min_steer = -3.14         # Maximum steering angle in radians


class PIDParameters:
    kp = 2.0                  # Proportional gain
    ki = 1.5                  # Integrative gain
    kd = 0.01                  # Derivative gain
    output_limits = (-2, 2)   # Saturation limits


class PurepursuitParameters:
    k_v = 0.25                # Speed proportional gain for Pure Pursuit
    k_c = 0.08                # Curve proportional gain for Pure Pursuit
    limit_curvature = 0.01    # Minimum heading angle for adding curve proportional gain 0.01
    look_ahead = 1.0          # Minimum look-ahead distance for Pure Pursuit


class StanleyParameters:
    k_stanley = 3.5           # Gain for cross-track error for Stanley
    k_he = 2.8                # Gain for heading error
    k_ctc = 0.6  


class MpcParameters:
    gain_mult = 1.0  
    k_x = 100.0
    k_y = 100.0
    k_theta = 11.0
    k_j = 1000000.0              
    T =  2.5                    # Horizon length in seconds
    dt = 0.05                   # Horizon timesteps
    N = int(T/dt)               # Horizon total points


class FrenetParameters:
    #MAX_SPEED = SimulationParameters.target_speed  # maximum speed [m/s]
    #MAX_ACCEL = 10.0  # maximum acceleration [m/ss]
    #MAX_CURVATURE = 2.0  # maximum curvature [1/m]
    #MAX_ROAD_WIDTH = 5.0  # maximum road width [m]
    #D_ROAD_W = 0.5  # road width sampling length [m]
    #DT = 0.2  # time tick [s]
    #MAX_T = 5.0  # max prediction time [s]
    #MIN_T = 4.5  # min prediction time [s]
    #TARGET_SPEED = SimulationParameters.target_speed  # target speed [m/s]
    #D_T_S = 0.5  # target speed sampling length [m/s]
    #N_S_SAMPLE = 1  # sampling number of target speed
    #ROBOT_RADIUS = 3.0  # robot radius [m]

    ## cost weights
    #K_J = 0.1
    #K_T = 0.1
    #K_D = 1.0
    #K_LAT = 1.0
    #K_LON = 1.0
    # Parameter
    MAX_SPEED = SimulationParameters.target_speed  # maximum speed [m/s]
    MAX_ACCEL = 10.0  # maximum acceleration [m/ss]
    MAX_CURVATURE = 2.0  # maximum curvature [1/m]
    MAX_ROAD_WIDTH = 5.0  # maximum road width [m]
    D_ROAD_W = 0.5  # road width sampling length [m]
    DT = 0.2  # time tick [s]
    MAX_T = 5.0  # max prediction time [s]
    MIN_T = 4.5  # min prediction time [s]
    TARGET_SPEED = SimulationParameters.target_speed  # target speed [m/s]
    D_T_S = 0.5  # target speed sampling length [m/s]
    N_S_SAMPLE = 1  # sampling number of target speed
    ROBOT_RADIUS = 3.0  # robot radius [m]

    # cost weights
    K_J = 0.1
    K_T = 0.5
    K_D = 0.5
    K_LAT = 1.0
    K_LON = 0.5
