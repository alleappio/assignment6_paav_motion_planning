"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))
import matplotlib
matplotlib.use('Qt5Agg')  # Or 'Agg', 'Qt5Agg', etc.
from quintic_polynomials_planner import QuinticPolynomial 
import cubic_spline_planner

SIM_LOOP = 500

show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


class FrenetPlanner():
    def __init__(self,max_speed, max_accel, max_curv, max_road_width, d_road_w, dt, max_t, min_t, target_speed, D_T_S, n_s_sample, robot_radius, K_J, K_T, K_D, K_LAT, K_LON, verbose=False):
        self.MAX_SPEED = max_speed  # maximum speed [m/s]
        self.MAX_ACCEL = max_accel  # maximum acceleration [m/ss]
        self.MAX_CURVATURE = max_curv  # maximum curvature [1/m]
        self.MAX_ROAD_WIDTH = max_road_width  # maximum road width [m]
        self.D_ROAD_W = d_road_w  # road width sampling length [m]
        self.DT = dt  # time tick [s]
        self.MAX_T = max_t  # max prediction time [s]
        self.MIN_T = min_t  # min prediction time [s]
        self.TARGET_SPEED = target_speed  # target speed [m/s]
        self.D_T_S = D_T_S  # target speed sampling length [m/s]
        self.N_S_SAMPLE = n_s_sample  # sampling number of target speed
        self.ROBOT_RADIUS = robot_radius  # robot radius [m]

        # cost weights
        self.K_J = K_J
        self.K_T = K_T
        self.K_D = K_D
        self.K_LAT = K_LAT
        self.K_LON = K_LON

        self.verbose = verbose

    def calc_frenet_paths(self, c_speed, c_accel, c_d, c_d_d, c_d_dd, s0):
        frenet_paths = []

        # generate path to each offset goal
        for di in np.arange(-self.MAX_ROAD_WIDTH, self.MAX_ROAD_WIDTH, self.D_ROAD_W):

            # Lateral motion planning
            for Ti in np.arange(self.MIN_T, self.MAX_T, self.DT):
                fp = FrenetPath()

                # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
                lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

                fp.t = [t for t in np.arange(0.0, Ti, self.DT)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Longitudinal motion planning (Velocity keeping)
                for tv in np.arange(self.TARGET_SPEED - self.D_T_S * self.N_S_SAMPLE,
                                    self.TARGET_SPEED + self.D_T_S * self.N_S_SAMPLE, self.D_T_S):
                    tfp = copy.deepcopy(fp)
                    lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)

                    tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                    tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                    # square of diff from target speed
                    ds = (self.TARGET_SPEED - tfp.s_d[-1]) ** 2

                    tfp.cd = self.K_J * Jp + self.K_T * Ti + self.K_D * tfp.d[-1] ** 2
                    tfp.cv = self.K_J * Js + self.K_T * Ti + self.K_D * ds
                    tfp.cf = self.K_LAT * tfp.cd + self.K_LON * tfp.cv

                    frenet_paths.append(tfp)

        return frenet_paths


    def calc_global_paths(self, fplist, csp):
        for fp in fplist:

            # calc global positions
            for i in range(len(fp.s)):
                ix, iy = csp.calc_position(fp.s[i])
                if ix is None:
                    break
                i_yaw = csp.calc_yaw(fp.s[i])
                di = fp.d[i]
                fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
                fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
                fp.x.append(fx)
                fp.y.append(fy)

            # calc yaw and ds
            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.hypot(dx, dy))

            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])

            # calc curvature
            for i in range(len(fp.yaw) - 1):
                fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

        return fplist


    def check_collision(self, fp, ob):
        if(len(ob[0]) is 0):
            return True
        for i in range(len(ob[:, 0])):
            d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
                 for (ix, iy) in zip(fp.x, fp.y)]

            collision = any([di <= self.ROBOT_RADIUS ** 2 for di in d])

            if collision:
                return False

        return True


    def check_paths(self, fplist, ob):
        ok_ind = []
        for i, _ in enumerate(fplist):
            # if any([v > self.MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            #     print("max speed exceeded")
            #     continue
            # elif any([abs(a) > self.MAX_ACCEL for a in
            #           fplist[i].s_dd]):  # Max accel check
            #     print("max accel exceeded")
            #     continue
            # elif any([abs(c) > self.MAX_CURVATURE for c in
            #           fplist[i].c]):  # Max curvature check
            #     print("max curv exceeded")
            #     continue
            # elif not self.check_collision(fplist[i], ob):
            #     print("collision checks failed")
            #     continue
            if not self.check_collision(fplist[i], ob):
                if(self.verbose):
                    print("collision checks failed")
                continue
            ok_ind.append(i)

        return [fplist[i] for i in ok_ind]


    def frenet_optimal_planning(self, csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob):
        fplist = self.calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)
        if(self.verbose):
            print("paths after frenet calc:",len(fplist))
        fplist = self.calc_global_paths(fplist, csp)
        fplist = self.check_paths(fplist, ob)
        if(self.verbose):
            print("paths after checks",len(fplist))

        # find minimum cost path
        min_cost = float("inf")
        best_path = None

        for fp in fplist:
            if min_cost >= fp.cf:
                min_cost = fp.cf
                best_path = fp

        return best_path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

def load_path(file_path):
    file = open(file_path, "r")
    
    xs = []
    ys = []

    while(file.readline()):
        line = file.readline()
        xs.append( float(line.split(",")[0]) )
        ys.append( float(line.split(",")[1]) )
    return xs, ys

def main():
    print(__file__ + " start!!")
    
    fp = FrenetPlanner(
        25.0,
        10.0,
        2.0 ,
        5.0 ,
        0.5,
        0.2 ,
        5.0 ,
        4.5 ,
        25.0,
        0.5,
        1,
        3.0 ,
        0.1,
        0.1,
        1.0,
        1.0,
        1.0,
    ) 

    # way points
    # wx = [0.0, 10.0, 20.5, 35.0, 70.5]
    # wy = [0.0, -6.0, 5.0, 6.5, 0.0]
    # obstacle lists
    ob = np.array([[100.0, -0.5],
                    [400.0, 0.5],
                    [570.0, 29.0],
                    [120.0, 200.0],
                    [33.0, 200.0],
                    [-70.0, 171.0]
                    ])
    # Load path and create a spline
    wx, wy = load_path("oval_trj.txt")
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    # initial state
    c_speed = 10.0  # current speed [m/s]
    c_accel = 0.0  # current acceleration [m/ss]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 20.0  # animation area length [m]

    for i in range(SIM_LOOP):
        path = fp.frenet_optimal_planning(
            csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)
        print(type(path))
        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]
        c_accel = path.s_dd[1]

        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(tx, ty)
            plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(path.x[1], path.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()
