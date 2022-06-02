import math
import matplotlib.pyplot as plt
import argparse

angle_of_rocket = []
real_angle_of_tvc = []
angle_of_tvc = []
time = []
angle_velocity = []
angle_acceleration = []
fx = []
dt = 0.001
proportional = 0
derivative = 0
integral = 0


def setup(args):
    global kp, ki, kd
    global moment_of_inertia, distance_to_center_of_mass, thrust, mass
    global servo_delay

    kp = args.p
    ki = args.i
    kd = args.d

    moment_of_inertia = args.moment
    distance_to_center_of_mass = args.distance
    thrust = args.thrust
    mass = args.mass

    servo_delay = args.delay

    for x in range(0, int(servo_delay / dt)):
        angle_of_rocket.append(12)
        angle_of_tvc.append(0)
        angle_velocity.append(0)
        angle_acceleration.append(0)
        fx.append(0)
        time.append(x*dt)


def getFx(tvc_angle, iteration):
    if tvc_angle[iteration] > 10:
        tvc_angle[iteration] = 10
    elif tvc_angle[iteration] < -10:
        tvc_angle[iteration] = -10
    fx.append(-thrust *
              math.sin(math.radians(tvc_angle[iteration-int(servo_delay/dt)])))
    return -thrust*math.sin(math.radians(tvc_angle[iteration-int(servo_delay/dt)]))


def getAngle(fx, iteration):
    angle_acceleration.append(math.degrees(
        (fx * distance_to_center_of_mass) / moment_of_inertia))
    angle_velocity.append(
        angle_velocity[iteration] + angle_acceleration[iteration + 1] * dt)
    angle_of_rocket.append(
        angle_of_rocket[iteration] + angle_velocity[iteration + 1] * dt)


def loop(iteration):
    global integral
    fx = getFx(angle_of_tvc, iteration)
    getAngle(fx, iteration)
    proportional = kp * angle_of_rocket[iteration+1]
    integral += ki * angle_of_rocket[iteration+1] * dt
    derivative = kd * \
        (angle_of_rocket[iteration+1] - angle_of_rocket[iteration]) / dt
    angle_of_tvc.append(proportional + integral + derivative)
    time.append(time[iteration] + dt)


def plot():
    plt.figure('TVC Simulator')

    grid = plt.GridSpec(2, 2, hspace=0.3)

    plt.subplot(grid[0, 0])
    plt.plot(time, angle_of_rocket, color='red')
    plt.title("Rocket angle")
    plt.xlabel("time (s)")
    plt.ylabel("angle (deg)")

    plt.subplot(grid[0, 1])
    plt.plot(time, angle_of_tvc, color='red')
    plt.title("TVC angle")
    plt.xlabel("time (s)")
    plt.ylabel("angle (deg)")

    plt.subplot(grid[1, 0])
    plt.plot(time, angle_velocity, color='red')
    plt.title("Angle velocity")
    plt.xlabel("time (s)")
    plt.ylabel("angle velocity (deg/s)")

    plt.subplot(grid[1, 1])
    plt.plot(time, fx, color='red')
    plt.title("Angle acceleration")
    plt.xlabel("time (s)")
    plt.ylabel("angle acceleration (deg/s^2)")

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='PID Simulator for Thrust-Vector-Control rockets.')
    parser.add_argument(
        '-p', help='Proportional element of PID', type=float, required=True)
    parser.add_argument('-i', help='Integral element of PID',
                        type=float, required=True)
    parser.add_argument('-d', help='Derivative element of PID',
                        type=float, required=True)

    parser.add_argument(
        '-angle', help='Initial rocket angle (deg)', type=float, required=True)

    parser.add_argument(
        '-thrust', help='Thrust of the engine (N)', type=float, required=True)
    parser.add_argument(
        '-moment', help='Moment of intertia (kg*m^2)', type=float, required=True)
    parser.add_argument(
        '-mass', help='Mass of the rocket (kg)', type=float, required=True)
    parser.add_argument(
        '-distance', help='Distance from the engine to the center of mass (m)', type=float, required=True)
    parser.add_argument(
        '-delay', help="Delay between sending a command and actuator's movement (s)", type=float, required=True)
    parser.add_argument(
        '-length', help="Length of the simulation (s)", type=float, required=True)

    args = parser.parse_args()
    setup(args)

    for x in range(int(servo_delay / dt) - 1, int(args.length / dt)):
        loop(x)

    plot()
