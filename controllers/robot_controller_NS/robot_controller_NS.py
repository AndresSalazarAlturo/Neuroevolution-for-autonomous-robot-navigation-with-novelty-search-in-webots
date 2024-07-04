"""obstacle_avoidance controller."""

## Utilities
from controller import Robot, Supervisor, Node, Keyboard , Emitter, Receiver
from board_trayectory_plot import *
import time

def nn_controller(inputs, params):
    """
        Neural network with forward propagation. No activation function.
        :param inputs: List with sensor values
        :param params: List with weights and bias values
        :return left_speed_command: Left motor speed
        :return right_speed_command: Right motor speed
    """

    ### set left motor speed
    # weighted connection from left eye to left motor command
    left_speed_command = inputs[0] * params[0]
    # weighted connection from left sensor to left motor command
    left_speed_command += inputs[3] * params[1]
    # weighted connection from right eye to left motor command
    left_speed_command += inputs[1] * params[2]
    # weighted connection from right sensor to left motor command
    left_speed_command += inputs[2] * params[3]
    # add bias term to left motor command
    left_speed_command += params[4]

    ### set right motor speed
    # weighted connection from left eye to right motor command
    right_speed_command = inputs[0] * params[5]
    # weighted connection from left sensor to right motor command
    right_speed_command += inputs[3] * params[6]
    # weighted connection from right eye to right motor command
    right_speed_command += inputs[1] * params[7]
    # weighted connection from right sensor to right motor command
    right_speed_command += inputs[2] * params[8]
    # add bias term to right motor command
    right_speed_command += params[9]

    # return motor speed commands to robot's controller
    return [left_speed_command, right_speed_command]

def get_robot_behaviour(robot_trayectory):
    """
        Get the robot's behavior given the robot trajectory.
        :param robot_trayectory: List of robot's position over time.
        :return robot_bd: Set with the robot's behavior over time.
    """

    ## Robot behaviour set.
    robot_bd = set()

    for coordinates in robot_trayectory:
        robot_bd.add(board.is_within_square(coordinates, my_grid))

    print(f"Robot behavior: {robot_bd}")

    return robot_bd

def run_controller_once(robot, PARAMETERS):
    """
        Run this to read sensors, motors and use the genotype for novelty evaluation.
        :param robot: Robot instance.
        :param PARAMETERS: List with the parameters to test.
        :return robot_trajectory: Return the robot trajectory as a list.
    """

    ## Get the start time of the simulation
    start_time = robot.getTime()

    ## Set the duration of the simulation in seconds
    ## 60 seconds to test
    duration = 30

    ## Reset robot trajectory
    robot_trayectory = []

    while robot.step(TIMESTEP) != -1:

        ## Check if the current time exceeds the duration
        if robot.getTime() - start_time >= duration:
            break

        ## Get sensors data
        ## Distance sensors data
        for ps in range(len(list_ps)):
            ## The value of the sensor is subtracted to the actual value of the sensor
            ## So the value of the sensor increase when an object is getting closer
            ps_values_list[ps] = 1000 - list_ps[ps].getValue()

        ## Get GPS data
        gps_value = gps.getValues()
        # msg = "GPS values: "
        # for each_value in gps_value:
        #     msg += " {0:0.5f}".format(each_value)
        # print(msg)

        for each_value in range(len(gps_value)):
            gps_value[each_value] = round(gps_value[each_value], 5)
            # print("Test: ", gps_value[each_value])

        ## List of list with the robot trajectory. Trajectory example [x, y, z]
        robot_trayectory.append(gps_value[:2])

        # print("Robot trayectory: ", robot_trayectory)

        left_speed, right_speed = nn_controller(ps_values_list, PARAMETERS)

        ## Limit speed to avoid warnings in console
        if left_speed > 10:
            left_speed = 8

        if right_speed > 10:
            right_speed = 8

        if left_speed < 0:
            left_speed = 0

        if right_speed < 0:
            right_speed = 0

        # print("After asking for parameters")

        ## Send the velocity to the motors
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

    return robot_trayectory

def run_robot(robot):
    """
        Run robot function. Initialize motors, sensors and run the robot.
        :param robot: Robot instance.
        :return: None.
    """

    # ## Store robot trayectory
    # robot_trayectory = []

    print("Hey, before first while loop")

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(TIMESTEP) != -1:

        ## Keep waiting for a message to continue the process
        if receiver.getQueueLength() == 0:
            continue

        if receiver.getQueueLength()>0:
            # print("I got a message")
            # message = reciever.getData().decode('utf-8')
            message = receiver.getString()
            # print("message in controller: ", message)
            # message = "return_fitness"
            if message=="return_behaviour":

                ## Get robot behavior
                robot_bd = get_robot_behaviour(robot_trayectory)

                ## Transform behavior into string list to be sent
                robot_behaviour_string = [str(gene) for gene in robot_bd]
                robot_behaviour_string = ','.join(robot_behaviour_string)

                ## Send the behaviour set to GA_supervisor
                emitter.send(robot_behaviour_string.encode('utf-8'))

                print(f"Robot: behaviour for {PARAMETERS} is {robot_bd}")

                ## Reset the motors
                left_motor.setPosition(float('inf'))
                left_motor.setVelocity(0.0)

                right_motor.setPosition(float('inf'))
                right_motor.setVelocity(0.0)

                ## Stop the motors
                stop = True

            else:
                # ## Use the parameters received from GA_supervisor.py
                message_ = [float(param) for param in message.split(',')]
                # message_ = [-4.0, -4.5, 4.5, 2.5, -1.5, -5.0, 3.0, 3.5, -5.0, 1.5]
                PARAMETERS = message_
                # print("Tested parameters: ", PARAMETERS)

                ## Keep the motors working
                stop = False

                ## Run robot once
                ## This function run the controller for some time to extract the trajectory and get novelty and map
                robot_trayectory = run_controller_once(robot, PARAMETERS)

                ## Stop the bot if the supervisor asks for the novelty value
                if stop:
                    left_motor.setVelocity(0)
                    right_motor.setVelocity(0)

            ## Ask the emitter the next packet after processing the previous one.
            ## Removes the processed packet from the queue and move to the next one.
            receiver.nextPacket()

    # return robot_trayectory

if __name__ == "__main__":

    # get the time step of the current world.
    TIMESTEP = 32
    # TIMESTEP = int(my_robot.getBasicTimeStep())
    fitness = 0

    ## Create the Robot instance
    my_robot = Robot()
    # TIMESTEP = int(my_robot.getBasicTimeStep())
    # MAX_SPEED = 6.28
    # fitness = 0
    ## Motor instances
    left_motor = my_robot.getDevice('motor_1')
    right_motor = my_robot.getDevice('motor_2')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    ## GPS
    gps = my_robot.getDevice('gps_1')
    gps.enable(TIMESTEP)

    ## Initialize keyboard
    keyb = Keyboard()
    keyb.enable(TIMESTEP)

    #Initialize Emitter and Receiver
    emitter = my_robot.getDevice("emitter")
    emitter.setChannel(2)

    receiver = my_robot.getDevice("receiver")
    receiver.enable(TIMESTEP)
    receiver.setChannel(1)

    ## Create a list of distance sensors
    list_ps = []
    for ind in [0, 1, 2, 3]:
        sensor_name = 'ps' + str(ind)
        list_ps.append(my_robot.getDevice(sensor_name))
        list_ps[-1].enable(TIMESTEP)

    ## Save the distance sensor real values
    ps_values_list = [0] * 4

    ## Store robot trayectory, list [x,y]
    robot_trayectory = []

    ## Create board and grid
    board = Board()
    my_grid = board.create_board()
    ## Run the simulation
    print("Start robot controller simulation")
    run_robot(my_robot)
