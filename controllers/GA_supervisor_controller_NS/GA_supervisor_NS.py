"""GA_supervisor controller."""

"""
C:\Documents\Sussex_Files\Semester_2\Dissertation\Dissertation_Test_Codes\dissertationTestEnv\Scripts\python.exe - Root python
"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor, Keyboard, Emitter, Receiver
import numpy as np
from new_GA_NS import *
from novelty_archive import *
from Levenshtein import distance as leven_distance

# create the supervisor instance.
my_supervisor = Supervisor()
TIMESTEP = 32
# TIMESTEP = int(my_supervisor.getBasicTimeStep())
my_supervisor.step(TIMESTEP)
## Robot node that I created
my_robot_node = my_supervisor.getFromDef("ROBOT_1")

## The emitter to send genotype to my_robot
emitter = my_supervisor.getDevice("emitter")
emitter.setChannel(1)

receiver = my_supervisor.getDevice("receiver")
receiver.enable(TIMESTEP)
receiver.setChannel(2)

## Establish Sync between emitter and receiver

POPULATION_SIZE = 10
GENOTYPE_SIZE = 10
NUM_GENERATIONS = 100
## Weighs, bias bounds
weights_bias_range = np.arange(-5, 5, 0.5)

def run_seconds(t, reset_position = False):
    """
        Simulation time.
    """
    # n = 1000*t/TIMESTEP
    start_time = my_supervisor.getTime()
    while my_supervisor.step(TIMESTEP) != 1:
        if my_supervisor.getTime() - start_time >= t:
            # print("time", t)
            break
        if reset_position:
            restore_robot_position()
            my_robot_node.resetPhysics()

def getPerformanceData():
    """
        Get the robot's behaviour in that simulation.
    """
    emitter.send("return_behaviour".encode('utf-8'))
    while my_supervisor.step(TIMESTEP) != -1:

        ## Keep waiting for a message to continue the process
        if receiver.getQueueLength() == 0:
            continue

        if receiver.getQueueLength()>0:
            # message = receiver.getData().decode('utf-8')
            message = receiver.getString()
            receiver.nextPacket()
            ## Robot behaviour in simulation
            ## Transform the string received to list and then set
            robot_behaviour_list = [int(square_pos) for square_pos in message.split(',')]
            robot_behaviour_set = set(robot_behaviour_list)

        return robot_behaviour_set

def send_genotype(genotype):
    """
        Send the genotype to the robot controller and
        evaluate novelty.
        :param genotype: List with the genotype information
    """
    genotype_string = [str(gene) for gene in genotype]
    genotype_string = ','.join(genotype_string)

    # print("Genotype string: ", genotype_string)
    # test_message = "Hello"
    # test_encode = test_message.encode('utf-8')
    # print("Encode Test: ", test_encode)

    emitter.send(genotype_string.encode('utf-8'))
    # emitter.send(genotype_string.encode('utf-8'))

def restore_robot_position():
    global init_translation,init_rotation
    my_robot_translation.setSFVec3f(init_translation)
    my_robot_rotation.setSFRotation(init_rotation)

def evaluate_genotype(genotype):
    ## Send genotype to my_robot
    send_genotype(genotype)

    ## Run for 1 minute
    run_seconds(30)

    ## Get the behavior
    behaviour = getPerformanceData()

    my_robot_node.resetPhysics()
    restore_robot_position()

    run_seconds(5, True)

    my_robot_node.resetPhysics()
    restore_robot_position()

    return behaviour

def run_optimization(population):

    print("---\n")
    print("Starting Optimization")
    print("Population Size %i , Genome Size %i"%(POPULATION_SIZE,GENOTYPE_SIZE))

    ## List to store average novelty
    average_novelty_over_time = []

    ## Genotype ID
    genotype_id = 0

    ## Create novelty search archive instance
    archive = NoveltySearchArchive(4, leven_distance)

    for gen in range(NUM_GENERATIONS):

        population_novelty = []
        for ind in range(POPULATION_SIZE):
            print("-----------------------------------------------")
            print("Generation %i , Genotype %i "%(gen,ind))

            ## Get genotype from population
            genotype = population[ind]

            print("Run optimization, genotype sent: ", genotype)

            ## Evaluate genotype
            behaviour_set = evaluate_genotype(genotype)

            ## Here add the behaviour to the archive or not.
            ## Add the first behaviour to the archive
            if len(archive.archive) == 0:
                archive.insert_entry(genome=genotype, data=behaviour_set, novelty=0, genome_id=genotype_id)
                archive.add_novelty_to_behaviour(0, genotype_id)

                ## Add novelty to population novelty
                population_novelty.append(0)

                ## Update genotype ID
                genotype_id += 1

            else:
                ## When there is at least one candidate in the archive
                ## This behaviour set is the new behaviour that is going
                ## To be compared with the behaviours in the archive
                novelty, diffs = archive.compute_novelty(behaviour_set)
                # print(f"Novelty for {behaviour_set} is {novelty}")
                archive.insert_entry(genome=genotype, data=behaviour_set, novelty=novelty, genome_id=genotype_id)
                archive.add_novelty_to_behaviour(novelty, genotype_id)
                # print("---------------------------------------------")
                # print(f"Novelty archive: {archive.archive}")

                ## Add novelty to population novelty
                population_novelty.append(novelty)

                ## Update genotype ID
                genotype_id += 1

        # print(f"Novelty archive: {archive.archive}")
        ## Get the most novel and least novel behaviour.
        most_novel_genome = archive.get_most_novel()
        least_novel_genome = archive.get_least_novel()
        ## Get the average novelty
        avg_novelty_archive = archive.get_avg_novelty()

        # print("---------------------------------------------")
        # print(f"Most novel genome: {most_novel_genome}")
        # print("---------------------------------------------")
        # print(f"Least novel genome: {least_novel_genome}")
        # print("---------------------------------------------")
        # print(f"Average novelty in archive: {avg_novelty_archive}")
        # print("---------------------------------------------")
        # print(f"Population novelty: {population_novelty}")
        # print("---------------------------------------------")

        ## Store average novelty over generations
        average_novelty_over_time.append(avg_novelty_archive)

        if(gen < NUM_GENERATIONS-1):

            # population = population_reproduce_novelty(population, population_novelty, GENOTYPE_SIZE)
            population = population_reproduce_novelty(archive.archive, population, POPULATION_SIZE, GENOTYPE_SIZE)
            print("New population: ", population)

    return most_novel_genome, least_novel_genome, average_novelty_over_time, archive

def main():
    ## Initialize keyboard
    global init_translation ,init_rotation, my_robot_translation, my_robot_rotation, population

    keyb = Keyboard()
    keyb.enable(TIMESTEP)

    my_robot_translation = my_robot_node.getField("translation")
    my_robot_rotation = my_robot_node.getField("rotation")
    init_translation = (my_robot_translation.getSFVec3f())
    init_rotation = (my_robot_rotation.getSFRotation())

    population = create_random_parameters_set(POPULATION_SIZE, GENOTYPE_SIZE, weights_bias_range)

    most_novel_genome, least_novel_genome, average_novelty_over_time, novelty_archive = run_optimization(population)

    print("------------------------------------------------------")
    print("Most novel Params All Time: ", most_novel_genome)
    print("------------------------------------------------------")
    print("Least novel All Time: ", least_novel_genome)
    print("------------------------------------------------------")
    # print("Average novelty Per generation All Time: ", average_novelty_over_time)
    print("Average novelty All Time: ", sum(average_novelty_over_time)/len(average_novelty_over_time))
    print("------------------------------------------------------")
    squares_explored_most_novel = len(most_novel_genome["data"])
    print(f"Explored squares by MOST novel candidate: {squares_explored_most_novel}")
    print("------------------------------------------------------")
    squares_explored_least_novel = len(least_novel_genome["data"])
    print(f"Explored squares by LEAST novel candidate: {squares_explored_least_novel}")
    print("------------------------------------------------------")
    print(f"Final novelty archive: {novelty_archive.archive}")

    send_genotype(most_novel_genome['genome'])

    ## Restore robot's position
    print("Restore robot position last time")
    restore_robot_position()

    while my_supervisor.step(TIMESTEP) != -1:
        key = keyb.getKey()

        if key == ord('Q'):
            quit()

main()
