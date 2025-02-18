import pandas as pd
import random
import numpy as np

MUTATION_PROBABILITY = 0.1
FIXED_BIAS = False
BOUNDS = 5
ELITE_PART = 0.4

def create_random_parameters_set(pop_size, geno_size, weights_bias_range):
    """
        Create random parameters set.
        :param weights_bias_range: Range with the possible values
        :return population: List of list with the genotypes
    """
    population = [0] * pop_size
    for pop_ind in range(pop_size):
        genotype = [0] * geno_size
        for ind in range(len(genotype)):
            if FIXED_BIAS:
                genotype[ind] = random.choice(weights_bias_range)
                genotype[4] = 2
                genotype[9] = 2
            else:
                genotype[ind] = random.choice(weights_bias_range)
        population[pop_ind] = genotype
    return population

def create_random_single_parameters_set(weights_bias_range):
    """
        Create random parameters set.
        :param weights_bias_range: Range with the possible values
        :return population: List of list with the genotypes
    """

    genotype = [0] * 10
    for ind in range(len(genotype)):
        genotype[ind] = random.choice(weights_bias_range)

    return genotype

def random_number_close_range(current_value, x, bounds, int_value = False):
    """
        Take the current value, add and subtract a constant value x to generate a randon
        value in the range(current_value - x, current_value + x)
        :param current_value: Current value that is the set to be the main value in the range
        :param x: Value to add and subtract to current_value

        :return: Random number in range(current_value - x, current_value + x)
    """

    lower_bound = current_value - x
    upper_bound = current_value + x

    # if lower_bound < 0:
    #     lower_bound = 0

    if int_value:
        ## Generate random integer number between lower and upper bounds
        random_rational_number = random.randint(lower_bound, upper_bound)

    else:
        ## Generate random rational number between lower and upper bounds
        close_range = np.arange(lower_bound, upper_bound, 0.5)
        # print("Close range value: ", close_range)
        random_rational_number = random.choice(close_range)
        while random_rational_number < -bounds or random_rational_number > bounds:
            random_rational_number = random.choice(close_range)

    return random_rational_number

def population_reproduce(p,fitness, n_genes):
    """
        Create new population based on the fitness.
        :param p: Population - List.
        :param fitness: Population fitness - List.
        :param n_genes: Number of genes in my genotype - Int.
    """
    pop_size = len(p)
    new_p = []

    dataframe = pd.DataFrame({"Param":p,"Fitness":fitness})
    dataframe = dataframe.sort_values(['Fitness'], ascending=False)
    dataframe = dataframe.reset_index(drop=True)

    # print("Data frame: ", dataframe)

    # print("input population: ", p)

    sorted_p = dataframe['Param'].tolist()

    # print("Sorted parameters: ", sorted_p)

    ## Selection
    elite_part = round(ELITE_PART * pop_size)
    new_p = new_p + sorted_p[:elite_part]

    for i in range(pop_size-elite_part):
        mom = p[random.randint(0, pop_size - 1)]
        dad = p[random.randint(0, pop_size - 1)]
        child = crossover(mom, dad, n_genes, p)
        child = mutate(child, n_genes, FIXED_BIAS)
        new_p.append(child)

    return new_p

# def population_reproduce_novelty(novelty_archive, p,novelty, n_genes):
#     """
#         Create new population based on the novelty.
#         :param p: Population - List.
#         :param novelty: Population novelty - List.
#         :param n_genes: Number of genes in my genotype - Int.
#     """
#     pop_size = len(p)
#     new_p = []

#     dataframe = pd.DataFrame({"Param":p,"Novelty":novelty})
#     dataframe = dataframe.sort_values(['Novelty'], ascending=False)
#     dataframe = dataframe.reset_index(drop=True)

#     # print("Data frame: ", dataframe)

#     # print("input population: ", p)

#     sorted_p = dataframe['Param'].tolist()

#     # print("Sorted parameters: ", sorted_p)

#     ## Selection
#     elite_part = round(ELITE_PART * pop_size)
#     new_p = new_p + sorted_p[:elite_part]

#     for i in range(pop_size-elite_part):
#         mom = p[random.randint(0, pop_size - 1)]
#         dad = p[random.randint(0, pop_size - 1)]
#         child = crossover(mom, dad)
#         child = mutate(child, n_genes, FIXED_BIAS)
#         new_p.append(child)

#     return new_p

def population_reproduce_novelty(novelty_archive, p, pop_size, n_genes):
    """
        Create new population based on the novelty.
        :param p: Population - List.
        :param novelty: Population novelty - List.
        :param n_genes: Number of genes in my genotype - Int.
    """

    new_p = []

    # Sort the list by 'novelty' key in descending order
    sorted_genomes = sorted(novelty_archive, key=lambda x: x['novelty'], reverse=True)

    # Extract just the genome identifiers in sorted order
    sorted_parameters = [genome['genome'] for genome in sorted_genomes]

    # print(f"Sorted parameters: {sorted_parameters}")

    ## Selection
    elite_part = len(novelty_archive)
    new_p = new_p + sorted_parameters

    for i in range(pop_size-elite_part):
        mom = p[random.randint(0, pop_size - 1)]
        dad = p[random.randint(0, pop_size - 1)]
        child = crossover(mom, dad)
        child = mutate(child, n_genes, FIXED_BIAS)
        new_p.append(child)

    return new_p

def population_get_fittest(p,f):

    f = np.array(f)
    p = np.array(p)

    pop_best_fitness = max(f)
    # print("max fitness", pop_best_fitness)
    pop_best_fitness_pos = np.argmax(f)

    # print("pos best fitness position: ", pop_best_fitness_pos)

    pop_best_params = p[pop_best_fitness_pos]
    # print("best params: ", pop_best_params)

    return pop_best_params, pop_best_fitness

def population_get_average_fitness(f):
    """
        Get population average fitness.
        :param f: List with population fitness.
    """
    return sum(f)/len(f)

def crossover(p1,p2):

    crossover = []
    locii = [random.randint(0,8) for _ in range(len(p1))]

    for i in range(len(p1)):
        if locii[i]>4:
            crossover.append(p2[i])
        else:
            crossover.append(p1[i])

    if FIXED_BIAS:
        crossover[4] = 2
        crossover[9] = 2

    return crossover

def mutate(child, n_genes, FIXED_BIAS):
    """
        Implement mutation.
    """

    if FIXED_BIAS:

        ## Set bias values to 2
        child[4] = 2
        child[9] = 2

        for gene_no in range(n_genes):
            if np.random.rand() < MUTATION_PROBABILITY:
                ## Gene 4 is bias for left motor
                ## Gene 9 is bias for right motor
                if gene_no == 4 or gene_no == 9:
                    child[gene_no] = child[gene_no]
                else:
                    child[gene_no] = random_number_close_range(child[gene_no], 1, BOUNDS)
    else:
        for gene_no in range(n_genes):
            if np.random.rand() < MUTATION_PROBABILITY:
                child[gene_no] = random_number_close_range(child[gene_no], 1, BOUNDS)

    return child

