## Utilities
import matplotlib.pyplot as plt
import numpy as np

class Square:
    """
        Create the instance of one square of the board.
    """
    def __init__(self, x, y, position):
        self.x = x
        self.y = y
        self.width = 0.125
        self.height = 0.125
        self.position = position

    def __str__(self):
        return f"Square at ({self.x}, {self.y})"

class Board:
    """
        Creates the board.
    """
    def __init__(self):
        self.width = 2
        self.height = 2
        self.square_size = 0.125

    def create_board(self):
        """
            Creates the grid. Grid is a list made of intances of Square class.
        """

        board = []
        for y in range(self.height * 8, -((self.height * 8) + 1), -1):
            for x in range(-self.width * 8, (self.width * 8) + 1):
                board.append(Square(x * self.square_size, y * self.square_size, len(board)))
        return board

    def is_within_square(self, coordinates, grid):
        """
            Change the attribute been_there to True if the coordinate is inside the coordinates of that square.
            Checks if the coordinate "x" is inside the horizontal boundaries of the square. It ensures that "x"
            is greater than or equal to the leftmost x-coordinate of the square(square.x) and less than the rightmost
            x-coordinate of the square, which is the sum of the square's x-coordinate and its width.
            It uses the same condition for "y" and the vertical boundaries.
            :param coordinates: The robot trayectory. List of lists
            :param grid: Board grid. List of lists
            :return: The position of the square in the grid.
        """

        ## Trajectory coordinates
        x, y = coordinates[0], coordinates[1]
        for square in grid:
            if square.x <= x < (square.x + square.width) and square.y <= y < (square.y + square.height):
                # print(f"Square position: {square.position}")
                return square.position

        return False

    def count_squares_with_been_there(self, grid):
        """
            Count the number of squares in grid that change the attribute been_there to True.
            :param grid: Board grid. List of lists
            :return count: Number of squares where the robot have been.
        """

        count = 0
        for square in grid:
            if square.been_there:
                count += 1
        return count

    def display_board(self, data):
        """
            Display the board with grid and the robot trajectory.
            :param data: The robot trayectory. List of lists
        """

        # Extract x and y coordinates from the list of lists
        x = [point[0] for point in data]
        y = [point[1] for point in data]

        # Create plot
        plt.figure(figsize=(10, 10))  # Set figure size
        plt.plot(x, y, color = 'red')  # Plot a line connecting the data points

        # Add horizontal and vertical lines at x=0 and y=0 to represent the axes
        plt.axhline(0, color='black', linestyle='--', linewidth=0.5)
        plt.axvline(0, color='black', linestyle='--', linewidth=0.5)

        # Set labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Robot trajectory')

        # Set custom ticks for x-axis and y-axis to change grid size
        plt.xticks(np.arange(-2, 2, 0.125))  # Set x-axis ticks at intervals of 0.125
        plt.yticks(np.arange(-2, 2, 0.125))  # Set y-axis ticks at intervals of 0.125

        # Set axis limits to ensure all quadrants are visible
        plt.xlim(-self.width/2, self.width/2)
        plt.ylim(-self.height/2, self.height/2)

        # Show grid
        plt.grid(True)

        ## Save plot
        plt.savefig("../../../Webots_Novelty_Search_Trajectory/First_Test.png")

        # Show plot
        plt.show()