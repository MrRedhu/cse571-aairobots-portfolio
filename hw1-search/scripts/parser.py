import argparse
import time

# Create the argument parser.
parser = argparse.ArgumentParser()
parser.add_argument("--dimension", 
    default="3x6",
    type=str,
    help="The dimension 'nxn' of the n x n grid.")
    
parser.add_argument("--obstacles",
    default=0,
    type=int,
    help="The number of obstacles in the grid.")
    
parser.add_argument("--seed",
    default=int(time.time()),
    type=int,
    help="The random seed")
    
parser.add_argument("--algorithm",
    default=None,
    choices=["all", "bfs", "ucs", "gbfs", "astar", "custom-astar"],
    type=str,
    help="The algorithm to run")
    
parser.add_argument("--submit",
    default=False,
    action="store_true",
    help="Run the experiments required for submission")
    
parser.add_argument("--output-file",
    default=None,
    help="Store the output in the specified file")

parser.add_argument("--env",
    default="cafeWorld",
    type=str,
    help="Choose environment b/w cafeWorld and canWorld")
    
def parse_args():
    """
        Parses the cmd line arguments.
        
        Returns
        ========
            args: Namespace
                The parsed args.
                
        Raises
        =======
            ArgumentError
                If the arguments do not parse correctly.
    """
    args = parser.parse_args()

    dimension_x, dimension_y = args.dimension.split("x")
    dimension_x, dimension_y = int(dimension_x), int(dimension_y)
   
    if dimension_x <= 2 or dimension_y <= 2:
   
        raise Exception("Number of dimensions must be greater than 2")
    
    if args.env not in ["cafeWorld","canWorld"]:
        raise ValueError("Environment can only be cafeWorld or canWorld")

    cell_size = 0.5
    grid_dimensions = [dimension_x/cell_size, dimension_y/cell_size]
    max_obstacles =  grid_dimensions[0]* (grid_dimensions[1] + 1) * 2
    if args.obstacles > max_obstacles:

        raise Exception("Invalid number of obstacles (%u) specified. "
            "Max obstacles possible for a grid of dimension (%u) is [0, %u]." % 
            (args.obstacles, args.dimension, max_obstacles))
   
    return args
