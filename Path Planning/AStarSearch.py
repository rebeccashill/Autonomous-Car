import numpy as np

start = np.array([0, 0])
goal = np.array([5, 9])
grid = np.array([[0, 0, 0, 0, 0, 0, 0, 1, 0, 0],    #0
                 [0, 1, 1, 0, 0, 0, 0, 1, 0, 0],    #1
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],    #2
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],    #3
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],    #4
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],    #5
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],    #6
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],    #7
                 [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],    #8
                 [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])   #9
         #Columns 0  1  2  3  4  5  6  7  8  9

# Copies of the grid to be used for visualizing results.
path = np.zeros([len(grid), len(grid)], dtype=int)
best_path = np.zeros([len(grid), len(grid)], dtype=int)
h_grid = np.zeros([len(grid), len(grid)], dtype=float)

class AStarSearch:
    def __init__(self, start, goal, grid, path, h_grid):
        self.pos = start
        self.pos_str = str(start)
        self.pos_depth = 0
        self.goal_str = str(goal)
        self.explored = {}
        self.not_explored = {}
        self.not_explored[str(start)] = 0
        self.grid = grid
        self.path = path
        self.h_grid = h_grid
    def get_possible_moves(self):
        potential_moves = self.generate_potential_moves(self.pos)
        for move in potential_moves:
            # Check if potential move is valid.
            if not self.valid_move(move):
                continue
            # Check if move has already been explored.
            if (str(move) not in self.explored) and (
                str(move) not in self.not_explored):
                self.not_explored[str(move)] = self.pos_depth + 1 + self.heuristic(move)
                #Visualize the Heuristic Grid
                self.h_grid[move[0], move[1]] = self.pos_depth + 1 + self.heuristic(move)
        # Since all next possible moves have been determined,
        # consider current location explored.
        self.explored[self.pos_str] = 0
        return True

    def explore_next_move(self):
        #Determine next move to explore
        sorted_not_explored = sorted(
            self.not_explored,
            key=self.not_explored.get,
            reverse=False)

        #Determine the pos and depth of the next move.
        self.pos_str = sorted_not_explored[0]
        self.pos = self.string_to_array(self.pos_str)
        self.pos_depth = self.not_explored.pop(self.pos_str) - self.heuristic(self.pos)

        #Write depth of next move onto path.
        self.path[self.pos[0], self.pos[1]] = round(self.pos_depth,1)

        return True

    def heuristic(self, move):
        distance = move - goal
        distance_squared = distance * distance
        return round(np.sqrt(sum(distance_squared)), 1)

    def generate_potential_moves(self,pos):
        u = np.array([-1, 0])
        d = np.array([-1, 0])
        l = np.array([0, -1])
        r = np.array([0, 1])

        potential_moves = [pos + u, pos + d, pos + l, pos + r]

        return potential_moves

    def valid_move(self, move):
        #Check if out of boundary.
        if (move[0] < 0) or (move[0] > 9):
            return False
        if (move[1] < 0) or (move[1] > 9):
            return False
        #Check if wall or obstacle exists.
        if self.grid[move[0], move[1]] == 1:
            return False
        return True

    def string_to_array(self, string):
        array = [int(string[1]), int(string[3])]
        return np.array(array)

astar = AStarSearch(start, goal, grid, path, h_grid)

while True:
    # Determine next possible moves
    astar.get_possible_moves()
    if astar.goal_found():
        break
    astar.explore_next_move()

print('')
print('Heuristic Grid')
print('--------------')
print(h_grid)
print('')
print('')
print('Explored Path')