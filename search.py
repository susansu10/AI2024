"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).

Please only change the parts of the file you are asked to.  Look for the lines
that say

"*** YOUR CODE HERE ***"

Follow the project description for details.

Good luck and happy searching!
"""

import util
from util import manhattanDistance

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    print("Solution:", [s, s, w, s, w, w, s, w])
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    stack = util.Stack()
    parent = {}
    visited = set()
    path = []
    
    stack.push((problem.getStartState(), '', 0))
    visited.add(problem.getStartState())

    while not stack.isEmpty():

        current, action, step = stack.pop()
            
        if problem.isGoalState(current):
            while current in parent:
                current, action = parent[current]
                path.append(action)
            path.reverse()  # Reverse the path to get the correct order from start to goal
            print("Path:", path)
            return path

        for successor, action, step in problem.getSuccessors(current):
            if successor not in visited:
                stack.push((successor, action, step))
                visited.add(successor)
                parent[successor] = (current, action)

    util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    queue = util.Queue()
    visited = set()
    parent = {}
    path = []
    
    queue.push((problem.getStartState(), '', 0))
    visited.add(problem.getStartState())
    
    while not queue.isEmpty():
        current, action, step = queue.pop()
        
        if problem.isGoalState(current):
            while current in parent:
                current, action = parent[current]
                path.append(action)
            path.reverse()
            return path
        
        for successor, action, step in problem.getSuccessors(current):
            if successor not in visited:
                queue.push((successor, action, step))
                visited.add(successor)
                parent[successor] = (current, action)
                
    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    queue = util.PriorityQueue()
    visited = set()
    parent = {}
    path = []
    cost = {}
    
    queue.push((problem.getStartState(), '', 0), 0)
    visited.add(problem.getStartState())
    cost[problem.getStartState()] = 0
    
    while not queue.isEmpty():
        current, action, step = queue.pop()
        
        if problem.isGoalState(current):
            while current in parent:
                current, action = parent[current]
                path.append(action)
            path.reverse()
            return path
        
        for successor, action, step in problem.getSuccessors(current):
            if successor not in visited or cost[current] + step < cost[successor]:
                queue.push((successor, action, step), cost[current] + step)
                visited.add(successor)
                parent[successor] = (current, action)
                cost[successor] = cost[current] + step
    
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    def heuristic1(item):
        state, action, step = item
        return step + heuristic(state, problem)
    
    queue = util.PriorityQueueWithFunction(heuristic1)
    visited = set()
    parent = {}
    path = []
    cost = {}
    
    queue.push((problem.getStartState(), '', 0))
    visited.add(problem.getStartState())
    cost[problem.getStartState()] = 0
    
    while not queue.isEmpty():
        current, action, step = queue.pop()
        
        if problem.isGoalState(current):
            while current in parent:
                current, action = parent[current]
                path.append(action)
            path.reverse()
            return path
        
        for successor, action, step in problem.getSuccessors(current):
            new_cost = cost[current] + heuristic1((successor, action, step))            
            if successor not in visited or new_cost < cost[successor]:

                queue.push((successor, action, step+cost[current]))
                visited.add(successor)
                parent[successor] = (current, action)
                cost[successor] = new_cost
    
    
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
