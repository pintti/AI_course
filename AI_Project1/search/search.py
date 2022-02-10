# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from errno import ESTALE
from queue import PriorityQueue
from re import search
from tokenize import ContStr
import util

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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
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
    pastNodes = util.Stack()
    routes = {}
    currentState = problem.getStartState()
    futureStack = util.Stack()
    while not problem.isGoalState(currentState):
        pastNodes.push(currentState)
        for node in problem.getSuccessors(currentState):
            point, direction, cost = node
            if point not in pastNodes.list:
                futureStack.push(point)
                routes[str(point)] = [currentState, direction]
        currentState = futureStack.pop()
    return getActionList(problem, currentState, routes)


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    pastNodes = util.Stack()
    futureNodes = util.Queue()
    futureNodes.push((problem.getStartState(), "", 0))
    routes = {str(problem.getStartState()): [None, []]}
    while True:
        if not futureNodes: return False
        node = futureNodes.pop()
        point, direction, cost = node
        if problem.isGoalState(point):
            return routes[str(point)][1]
        if point not in pastNodes.list:
            pastNodes.push(node[0])
            for childNode in problem.getSuccessors(point):
                childPoint, childDir, childCost = childNode
                if childPoint not in pastNodes.list:
                    if str(childPoint) not in routes.keys():
                        routes[str(childPoint)] = [point, routes[str(point)][1][:]]
                        if childDir:
                            routes[str(childPoint)][1].append(childDir)
                        futureNodes.push(childNode)

        

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    pastNodes = util.Stack()
    routes = {}
    currentState = problem.getStartState()
    futureStack = util.PriorityQueue()
    currentCost = 0
    while not problem.isGoalState(currentState):
        pastNodes.push(currentState)
        for node in problem.getSuccessors(currentState):
            point, direction, cost = node
            if point not in pastNodes.list and point not in futureStack.heap:
                futureStack.update(point, currentCost + cost)
                if point not in routes.keys():
                    routes[str(point)] = [currentState, direction, cost]
                else:
                    if routes[str(point)][2] > currentCost + cost:
                        routes[str(point)] = [currentState, direction, cost]
        currentState = futureStack.pop()
        currentCost = problem.getCostOfActions(getActionListCost(problem, currentState, routes))
    return getActionListCost(problem, currentState, routes)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    pastNodes = util.Stack()
    routes = {}
    currentState = problem.getStartState()
    futureStack = util.PriorityQueue()
    currentCost = 0
    while not problem.isGoalState(currentState):
        pastNodes.push(currentState)
        for node in problem.getSuccessors(currentState):
            point, direction, cost = node
            cost += heuristic(point, problem) + currentCost
            if point not in pastNodes.list and point not in futureStack.heap:
                futureStack.update(point, cost)
                if point not in routes.keys():
                    routes[str(point)] = [currentState, direction, cost]
                elif routes[str(point)][2] > cost:
                    routes[str(point)] = [currentState, direction, cost]
        currentState = futureStack.pop()
        currentCost = problem.getCostOfActions(getActionListCost(problem, currentState, routes))
    return getActionListCost(problem, currentState, routes)

def getActionList(problem, currentState, routes):
    actions = []
    while currentState != problem.getStartState():
        currentState, action = routes[str(currentState)]
        actions.append(action)
    actions.reverse()
    return actions

def getActionListCost(problem, currentState, routes):
    actions = []
    while currentState != problem.getStartState():
        currentState, action, _ = routes[str(currentState)]
        actions.append(action)
    actions.reverse()
    return actions

def getPastList(start, state, routes):
    actions = []
    actions.append(routes[str(state)][1])
    while state != start:
        state, action = routes[str(state)]
        if action:
            actions.append(action)
    actions.reverse()
    return actions

        
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
