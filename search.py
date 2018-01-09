#!/usr/bin/env python
# -*-coding: utf-8 -*-


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
        The sequence must be composed of legal actions.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of actions that solves tinyMaze.  For any other maze, the
    sequence of actions will be incorrect, so only use this for tinyMaze.
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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from util import Stack
    from game import Directions

    path = []#轨迹队列
    action = []#动作队列
    stack = Stack()#栈
    relation = { }#路径字典
    startState = problem.getStartState()#起点位置
    nowPosition = startState#当前位置

    while True:
        if nowPosition not in path:
            path.append(nowPosition)#当前位置加入路径
        #获取当前位置周围的点
        around = problem.getSuccessors(nowPosition)
        for item in around:
            if(item[0] not in path):
                stack.push(item[0])#可以走的位置入栈
                relation[item[0]] = nowPosition#先后关系加入字典
        nowPosition = stack.pop()#后进先出

        #如果到达目标位置，列出路径的移动方向
        if problem.isGoalState(nowPosition):
            while(nowPosition != startState):
                if nowPosition[0] == relation[nowPosition][0] + 1:
                    action.append(Directions.EAST)#东移
                if nowPosition[1] == relation[nowPosition][1] - 1:
                    action.append(Directions.SOUTH)#南移
                if nowPosition[0] == relation[nowPosition][0] - 1:
                    action.append(Directions.WEST)#西移
                if nowPosition[1] == relation[nowPosition][1] + 1:
                    action.append(Directions.NORTH)#北移
                nowPosition = relation[nowPosition]#往下一个位置移动
            #将动作倒置
            action.reverse()
            #返回动作
            return  action

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    from game import Directions

    path = []#轨迹队列
    action = []#动作队列
    Queue = Queue()#队列
    relation = {}#路径字典
    startState = problem.getStartState()#起点位置
    nowPosition = startState#当前位置

    while True:
        # 将走过的位置加入路径
        if nowPosition not in path:
            path.append(nowPosition)
        # 获取当前位置周围的点
        around = problem.getSuccessors(nowPosition)
        for item in around:
            if (item[0] not in path):
                Queue.push(item[0])#可以走的位置入队列
                relation[item[0]] = nowPosition#先后关系加入字典
        nowPosition = Queue.pop()#先进先出

        # 如果到达目标位置，列出路径的移动方向
        if problem.isGoalState(nowPosition):
            while (nowPosition != startState):
                if nowPosition[0] == relation[nowPosition][0] + 1:
                    action.append(Directions.EAST)#东移
                if nowPosition[1] == relation[nowPosition][1] - 1:
                    action.append(Directions.SOUTH)#南移
                if nowPosition[0] == relation[nowPosition][0] - 1:
                    action.append(Directions.WEST)#西移
                if nowPosition[1] == relation[nowPosition][1] + 1:
                    action.append(Directions.NORTH)#北移
                nowPosition = relation[nowPosition]#往下一个位置移动
            #动作顺序倒置
            action.reverse()
            #返回系列动作
            return action

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    from game import Directions
    import heapq

    path = []#轨迹队列
    action = []#动作队列
    priQueue = PriorityQueue()#优先队列
    relation = {}#路径字典
    startState = problem.getStartState()#起点位置
    nowPosition = startState#当前位置
    priority=0
    while True:
        # 将走过的位置加入路径
        if nowPosition not in path:
            path.append(nowPosition)
        # 获取当前位置周围的点
        around = problem.getSuccessors(nowPosition)
        for item in around:
            if (item[0] not in path):
                priQueue.push(item[0],item[2]+priority)#位置和代价加入优先队列
                relation[item[0]] = nowPosition #先后关系加入字典
        (priority,count,nowPosition) = heapq.heappop((priQueue.heap))#代价最低的位置出队列

        # 如果到达目标位置，列出路径的移动方向
        if problem.isGoalState(nowPosition):
            while (nowPosition != startState):
                if nowPosition[0] == relation[nowPosition][0] + 1:
                    action.append(Directions.EAST)#东移
                if nowPosition[1] == relation[nowPosition][1] - 1:
                    action.append(Directions.SOUTH)#南移
                if nowPosition[0] == relation[nowPosition][0] - 1:
                    action.append(Directions.WEST)#西移
                if nowPosition[1] == relation[nowPosition][1] + 1:
                    action.append(Directions.NORTH)#北移
                nowPosition = relation[nowPosition]

            action.reverse()# 动作倒置
            return action#返回动作
    
def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    from game import Directions
    import heapq

    path = []#轨迹队列
    action = []#动作队列
    relation = {}#路径字典
    priQueue = PriorityQueue()#优先队列

    startState = problem.getStartState()#起点位置
    nowPosition = startState#当前位置

    while True:
        # 将走过的位置加入路径
        if nowPosition not in path:
            path.append(nowPosition)
        # 获取当前位置周围的点
        around = problem.getSuccessors(nowPosition)
        #周围的位置加入优先队列
        for item in around:
            if(item[0] not  in path):
                for i in priQueue.heap:
                    if i[2]==item[0]:#如果相邻点已经在priQueue中
                        if i[1] > item[1]:#检查这条路径是否更优，即是否存在更小的G值
                            #如果G值更小，则把当前节点设为那个位置的父亲节点
                            priQueue.heap.remove(i[0], i[1])#然后重新计算F值和G值
                            priQueue.push(item[0], item[2] + heuristic(item[0], problem))
                            relation[item[0]] = nowPosition  # 先后关系加入字典
                            break
                #G=item[2],H=heuristic(item[0],problem),F=item[2] + heuristic(item[0], problem)
                priQueue.push(item[0], item[2] + heuristic(item[0], problem))
                relation[item[0]] = nowPosition  # 先后关系加入字典
        (priority, nowCost, nowPosition) = heapq.heappop((priQueue.heap))#代价最低的位置出队列

        # 如果到达目标位置，列出路径的移动方向
        if problem.isGoalState(nowPosition):
            while (nowPosition != startState):
                if nowPosition[0] == relation[nowPosition][0] + 1:
                    action.append(Directions.EAST)#东移
                if nowPosition[1] == relation[nowPosition][1] - 1:
                    action.append(Directions.SOUTH)#南移
                if nowPosition[0] == relation[nowPosition][0] - 1:
                    action.append(Directions.WEST)#西移
                if nowPosition[1] == relation[nowPosition][1] + 1:
                    action.append(Directions.NORTH)#北移
                nowPosition = relation[nowPosition]
            #动作顺序倒置
            action.reverse()
            return action#返回动作队列


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
