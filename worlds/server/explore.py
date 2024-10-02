import numpy as np
import sys
from flask import Flask, request, jsonify

app = Flask(__name__)


class CalculateTrajectories:
    def __init__(self, r, c, MST):
        self.MAX_NODES = 4 * r * c
        self.PathSequence = []
        self.rows = r
        self.cols = c
        self.MSTvector = MST
        self.MSTedges = len(self.MSTvector)
        self.allEdges = set()
        self.nodes = {}
        for node in range(self.MAX_NODES):
            self.nodes[node] = None

    def initializeGraph(self, A, connect4):
        for i in range(2 * self.rows):
            for j in range(2 * self.cols):
                if A[i, j]:
                    if i > 0 and A[i - 1][j]:
                        self.AddToAllEdges(
                            i * 2 * self.cols + j, (i - 1) * 2 * self.cols + j, 1
                        )
                    if i < 2 * self.rows - 1 and A[i + 1][j]:
                        self.AddToAllEdges(
                            i * 2 * self.cols + j, (i + 1) * 2 * self.cols + j, 1
                        )
                    if j > 0 and A[i][j - 1]:
                        self.AddToAllEdges(
                            i * 2 * self.cols + j, i * 2 * self.cols + j - 1, 1
                        )
                    if j < 2 * self.cols - 1 and A[i][j + 1]:
                        self.AddToAllEdges(
                            i * 2 * self.cols + j, i * 2 * self.cols + j + 1, 1
                        )

                    if not connect4:
                        if i > 0 and j > 0 and A[i - 1][j - 1]:
                            self.AddToAllEdges(
                                i * 2 * self.cols + j,
                                (i - 1) * 2 * self.cols + j - 1,
                                1,
                            )
                        if (
                            i < 2 * self.rows - 1
                            and j < 2 * self.cols - 1
                            and A[i + 1][j + 1]
                        ):
                            self.AddToAllEdges(
                                i * 2 * self.cols + j,
                                (i + 1) * 2 * self.cols + j + 1,
                                1,
                            )
                        if i > 2 * self.rows - 1 and j > 0 and A[i + 1][j - 1]:
                            self.AddToAllEdges(
                                i * 2 * self.cols + j,
                                (i + 1) * 2 * self.cols + j - 1,
                                1,
                            )
                        if i > 0 and j < 2 * self.cols - 1 and A[i - 1][j + 1]:
                            self.AddToAllEdges(
                                i * 2 * self.cols + j,
                                (i - 1) * 2 * self.cols + j + 1,
                                1,
                            )

    def AddToAllEdges(self, _from: int, to: int, cost):
        self.allEdges.add(Edge(_from, to, cost))

        if (self.nodes[_from]) is None:
            self.nodes[_from] = set()

        self.nodes[_from].add(to)

        if (self.nodes[to]) is None:
            self.nodes[to] = set()

        self.nodes[to].add(_from)

    def RemoveTheAppropriateEdges(self):
        for i in range(self.MSTedges):
            e = self.MSTvector[i]
            maxN = max(e.src, e.dst)
            minN = min(e.src, e.dst)

            if np.absolute(e.src - e.dst) == 1:
                alpha = (4 * minN + 3) - 2 * (maxN % self.cols)
                eToRemove = Edge(alpha, alpha + 2 * self.cols, 1)
                eToRemoveMirr = Edge(alpha + 2 * self.cols, alpha, 1)
                eToRemove2 = Edge(alpha + 1, alpha + 1 + 2 * self.cols, 1)
                eToRemove2Mirr = Edge(alpha + 1 + 2 * self.cols, alpha + 1, 1)

            else:
                alpha = (4 * minN + 2 * self.cols) - 2 * (maxN % self.cols)
                eToRemove = Edge(alpha, alpha + 1, 1)
                eToRemoveMirr = Edge(alpha + 1, alpha, 1)
                eToRemove2 = Edge(alpha + 2 * self.cols, alpha + 1 + 2 * self.cols, 1)
                eToRemove2Mirr = Edge(
                    alpha + 1 + 2 * self.cols, alpha + 2 * self.cols, 1
                )

            if eToRemove in self.allEdges:
                self.SafeRemoveEdge(eToRemove)

            if eToRemoveMirr in self.allEdges:
                self.SafeRemoveEdge(eToRemoveMirr)

            if eToRemove2 in self.allEdges:
                self.SafeRemoveEdge(eToRemove2)

            if eToRemove2Mirr in self.allEdges:
                self.SafeRemoveEdge(eToRemove2Mirr)

    def SafeRemoveEdge(self, curEdge):
        try:
            self.allEdges.remove(curEdge)
            # successful removal from priority queue: allEdges
            if curEdge.dst in self.nodes[curEdge.src]:
                self.nodes[curEdge.src].remove(curEdge.dst)
            if curEdge.src in self.nodes[curEdge.dst]:
                self.nodes[curEdge.dst].remove(curEdge.src)

        except KeyError:
            # This is a serious problem
            print("TreeSet should have contained this element!!")
            sys.exit(1)

    def CalculatePathsSequence(self, StartingNode):
        currentNode = StartingNode
        RemovedNodes = set()
        movement = []
        PathSequence = []

        movement.append(2 * self.cols)
        movement.append(-1)
        movement.append(-2 * self.cols)
        movement.append(1)

        found = False
        prevNode = 0
        for idx in range(4):
            if (currentNode + movement[idx]) in list(self.nodes[currentNode]):
                prevNode = currentNode + movement[idx]
                found = True
                break

        if not found:
            return

        while True:
            if currentNode != StartingNode:
                RemovedNodes.add(currentNode)

            offset = movement.index(prevNode - currentNode)

            prevNode = currentNode

            found = False
            for idx in range(4):
                if (
                    prevNode + movement[(idx + offset) % 4] in self.nodes[prevNode]
                ) and not (prevNode + movement[(idx + offset) % 4] in RemovedNodes):
                    currentNode = prevNode + movement[(idx + offset) % 4]
                    found = True
                    break

            if not found:
                return

            if prevNode in self.nodes[currentNode]:
                self.nodes[currentNode].remove(prevNode)

            if currentNode in self.nodes[prevNode]:
                self.nodes[prevNode].remove(currentNode)

            i = int(currentNode / (2 * self.cols))
            j = currentNode % (2 * self.cols)
            previ = int(prevNode / (2 * self.cols))
            prevj = prevNode % (2 * self.cols)
            self.PathSequence.append((previ, prevj, i, j))


import numpy as np
import sys
import cv2

# from Visualization import darp_area_visualization
import time
import random
import os
from numba import njit

np.set_printoptions(threshold=sys.maxsize)

random.seed(1)
os.environ["PYTHONHASHSEED"] = str(1)
np.random.seed(1)


@njit(fastmath=True)
def assign(droneNo, rows, cols, GridEnv, MetricMatrix, A):
    ArrayOfElements = np.zeros(droneNo)
    for i in range(rows):
        for j in range(cols):
            if GridEnv[i, j] == -1:
                minV = MetricMatrix[0, i, j]
                indMin = 0
                for r in range(droneNo):
                    if MetricMatrix[r, i, j] < minV:
                        minV = MetricMatrix[r, i, j]
                        indMin = r

                A[i, j] = indMin
                ArrayOfElements[indMin] += 1

            elif GridEnv[i, j] == -2:
                A[i, j] = droneNo
    return A, ArrayOfElements


@njit(fastmath=True)
def inverse_binary_map_as_uint8(BinaryMap):
    # cv2.distanceTransform needs input of dtype unit8 (8bit)
    return np.logical_not(BinaryMap).astype(np.uint8)


@njit(fastmath=True)
def euclidian_distance_points2d(array1: np.array, array2: np.array) -> np.float_:
    # this runs much faster than the (numba) np.linalg.norm and is totally enough for our purpose
    return (((array1[0] - array2[0]) ** 2) + ((array1[1] - array2[1]) ** 2)) ** 0.5


@njit(fastmath=True)
def constructBinaryImages(labels_im, robo_start_point, rows, cols):
    BinaryRobot = np.copy(labels_im)
    BinaryNonRobot = np.copy(labels_im)
    for i in range(rows):
        for j in range(cols):
            if labels_im[i, j] == labels_im[robo_start_point]:
                BinaryRobot[i, j] = 1
                BinaryNonRobot[i, j] = 0
            elif labels_im[i, j] != 0:
                BinaryRobot[i, j] = 0
                BinaryNonRobot[i, j] = 1

    return BinaryRobot, BinaryNonRobot


@njit(fastmath=True)
def CalcConnectedMultiplier(rows, cols, dist1, dist2, CCvariation):
    returnM = np.zeros((rows, cols))
    MaxV = 0
    MinV = 2**30

    for i in range(rows):
        for j in range(cols):
            returnM[i, j] = dist1[i, j] - dist2[i, j]
            if MaxV < returnM[i, j]:
                MaxV = returnM[i, j]
            if MinV > returnM[i, j]:
                MinV = returnM[i, j]

    for i in range(rows):
        for j in range(cols):
            returnM[i, j] = (returnM[i, j] - MinV) * (
                (2 * CCvariation) / (MaxV - MinV)
            ) + (1 - CCvariation)

    return returnM


class DARP:
    def __init__(
        self,
        nx,
        ny,
        notEqualPortions,
        given_initial_positions,
        given_portions,
        obstacles_positions,
        visualization,
        MaxIter=80000,
        CCvariation=0.01,
        randomLevel=0.0001,
        dcells=2,
        importance=False,
    ):
        self.rows = nx
        self.cols = ny
        (
            self.initial_positions,
            self.obstacles_positions,
            self.portions,
        ) = self.sanity_check(
            given_initial_positions,
            given_portions,
            obstacles_positions,
            notEqualPortions,
        )

        self.visualization = visualization
        self.MaxIter = MaxIter
        self.CCvariation = CCvariation
        self.randomLevel = randomLevel
        self.dcells = dcells
        self.importance = importance
        self.notEqualPortions = notEqualPortions

        print("\nInitial Conditions Defined:")
        print("Grid Dimensions:", nx, ny)
        print("Number of Robots:", len(self.initial_positions))
        print("Initial Robots' positions", self.initial_positions)
        print("Portions for each Robot:", self.portions, "\n")

        self.droneNo = len(self.initial_positions)
        self.A = np.zeros((self.rows, self.cols))
        self.GridEnv = self.defineGridEnv()

        self.connectivity = np.zeros(
            (self.droneNo, self.rows, self.cols), dtype=np.uint8
        )
        self.BinaryRobotRegions = np.zeros(
            (self.droneNo, self.rows, self.cols), dtype=bool
        )

        (
            self.MetricMatrix,
            self.termThr,
            self.Notiles,
            self.DesireableAssign,
            self.TilesImportance,
            self.MinimumImportance,
            self.MaximumImportance,
        ) = self.construct_Assignment_Matrix()
        self.ArrayOfElements = np.zeros(self.droneNo)
        self.color = []

        for r in range(self.droneNo):
            np.random.seed(r)
            self.color.append(list(np.random.choice(range(256), size=3)))

        np.random.seed(1)
        if self.visualization:
            self.assignment_matrix_visualization = darp_area_visualization(
                self.A, self.droneNo, self.color, self.initial_positions
            )

    def sanity_check(
        self, given_initial_positions, given_portions, obs_pos, notEqualPortions
    ):
        initial_positions = []
        for position in given_initial_positions:
            if position < 0 or position >= self.rows * self.cols:
                print("Initial positions should be inside the Grid.")
                sys.exit(1)
            initial_positions.append((position // self.cols, position % self.cols))

        obstacles_positions = []
        for obstacle in obs_pos:
            if obstacle < 0 or obstacle >= self.rows * self.cols:
                print("Obstacles should be inside the Grid.")
                sys.exit(2)
            obstacles_positions.append((obstacle // self.cols, obstacle % self.cols))

        portions = []
        if notEqualPortions:
            portions = given_portions
        else:
            for drone in range(len(initial_positions)):
                portions.append(1 / len(initial_positions))

        if len(initial_positions) != len(portions):
            print("Portions should be defined for each drone")
            sys.exit(3)

        s = sum(portions)
        if abs(s - 1) >= 0.0001:
            print("Sum of portions should be equal to 1.")
            sys.exit(4)

        for position in initial_positions:
            for obstacle in obstacles_positions:
                if position[0] == obstacle[0] and position[1] == obstacle[1]:
                    print("Initial positions should not be on obstacles")
                    sys.exit(5)

        return initial_positions, obstacles_positions, portions

    def defineGridEnv(self):
        GridEnv = np.full(
            shape=(self.rows, self.cols), fill_value=-1
        )  # create non obstacle map with value -1

        # obstacle tiles value is -2
        for idx, obstacle_pos in enumerate(self.obstacles_positions):
            GridEnv[obstacle_pos[0], obstacle_pos[1]] = -2

        connectivity = np.zeros((self.rows, self.cols))

        mask = np.where(GridEnv == -1)
        connectivity[mask[0], mask[1]] = 255
        image = np.uint8(connectivity)
        num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)

        if num_labels > 2:
            print(
                "The environment grid MUST not have unreachable and/or closed shape regions"
            )
            sys.exit(6)

        # initial robot tiles will have their array.index as value
        for idx, robot in enumerate(self.initial_positions):
            GridEnv[robot] = idx
            self.A[robot] = idx

        return GridEnv

    def divideRegions(self):
        success = False
        cancelled = False
        criterionMatrix = np.zeros((self.rows, self.cols))
        iteration = 0

        while self.termThr <= self.dcells and not success and not cancelled:
            downThres = (self.Notiles - self.termThr * (self.droneNo - 1)) / (
                self.Notiles * self.droneNo
            )
            upperThres = (self.Notiles + self.termThr) / (self.Notiles * self.droneNo)

            success = True

            # Main optimization loop

            iteration = 0

            while iteration <= self.MaxIter and not cancelled:
                self.A, self.ArrayOfElements = assign(
                    self.droneNo,
                    self.rows,
                    self.cols,
                    self.GridEnv,
                    self.MetricMatrix,
                    self.A,
                )
                ConnectedMultiplierList = np.ones((self.droneNo, self.rows, self.cols))
                ConnectedRobotRegions = np.zeros(self.droneNo)
                plainErrors = np.zeros((self.droneNo))
                divFairError = np.zeros((self.droneNo))

                self.update_connectivity()
                for r in range(self.droneNo):
                    ConnectedMultiplier = np.ones((self.rows, self.cols))
                    ConnectedRobotRegions[r] = True
                    num_labels, labels_im = cv2.connectedComponents(
                        self.connectivity[r, :, :], connectivity=4
                    )
                    if num_labels > 2:
                        ConnectedRobotRegions[r] = False
                        BinaryRobot, BinaryNonRobot = constructBinaryImages(
                            labels_im, self.initial_positions[r], self.rows, self.cols
                        )
                        ConnectedMultiplier = CalcConnectedMultiplier(
                            self.rows,
                            self.cols,
                            self.NormalizedEuclideanDistanceBinary(True, BinaryRobot),
                            self.NormalizedEuclideanDistanceBinary(
                                False, BinaryNonRobot
                            ),
                            self.CCvariation,
                        )
                    ConnectedMultiplierList[r, :, :] = ConnectedMultiplier
                    plainErrors[r] = self.ArrayOfElements[r] / (
                        self.DesireableAssign[r] * self.droneNo
                    )
                    if plainErrors[r] < downThres:
                        divFairError[r] = downThres - plainErrors[r]
                    elif plainErrors[r] > upperThres:
                        divFairError[r] = upperThres - plainErrors[r]

                if self.IsThisAGoalState(self.termThr, ConnectedRobotRegions):
                    break

                TotalNegPerc = 0
                totalNegPlainErrors = 0
                correctionMult = np.zeros(self.droneNo)

                for r in range(self.droneNo):
                    if divFairError[r] < 0:
                        TotalNegPerc += np.absolute(divFairError[r])
                        totalNegPlainErrors += plainErrors[r]

                    correctionMult[r] = 1

                for r in range(self.droneNo):
                    if totalNegPlainErrors != 0:
                        if divFairError[r] < 0:
                            correctionMult[r] = 1 + (
                                plainErrors[r] / totalNegPlainErrors
                            ) * (TotalNegPerc / 2)
                        else:
                            correctionMult[r] = 1 - (
                                plainErrors[r] / totalNegPlainErrors
                            ) * (TotalNegPerc / 2)

                        criterionMatrix = self.calculateCriterionMatrix(
                            self.TilesImportance[r],
                            self.MinimumImportance[r],
                            self.MaximumImportance[r],
                            correctionMult[r],
                            divFairError[r] < 0,
                        )

                    self.MetricMatrix[r] = self.FinalUpdateOnMetricMatrix(
                        criterionMatrix,
                        self.generateRandomMatrix(),
                        self.MetricMatrix[r],
                        ConnectedMultiplierList[r, :, :],
                    )

                iteration += 1
                if self.visualization:
                    self.assignment_matrix_visualization.placeCells(
                        self.A, iteration_number=iteration
                    )
                    time.sleep(0.2)

            if iteration >= self.MaxIter:
                self.MaxIter = self.MaxIter / 2
                success = False
                self.termThr += 1

        self.getBinaryRobotRegions()
        return success, iteration

    def getBinaryRobotRegions(self):
        ind = np.where(self.A < self.droneNo)
        temp = (self.A[ind].astype(int),) + ind
        self.BinaryRobotRegions[temp] = True

    def generateRandomMatrix(self):
        RandomMatrix = np.zeros((self.rows, self.cols))
        RandomMatrix = 2 * self.randomLevel * np.random.uniform(
            0, 1, size=RandomMatrix.shape
        ) + (1 - self.randomLevel)
        return RandomMatrix

    def FinalUpdateOnMetricMatrix(self, CM, RM, currentOne, CC):
        MMnew = np.zeros((self.rows, self.cols))
        MMnew = currentOne * CM * RM * CC

        return MMnew

    def IsThisAGoalState(self, thresh, connectedRobotRegions):
        for r in range(self.droneNo):
            if (
                np.absolute(self.DesireableAssign[r] - self.ArrayOfElements[r]) > thresh
                or not connectedRobotRegions[r]
            ):
                return False
        return True

    def update_connectivity(self):
        self.connectivity = np.zeros(
            (self.droneNo, self.rows, self.cols), dtype=np.uint8
        )
        for i in range(self.droneNo):
            mask = np.where(self.A == i)
            self.connectivity[i, mask[0], mask[1]] = 255

    # Construct Assignment Matrix
    def construct_Assignment_Matrix(self):
        Notiles = self.rows * self.cols
        fair_division = 1 / self.droneNo
        effectiveSize = Notiles - self.droneNo - len(self.obstacles_positions)
        termThr = 0

        if effectiveSize % self.droneNo != 0:
            termThr = 1

        DesireableAssign = np.zeros(self.droneNo)
        MaximunDist = np.zeros(self.droneNo)
        MaximumImportance = np.zeros(self.droneNo)
        MinimumImportance = np.zeros(self.droneNo)

        for i in range(self.droneNo):
            DesireableAssign[i] = effectiveSize * self.portions[i]
            MinimumImportance[i] = sys.float_info.max
            if DesireableAssign[i] != int(DesireableAssign[i]) and termThr != 1:
                termThr = 1

        AllDistances = np.zeros((self.droneNo, self.rows, self.cols))
        TilesImportance = np.zeros((self.droneNo, self.rows, self.cols))

        for x in range(self.rows):
            for y in range(self.cols):
                tempSum = 0
                for r in range(self.droneNo):
                    AllDistances[r, x, y] = euclidian_distance_points2d(
                        np.array(self.initial_positions[r]), np.array((x, y))
                    )  # E!
                    if AllDistances[r, x, y] > MaximunDist[r]:
                        MaximunDist[r] = AllDistances[r, x, y]
                    tempSum += AllDistances[r, x, y]

                for r in range(self.droneNo):
                    if tempSum - AllDistances[r, x, y] != 0:
                        TilesImportance[r, x, y] = 1 / (tempSum - AllDistances[r, x, y])
                    else:
                        TilesImportance[r, x, y] = 1
                    # Todo FixMe!
                    if TilesImportance[r, x, y] > MaximumImportance[r]:
                        MaximumImportance[r] = TilesImportance[r, x, y]

                    if TilesImportance[r, x, y] < MinimumImportance[r]:
                        MinimumImportance[r] = TilesImportance[r, x, y]

        return (
            AllDistances,
            termThr,
            Notiles,
            DesireableAssign,
            TilesImportance,
            MinimumImportance,
            MaximumImportance,
        )

    def calculateCriterionMatrix(
        self,
        TilesImportance,
        MinimumImportance,
        MaximumImportance,
        correctionMult,
        smallerthan_zero,
    ):
        returnCrit = np.zeros((self.rows, self.cols))
        if self.importance:
            if smallerthan_zero:
                returnCrit = (TilesImportance - MinimumImportance) * (
                    (correctionMult - 1) / (MaximumImportance - MinimumImportance)
                ) + 1
            else:
                returnCrit = (TilesImportance - MinimumImportance) * (
                    (1 - correctionMult) / (MaximumImportance - MinimumImportance)
                ) + correctionMult
        else:
            returnCrit[:, :] = correctionMult

        return returnCrit

    def NormalizedEuclideanDistanceBinary(self, RobotR, BinaryMap):
        distRobot = cv2.distanceTransform(
            inverse_binary_map_as_uint8(BinaryMap),
            distanceType=2,
            maskSize=0,
            dstType=5,
        )
        MaxV = np.max(distRobot)
        MinV = np.min(distRobot)

        # Normalization
        if RobotR:
            distRobot = (distRobot - MinV) * (1 / (MaxV - MinV)) + 1
        else:
            distRobot = (distRobot - MinV) * (1 / (MaxV - MinV))

        return distRobot


class Edge(object):
    def __init__(self, _from, to, weight):
        self.src = _from
        self.dst = to
        self.weight = weight

    def __eq__(self, other):
        return (
            self.src == other.src
            and self.dst == other.dst
            and self.weight == other.weight
        )

    def __hash__(self):
        return hash((self.src, self.dst, self.weight))


class Graph:
    def __init__(self, nodes, arg_edgelist):
        self.nodes = nodes
        self.num_nodes = len(self.nodes)
        self.edgelist = arg_edgelist
        self.parent = []
        self.rank = []
        # mst stores edges of the minimum spanning tree
        self.mst = []

    def FindParent(self, node):
        # With path-compression.

        if node != self.parent[node]:
            self.parent[node] = self.FindParent(self.parent[node])
        return self.parent[node]

    def KruskalMST(self):
        # Sort objects of an Edge class based on attribute (weight)
        self.edgelist.sort(key=lambda Edge: Edge.weight)

        self.parent = [None] * self.num_nodes
        self.rank = [None] * self.num_nodes

        for n in self.nodes:
            self.parent[n] = n  # Every node is the parent of itself at the beginning
            self.rank[n] = 0  # Rank of every node is 0 at the beginning

        for edge in self.edgelist:
            root1 = self.FindParent(edge.src)
            root2 = self.FindParent(edge.dst)

            # Parents of the source and destination nodes are not in the same subset
            # Add the edge to the spanning tree
            if root1 != root2:
                self.mst.append(edge)
                if self.rank[root1] < self.rank[root2]:
                    self.parent[root1] = root2
                    self.rank[root2] += 1
                else:
                    self.parent[root2] = root1
                    self.rank[root1] += 1

        cost = 0
        for edge in self.mst:
            cost += edge.weight


# from Edges import Edge, Graph
import sys


class Kruskal(object):
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.allEdges = []
        self.MAX_NODES = self.rows * self.cols
        self.nodes = {}
        for node in range(self.MAX_NODES):
            self.nodes[node] = None
        self.mst = []

    def initializeGraph(self, A, connect4, mode):
        cost1 = 1
        cost2 = 1

        for i in range(self.rows):
            for j in range(self.cols):
                if A[i][j]:
                    if mode == 0:
                        cost2 = self.rows - i
                    elif mode == 1:
                        cost2 = i + 1
                    elif mode == 2:
                        cost1 = self.cols - j
                    elif mode == 3:
                        cost1 = j + 1

                    if i > 0 and A[i - 1][j]:
                        self.AddToAllEdges(
                            i * self.cols + j, (i - 1) * self.cols + j, cost1
                        )
                    if i < self.rows - 1 and A[i + 1][j]:
                        self.AddToAllEdges(
                            i * self.cols + j, (i + 1) * self.cols + j, cost1
                        )
                    if j > 0 and A[i][j - 1]:
                        self.AddToAllEdges(
                            i * self.cols + j, i * self.cols + j - 1, cost2
                        )
                    if j < self.cols - 1 and A[i][j + 1]:
                        self.AddToAllEdges(
                            i * self.cols + j, i * self.cols + j + 1, cost2
                        )

                    if not connect4:
                        if i > 0 and j > 0 and A[i - 1][j - 1]:
                            AddToAllEdges(
                                i * self.cols + j, (i - 1) * self.cols + j - 1, 1
                            )
                        if i < rows - 1 and j < self.cols - 1 and A[i + 1][j + 1]:
                            AddToAllEdges(
                                i * self.cols + j, (i + 1) * self.cols + j + 1, 1
                            )
                        if i > rows - 1 and j > 0 and A[i + 1][j - 1]:
                            AddToAllEdges(
                                i * self.cols + j, (i + 1) * self.cols + j - 1, 1
                            )
                        if i > 0 and j < self.cols - 1 and A[i - 1][j + 1]:
                            AddToAllEdges(
                                i * self.cols + j, (i - 1) * self.cols + j + 1, 1
                            )

    def AddToAllEdges(self, _from: int, to: int, cost):
        self.allEdges.insert(0, Edge(_from, to, cost))

        if (self.nodes[_from]) is None:
            self.nodes[_from] = set()
            self.nodes[_from].add(_from)

        if (self.nodes[to]) is None:
            self.nodes[to] = set()
            self.nodes[to].add(to)

        return

    def performKruskal(self):
        g1 = Graph(self.nodes, self.allEdges)
        g1.KruskalMST()
        self.mst = g1.mst


import numpy as np


class turns:
    def __init__(self, paths):
        """
        paths: List of lists of moves per drone
        """
        self.paths = paths
        self.turns = []

    def __str__(self):
        return (
            "\n"
            f"Turns: {self.turns}\n"
            f"Average: {self.avg:.3f}\n"
            f"Standard Deviation: {self.std:.3f}\n"
        )

    def count_turns(self):
        for path in self.paths:
            num_turns = -1
            last_move = ""
            for move in path:
                if move[0] == move[2]:
                    current_move = "horizontal"
                elif move[1] == move[3]:
                    current_move = "vertical"

                if last_move != current_move:
                    num_turns += 1

                last_move = current_move
            self.turns.append(num_turns)

    def find_avg_and_std(self):
        self.avg = np.average(self.turns)
        self.std = np.std(self.turns)


import pickle

# from darp import DARP
import numpy as np

# from kruskal import Kruskal
# from CalculateTrajectories import CalculateTrajectories
# from Visualization import visualize_paths
import sys
import argparse

from PIL import Image
import time


def get_area_map(path, area=0, obs=-1):
    """
    Creates an array from a given png-image(path).
    :param path: path to the png-image
    :param area: non-obstacles tiles value; standard is 0
    :param obs: obstacle tiles value; standard is -1
    :return: an array of area(0) and obstacle(-1) tiles
    """
    le_map = np.array(Image.open(path))
    ma = np.array(le_map).mean(axis=2) != 0
    le_map = np.int8(np.zeros(ma.shape))
    le_map[ma] = area
    le_map[~ma] = obs
    return le_map


def get_area_indices(area, value, inv=False, obstacle=-1):
    """
    Returns area tiles indices that have value
    If inv(erted), returns indices that don't have value
    :param area: array with value and obstacle tiles
    :param value: searched tiles with value
    :param inv: if True: search will be inverted and index of non-value tiles will get returned
    :param obstacle: defines obstacle tiles
    :return:
    """
    try:
        value = int(value)
        if inv:
            return np.concatenate([np.where((area != value))]).T
        return np.concatenate([np.where((area == value))]).T
    except:
        mask = area == value[0]
        if inv:
            mask = area != value[0]
        for v in value[1:]:
            if inv:
                mask &= area != v
            else:
                mask |= area == v
        mask &= area != obstacle
        return np.concatenate([np.where(mask)]).T


class MultiRobotPathPlanner(DARP):
    def __init__(
        self,
        nx,
        ny,
        notEqualPortions,
        initial_positions,
        portions,
        obs_pos,
        visualization,
        MaxIter=80000,
        CCvariation=0.01,
        randomLevel=0.0001,
        dcells=2,
        importance=False,
    ):
        start_time = time.time()
        # Initialize DARP
        self.darp_instance = DARP(
            nx,
            ny,
            notEqualPortions,
            initial_positions,
            portions,
            obs_pos,
            visualization,
            MaxIter=MaxIter,
            CCvariation=CCvariation,
            randomLevel=randomLevel,
            dcells=dcells,
            importance=importance,
        )

        # Divide areas based on robots initial positions
        self.DARP_success, self.iterations = self.darp_instance.divideRegions()

        # Check if solution was found
        if not self.DARP_success:
            print("DARP did not manage to find a solution for the given configuration!")
        else:
            # Iterate for 4 different ways to join edges in MST
            self.mode_to_drone_turns = []
            AllRealPaths_dict = {}
            subCellsAssignment_dict = {}
            for mode in range(4):
                MSTs = self.calculateMSTs(
                    self.darp_instance.BinaryRobotRegions,
                    self.darp_instance.droneNo,
                    self.darp_instance.rows,
                    self.darp_instance.cols,
                    mode,
                )
                AllRealPaths = []
                for r in range(self.darp_instance.droneNo):
                    ct = CalculateTrajectories(
                        self.darp_instance.rows, self.darp_instance.cols, MSTs[r]
                    )
                    ct.initializeGraph(
                        self.CalcRealBinaryReg(
                            self.darp_instance.BinaryRobotRegions[r],
                            self.darp_instance.rows,
                            self.darp_instance.cols,
                        ),
                        True,
                    )
                    ct.RemoveTheAppropriateEdges()
                    ct.CalculatePathsSequence(
                        4
                        * self.darp_instance.initial_positions[r][0]
                        * self.darp_instance.cols
                        + 2 * self.darp_instance.initial_positions[r][1]
                    )
                    AllRealPaths.append(ct.PathSequence)

                self.TypesOfLines = np.zeros(
                    (self.darp_instance.rows * 2, self.darp_instance.cols * 2, 2)
                )
                for r in range(self.darp_instance.droneNo):
                    flag = False
                    for connection in AllRealPaths[r]:
                        if flag:
                            if self.TypesOfLines[connection[0]][connection[1]][0] == 0:
                                indxadd1 = 0
                            else:
                                indxadd1 = 1

                            if (
                                self.TypesOfLines[connection[2]][connection[3]][0] == 0
                                and flag
                            ):
                                indxadd2 = 0
                            else:
                                indxadd2 = 1
                        else:
                            if not (
                                self.TypesOfLines[connection[0]][connection[1]][0] == 0
                            ):
                                indxadd1 = 0
                            else:
                                indxadd1 = 1
                            if not (
                                self.TypesOfLines[connection[2]][connection[3]][0] == 0
                                and flag
                            ):
                                indxadd2 = 0
                            else:
                                indxadd2 = 1

                        flag = True
                        if connection[0] == connection[2]:
                            if connection[1] > connection[3]:
                                self.TypesOfLines[connection[0]][connection[1]][
                                    indxadd1
                                ] = 2
                                self.TypesOfLines[connection[2]][connection[3]][
                                    indxadd2
                                ] = 3
                            else:
                                self.TypesOfLines[connection[0]][connection[1]][
                                    indxadd1
                                ] = 3
                                self.TypesOfLines[connection[2]][connection[3]][
                                    indxadd2
                                ] = 2

                        else:
                            if connection[0] > connection[2]:
                                self.TypesOfLines[connection[0]][connection[1]][
                                    indxadd1
                                ] = 1
                                self.TypesOfLines[connection[2]][connection[3]][
                                    indxadd2
                                ] = 4
                            else:
                                self.TypesOfLines[connection[0]][connection[1]][
                                    indxadd1
                                ] = 4
                                self.TypesOfLines[connection[2]][connection[3]][
                                    indxadd2
                                ] = 1

                subCellsAssignment = np.zeros(
                    (2 * self.darp_instance.rows, 2 * self.darp_instance.cols)
                )
                for i in range(self.darp_instance.rows):
                    for j in range(self.darp_instance.cols):
                        subCellsAssignment[2 * i][2 * j] = self.darp_instance.A[i][j]
                        subCellsAssignment[2 * i + 1][2 * j] = self.darp_instance.A[i][
                            j
                        ]
                        subCellsAssignment[2 * i][2 * j + 1] = self.darp_instance.A[i][
                            j
                        ]
                        subCellsAssignment[2 * i + 1][2 * j + 1] = self.darp_instance.A[
                            i
                        ][j]

                drone_turns = turns(AllRealPaths)
                drone_turns.count_turns()
                drone_turns.find_avg_and_std()
                self.mode_to_drone_turns.append(drone_turns)

                AllRealPaths_dict[mode] = AllRealPaths
                subCellsAssignment_dict[mode] = subCellsAssignment

            # Find mode with the smaller number of turns
            averge_turns = [x.avg for x in self.mode_to_drone_turns]
            self.min_mode = averge_turns.index(min(averge_turns))

            # Retrieve number of cells per robot for the configuration with the smaller number of turns
            min_mode_num_paths = [len(x) for x in AllRealPaths_dict[self.min_mode]]
            min_mode_returnPaths = AllRealPaths_dict[self.min_mode]

            # Uncomment if you want to visualize all available modes

            if self.darp_instance.visualization:
                for mode in range(4):
                    image = visualize_paths(
                        AllRealPaths_dict[mode],
                        subCellsAssignment_dict[mode],
                        self.darp_instance.droneNo,
                        self.darp_instance.color,
                    )
                    image.visualize_paths(mode)
                print("Best Mode:", self.min_mode)

            # Combine all modes to get one mode with the least available turns for each drone
            combined_modes_paths = []
            combined_modes_turns = []

            for r in range(self.darp_instance.droneNo):
                min_turns = sys.maxsize
                temp_path = []
                for mode in range(4):
                    if self.mode_to_drone_turns[mode].turns[r] < min_turns:
                        temp_path = self.mode_to_drone_turns[mode].paths[r]
                        min_turns = self.mode_to_drone_turns[mode].turns[r]
                combined_modes_paths.append(temp_path)
                combined_modes_turns.append(min_turns)

            self.best_case = turns(combined_modes_paths)
            self.best_case.turns = combined_modes_turns
            self.best_case.find_avg_and_std()

            # Retrieve number of cells per robot for the best case configuration
            best_case_num_paths = [len(x) for x in self.best_case.paths]
            best_case_returnPaths = self.best_case.paths

            # visualize best case
            if self.darp_instance.visualization:
                image = visualize_paths(
                    self.best_case.paths,
                    subCellsAssignment_dict[self.min_mode],
                    self.darp_instance.droneNo,
                    self.darp_instance.color,
                )
                image.visualize_paths("Combined Modes")

            self.execution_time = time.time() - start_time

            print(f"\nResults:")
            print(f"Number of cells per robot: {best_case_num_paths}")
            print(
                f"Minimum number of cells in robots paths: {min(best_case_num_paths)}"
            )
            print(
                f"Maximum number of cells in robots paths: {max(best_case_num_paths)}"
            )
            print(
                f"Average number of cells in robots paths: {np.mean(np.array(best_case_num_paths))}"
            )
            print(f"\nTurns Analysis: {self.best_case}")

    def CalcRealBinaryReg(self, BinaryRobotRegion, rows, cols):
        temp = np.zeros((2 * rows, 2 * cols))
        RealBinaryRobotRegion = np.zeros((2 * rows, 2 * cols), dtype=bool)
        for i in range(2 * rows):
            for j in range(2 * cols):
                temp[i, j] = BinaryRobotRegion[(int(i / 2))][(int(j / 2))]
                if temp[i, j] == 0:
                    RealBinaryRobotRegion[i, j] = False
                else:
                    RealBinaryRobotRegion[i, j] = True

        return RealBinaryRobotRegion

    def calculateMSTs(self, BinaryRobotRegions, droneNo, rows, cols, mode):
        MSTs = []
        for r in range(droneNo):
            k = Kruskal(rows, cols)
            k.initializeGraph(BinaryRobotRegions[r, :, :], True, mode)
            k.performKruskal()
            MSTs.append(k.mst)
        return MSTs


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "-grid",
        default=(10, 10),
        type=int,
        nargs=2,
        help="Dimensions of the Grid (default: (10, 10))",
    )
    argparser.add_argument(
        "-obs_pos",
        default=[5, 6, 7],
        nargs="*",
        type=int,
        help="Obstacles Positions (default: None)",
    )
    argparser.add_argument(
        "-in_pos",
        default=[0, 3, 9],
        nargs="*",
        type=int,
        help="Initial Positions of the robots (default: (1, 3, 9))",
    )
    argparser.add_argument(
        "-nep",
        action="store_true",
        help="Not Equal Portions shared between the Robots in the Grid (default: False)",
    )
    argparser.add_argument(
        "-portions",
        default=[0.2, 0.3, 0.5],
        nargs="*",
        type=float,
        help="Portion for each Robot in the Grid (default: (0.2, 0.7, 0.1))",
    )
    argparser.add_argument(
        "-vis",
        default=False,
        action="store_true",
        help="Visualize results (default: False)",
    )
    args = argparser.parse_args()

    MultiRobotPathPlanner(
        args.grid[0],
        args.grid[1],
        args.nep,
        args.in_pos,
        args.portions,
        args.obs_pos,
        args.vis,
    )
import numpy as np


class turns:
    def __init__(self, paths):
        """
        paths: List of lists of moves per drone
        """
        self.paths = paths
        self.turns = []

    def __str__(self):
        return (
            "\n"
            f"Turns: {self.turns}\n"
            f"Average: {self.avg:.3f}\n"
            f"Standard Deviation: {self.std:.3f}\n"
        )

    def count_turns(self):
        for path in self.paths:
            num_turns = -1
            last_move = ""
            for move in path:
                if move[0] == move[2]:
                    current_move = "horizontal"
                elif move[1] == move[3]:
                    current_move = "vertical"

                if last_move != current_move:
                    num_turns += 1

                last_move = current_move
            self.turns.append(num_turns)

    def find_avg_and_std(self):
        self.avg = np.average(self.turns)
        self.std = np.std(self.turns)


import sys
import pygame
from pygame.locals import KEYDOWN, K_q
import numpy as np
import time
from sklearn.preprocessing import MinMaxScaler

# CONSTANTS:
BLACK = (0, 0, 0)
GREY = (160, 160, 160)


class visualize_paths:
    def __init__(self, AllRealPaths, subCellsAssignment, DroneNo, color):
        self.AllRealPaths = AllRealPaths
        self.subCellsAssignment = subCellsAssignment
        min_max_scaler = MinMaxScaler(feature_range=(0, 800))
        self.dimensions = min_max_scaler.fit_transform(
            np.array(
                [self.subCellsAssignment.shape[0], self.subCellsAssignment.shape[1], 0]
            ).reshape(-1, 1)
        ).ravel()

        self.DroneNo = DroneNo
        self._VARS = {
            "surf": False,
            "gridWH": (self.dimensions[0], self.dimensions[1]),
            "gridOrigin": (0, 0),
            "gridCellsX": self.subCellsAssignment.shape[0],
            "gridCellsY": self.subCellsAssignment.shape[1],
            "lineWidth": 2,
        }
        self.color = color

    def visualize_paths(self, mode):
        pygame.init()
        self._VARS["surf"] = pygame.display.set_mode(
            (self.dimensions[1], self.dimensions[0])
        )
        pygame.display.set_caption("Mode: " + str(mode))
        while True:
            keep_going = self.checkEvents()
            if not keep_going:
                break
            self._VARS["surf"].fill(GREY)
            self.drawSquareGrid(
                self._VARS["gridOrigin"],
                self._VARS["gridWH"],
                self._VARS["gridCellsX"],
                self._VARS["gridCellsY"],
            )
            self.placeCells()
            pygame.display.update()

    def placeCells(self):
        cellBorder = 0
        celldimX = self._VARS["gridWH"][0] / self._VARS["gridCellsX"]
        celldimY = self._VARS["gridWH"][1] / self._VARS["gridCellsY"]

        for r in range(self.DroneNo):
            for point in self.AllRealPaths[r]:
                color = pygame.Color(255, 0, 0)
                pygame.draw.line(
                    self._VARS["surf"],
                    self.color[r],
                    (
                        self._VARS["gridOrigin"][0]
                        + (celldimX * point[1] + celldimX / 2),
                        self._VARS["gridOrigin"][1]
                        + (celldimY * point[0])
                        + celldimY / 2,
                    ),
                    (
                        self._VARS["gridOrigin"][0]
                        + (celldimX * point[3])
                        + celldimX / 2,
                        self._VARS["gridOrigin"][1]
                        + (celldimY * point[2])
                        + celldimY / 2,
                    ),
                    width=4,
                )

        cellBorder = 0

        for row in range(self.subCellsAssignment.shape[0]):
            for column in range(self.subCellsAssignment.shape[1]):
                if self.subCellsAssignment[row][column] == self.DroneNo:
                    self.drawSquareCell(
                        self._VARS["gridOrigin"][0]
                        + (celldimX * column)
                        + self._VARS["lineWidth"] / 2,
                        self._VARS["gridOrigin"][1]
                        + (celldimY * row)
                        + self._VARS["lineWidth"] / 2,
                        celldimX,
                        celldimY,
                        BLACK,
                    )

    # Draw filled rectangle at coordinates
    def drawSquareCell(self, x, y, dimX, dimY, color):
        pygame.draw.rect(self._VARS["surf"], color, (x, y, dimX, dimY))

    def drawSquareGrid(self, origin, gridWH, cellsX, cellsY):
        CONTAINER_WIDTH_HEIGHT = gridWH
        cont_x, cont_y = (0, 0)

        # DRAW Grid Border:
        # TOP lEFT TO RIGHT
        pygame.draw.line(
            self._VARS["surf"],
            BLACK,
            (cont_x, cont_y),
            (CONTAINER_WIDTH_HEIGHT[1] + cont_x, cont_y),
            self._VARS["lineWidth"],
        )

        # # BOTTOM lEFT TO RIGHT
        pygame.draw.line(
            self._VARS["surf"],
            BLACK,
            (cont_x, CONTAINER_WIDTH_HEIGHT[0] + cont_y),
            (CONTAINER_WIDTH_HEIGHT[1] + cont_x, CONTAINER_WIDTH_HEIGHT[0] + cont_y),
            self._VARS["lineWidth"],
        )

        # # LEFT TOP TO BOTTOM
        pygame.draw.line(
            self._VARS["surf"],
            BLACK,
            (cont_x, cont_y),
            (cont_x, cont_y + CONTAINER_WIDTH_HEIGHT[0]),
            self._VARS["lineWidth"],
        )
        # # RIGHT TOP TO BOTTOM
        pygame.draw.line(
            self._VARS["surf"],
            BLACK,
            (CONTAINER_WIDTH_HEIGHT[1] + cont_x, cont_y),
            (CONTAINER_WIDTH_HEIGHT[1] + cont_x, CONTAINER_WIDTH_HEIGHT[0] + cont_y),
            self._VARS["lineWidth"],
        )

        # Get cell size, just one since its a square grid.
        cellSizeX = CONTAINER_WIDTH_HEIGHT[0] / cellsX
        cellSizeY = CONTAINER_WIDTH_HEIGHT[1] / cellsY

        for x in range(cellsY):
            pygame.draw.line(
                self._VARS["surf"],
                BLACK,
                (cont_x + (cellSizeX * x), cont_y),
                (cont_x + (cellSizeX * x), CONTAINER_WIDTH_HEIGHT[0] + cont_y),
                2,
            )
        for y in range(cellsX):
            # # HORIZONTAl DIVISIONS
            pygame.draw.line(
                self._VARS["surf"],
                BLACK,
                (cont_x, cont_y + (cellSizeY * y)),
                (cont_x + CONTAINER_WIDTH_HEIGHT[1], cont_y + (cellSizeY * y)),
                2,
            )

    def checkEvents(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == KEYDOWN and event.key == K_q
            ):
                pygame.quit()
                return False
        return True


class darp_area_visualization(object):
    def __init__(self, Assignment_matrix, DroneNo, color, init_robot_pos):
        self.Assignment_matrix = Assignment_matrix
        min_max_scaler = MinMaxScaler(feature_range=(0, 800))
        dimensions = min_max_scaler.fit_transform(
            np.array(
                [self.Assignment_matrix.shape[0], self.Assignment_matrix.shape[1], 0]
            ).reshape(-1, 1)
        ).ravel()

        self.DroneNo = DroneNo
        self._VARS = {
            "surf": False,
            "gridWH": (dimensions[0], dimensions[1]),
            "gridOrigin": (0, 0),
            "gridCellsX": self.Assignment_matrix.shape[0],
            "gridCellsY": self.Assignment_matrix.shape[1],
            "lineWidth": 2,
        }
        self.color = color
        self.init_robot_pos_colors = [
            np.clip((r[0] - 20, r[1] + 20, r[2] - 20), 0, 255).tolist()
            for r in self.color
        ]
        self.init_robot_pos = init_robot_pos
        pygame.init()
        self._VARS["surf"] = pygame.display.set_mode((dimensions[1], dimensions[0]))
        self.checkEvents()
        self._VARS["surf"].fill(GREY)
        self.drawSquareGrid(
            self._VARS["gridOrigin"],
            self._VARS["gridWH"],
            self._VARS["gridCellsX"],
            self._VARS["gridCellsY"],
        )
        self.placeCells(self.Assignment_matrix)
        pygame.display.set_caption("Assignment Matrix")
        pygame.display.update()
        # time.sleep(5)

    def checkEvents(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == KEYDOWN and event.key == K_q:
                pygame.quit()
                sys.exit()

    def drawSquareGrid(self, origin, gridWH, cellsX, cellsY):
        CONTAINER_WIDTH_HEIGHT = gridWH
        cont_x, cont_y = (0, 0)

        # DRAW Grid Border:
        # TOP lEFT TO RIGHT
        pygame.draw.line(
            self._VARS["surf"],
            BLACK,
            (cont_x, cont_y),
            (CONTAINER_WIDTH_HEIGHT[1] + cont_x, cont_y),
            self._VARS["lineWidth"],
        )

        # # BOTTOM lEFT TO RIGHT
        pygame.draw.line(
            self._VARS["surf"],
            BLACK,
            (cont_x, CONTAINER_WIDTH_HEIGHT[0] + cont_y),
            (CONTAINER_WIDTH_HEIGHT[1] + cont_x, CONTAINER_WIDTH_HEIGHT[0] + cont_y),
            self._VARS["lineWidth"],
        )

        # # LEFT TOP TO BOTTOM
        pygame.draw.line(
            self._VARS["surf"],
            BLACK,
            (cont_x, cont_y),
            (cont_x, cont_y + CONTAINER_WIDTH_HEIGHT[0]),
            self._VARS["lineWidth"],
        )
        # # RIGHT TOP TO BOTTOM
        pygame.draw.line(
            self._VARS["surf"],
            BLACK,
            (CONTAINER_WIDTH_HEIGHT[1] + cont_x, cont_y),
            (CONTAINER_WIDTH_HEIGHT[1] + cont_x, CONTAINER_WIDTH_HEIGHT[0] + cont_y),
            self._VARS["lineWidth"],
        )

        # Get cell size, just one since its a square grid.
        cellSizeX = CONTAINER_WIDTH_HEIGHT[0] / cellsX
        cellSizeY = CONTAINER_WIDTH_HEIGHT[1] / cellsY

        for x in range(cellsY):
            pygame.draw.line(
                self._VARS["surf"],
                BLACK,
                (cont_x + (cellSizeX * x), cont_y),
                (cont_x + (cellSizeX * x), CONTAINER_WIDTH_HEIGHT[0] + cont_y),
                2,
            )
        for y in range(cellsX):
            # # HORIZONTAl DIVISIONS
            pygame.draw.line(
                self._VARS["surf"],
                BLACK,
                (cont_x, cont_y + (cellSizeY * y)),
                (cont_x + CONTAINER_WIDTH_HEIGHT[1], cont_y + (cellSizeY * y)),
                2,
            )

        pygame.display.update()

    def placeCells(self, Assignment_matrix, iteration_number=0):
        celldimX = self._VARS["gridWH"][0] / self._VARS["gridCellsX"]
        celldimY = self._VARS["gridWH"][1] / self._VARS["gridCellsY"]

        for row in range(self.Assignment_matrix.shape[0]):
            for column in range(self.Assignment_matrix.shape[1]):
                if self.Assignment_matrix[row][column] == self.DroneNo:
                    self.drawSquareCell(
                        self._VARS["gridOrigin"][0]
                        + (celldimX * column)
                        + self._VARS["lineWidth"] / 2,
                        self._VARS["gridOrigin"][1]
                        + (celldimY * row)
                        + self._VARS["lineWidth"] / 2,
                        celldimX,
                        celldimY,
                        BLACK,
                    )
                    continue
                for r in range(self.DroneNo):
                    if self.init_robot_pos[r] == (row, column):
                        self.drawSquareCell(
                            self._VARS["gridOrigin"][0]
                            + (celldimX * column)
                            + self._VARS["lineWidth"] / 2,
                            self._VARS["gridOrigin"][1]
                            + (celldimY * row)
                            + self._VARS["lineWidth"] / 2,
                            celldimX,
                            celldimY,
                            self.init_robot_pos_colors[r],
                        )
                        continue
                    else:
                        if self.Assignment_matrix[row][column] == r:
                            self.drawSquareCell(
                                self._VARS["gridOrigin"][0]
                                + (celldimX * column)
                                + self._VARS["lineWidth"] / 2,
                                self._VARS["gridOrigin"][1]
                                + (celldimY * row)
                                + self._VARS["lineWidth"] / 2,
                                celldimX,
                                celldimY,
                                self.color[r],
                            )

        self.drawSquareGrid(
            self._VARS["gridOrigin"],
            self._VARS["gridWH"],
            self._VARS["gridCellsX"],
            self._VARS["gridCellsY"],
        )

        pygame.display.set_caption(
            "Assignment Matrix [Iteration: " + str(iteration_number) + "]"
        )
        pygame.display.update()

    def drawSquareCell(self, x, y, dimX, dimY, color):
        pygame.draw.rect(self._VARS["surf"], color, (x, y, dimX, dimY))

def convert_to_directions(path):
    # Define a mapping from vector changes to directions
    direction_map = {
        (0, 1): "right",  # Moving right
        (1, 0): "down",  # Moving down
        (0, -1): "left",  # Moving left
        (-1, 0): "up",  # Moving up
    }

    directions = []
    for i in range(len(path) - 1):
        # Calculate movement vector
        move = (path[i + 1][2] - path[i][2], path[i + 1][3] - path[i][3])

        # Add the corresponding direction to the list
        directions.append(direction_map.get(move, "forward"))

    return directions

# import matplotlib.pyplot as plt
# import numpy as np

# def plot_robot_paths(grid_size, paths, obstacles, start_positions):
#     fig, ax = plt.subplots()
#     ax.set_xlim(0, grid_size[0])
#     ax.set_ylim(0, grid_size[1])
#     ax.set_xticks(np.arange(0, grid_size[0], 1))
#     ax.set_yticks(np.arange(0, grid_size[1], 1))
#     ax.grid()

#     # Plot obstacles
#     for obs in obstacles:
#         ax.plot(obs[1], obs[0], 'ks')  # Black square for obstacles

#     # Plot paths
#     colors = ['r', 'g', 'b', 'c', 'm', 'y']  # Add more colors for more robots
#     for idx, path in enumerate(paths):
#         x, y = zip(*[(move[1], move[0]) for move in path])  # Extract x and y coordinates
#         ax.plot(x, y, color=colors[idx % len(colors)] )  # Use different color for each path

#         # Starting positions
#         if idx < len(start_positions):
#             ax.plot(start_positions[idx][1], start_positions[idx][0], colors[idx % len(colors)] + 'X')  # Starting position

#     plt.gca().invert_yaxis()  # Invert y axis to match grid coordinate system
#     plt.show()

# # Example usage
# grid_size = (8, 8)  # Grid dimensions
# paths = mpp_instance.best_case.paths  # Paths from your MultiRobotPathPlanner instance
# obstacles = [(obs // grid_size[0], obs % grid_size[1]) for obs in [4 ,12, 20, 40, 41, 42]]  # Convert obstacle indices to coordinates
# start_positions = [(pos // grid_size[0], pos % grid_size[1]) for pos in [32,30,50]]  # Convert start positions to coordinates
# plot_robot_paths(grid_size, paths, obstacles, start_positions)


import matplotlib.pyplot as plt
import numpy as np

def plot_area_coverage(grid, drone_no, colors):
    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap=plt.cm.get_cmap('viridis', drone_no))
    plt.colorbar(ticks=range(drone_no))
    plt.clim(-0.5, drone_no - 0.5)
    plt.grid(which='both', color='lightgrey', linewidth=0.5)
    plt.title("Area Coverage per Robot")
    plt.show()

# area_grid = np.array(mpp_instance.A)
# colors = ['red', 'green', 'blue']
# plot_area_coverage(area_grid, mpp_instance.darp_instance.droneNo, colors)

def calculate_path_length(path):
    return len(path)  # Simple measure; replace with actual distance calculation if available

def plot_efficiency_metrics(paths):
    total_distances = [calculate_path_length(path) for path in paths]
    total_distance = sum(total_distances)
    average_distance = np.mean(total_distances)
    std_deviation = np.std(total_distances)

    fig, ax = plt.subplots()
    metrics = ['Total Distance', 'Average Distance', 'Std Deviation']
    values = [total_distance, average_distance, std_deviation]
    ax.bar(metrics, values, color=['red', 'green', 'blue'])
    plt.title('Efficiency Metrics')
    for i, v in enumerate(values):
        ax.text(i, v + 0.5, f"{v:.2f}", color='black', ha='center')
    plt.show()


@app.route("/explore", methods=["get"])
def explore():
    nx = 8  
    ny = 8 
    notEqualPortions = False 
    initial_positions = [32, 30, 50]  
    portions = [1 / 3, 1 / 3, 1 / 3] 
    obs_pos = [4, 12, 20, 40, 41, 42]
    mpp_instance = MultiRobotPathPlanner(
        nx, ny, notEqualPortions, initial_positions, portions, obs_pos, False
    )
    robotPaths = {
        "EPUCK1": convert_to_directions(mpp_instance.best_case.paths[0]),
        "EPUCK2": convert_to_directions(mpp_instance.best_case.paths[1]),
        "EPUCK3": convert_to_directions(mpp_instance.best_case.paths[2]),
    }
    return robotPaths


if __name__ == "__main__":
    app.run()
