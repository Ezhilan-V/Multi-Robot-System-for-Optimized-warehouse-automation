class CalculateTrajectories {
    constructor(r, c, MST) {
        this.MAX_NODES = 4 * r * c;
        this.PathSequence = [];
        this.rows = r;
        this.cols = c;
        this.MSTvector = MST;
        this.MSTedges = this.MSTvector.length;
        this.allEdges = new Set();
        this.nodes = {};
        for (let node = 0; node < this.MAX_NODES; node++) {
            this.nodes[node] = null;
        }
    }
    initializeGraph(A, connect4) {
        for (let i = 0; i < 2 * this.rows; i++) {
            for (let j = 0; j < 2 * this.cols; j++) {
                if (A[i][j]) {
                    if (i > 0 && A[i - 1][j]) {
                        this.AddToAllEdges(
                            i * 2 * this.cols + j, (i - 1) * 2 * this.cols + j, 1
                        );
                    }
                    if (i < 2 * this.rows - 1 && A[i + 1][j]) {
                        this.AddToAllEdges(
                            i * 2 * this.cols + j, (i + 1) * 2 * this.cols + j, 1
                        );
                    }
                    if (j > 0 && A[i][j - 1]) {
                        this.AddToAllEdges(
                            i * 2 * this.cols + j, i * 2 * this.cols + j - 1, 1
                        );
                    }
                    if (j < 2 * this.cols - 1 && A[i][j + 1]) {
                        this.AddToAllEdges(
                            i * 2 * this.cols + j, i * 2 * this.cols + j + 1, 1
                        );
                    }
                    if (!connect4) {
                        if (i > 0 && j > 0 && A[i - 1][j - 1]) {
                            this.AddToAllEdges(
                                i * 2 * this.cols + j,
                                (i - 1) * 2 * this.cols + j - 1,
                                1
                            );
                        }
                        if (
                            i < 2 * this.rows - 1
                            && j < 2 * this.cols - 1
                            && A[i + 1][j + 1]
                        ) {
                            this.AddToAllEdges(
                                i * 2 * this.cols + j,
                                (i + 1) * 2 * this.cols + j + 1,
                                1
                            );
                        }
                        if (i > 2 * this.rows - 1 && j > 0 && A[i + 1][j - 1]) {
                            this.AddToAllEdges(
                                i * 2 * this.cols + j,
                                (i + 1) * 2 * this.cols + j - 1,
                                1
                            );
                        }
                        if (i > 0 && j < 2 * this.cols - 1 && A[i - 1][j + 1]) {
                            this.AddToAllEdges(
                                i * 2 * this.cols + j,
                                (i - 1) * 2 * this.cols + j + 1,
                                1
                            );
                        }
                    }
                }
            }
        }
    }
    AddToAllEdges(_from, to, cost) {
        this.allEdges.add(new Edge(_from, to, cost));
        if (this.nodes[_from] === null) {
            this.nodes[_from] = new Set();
        }
        this.nodes[_from].add(to);
        if (this.nodes[to] === null) {
            this.nodes[to] = new Set();
        }
        this.nodes[to].add(_from);
    }
    RemoveTheAppropriateEdges() {
        for (let i = 0; i < this.MSTedges; i++) {
            let e = this.MSTvector[i];
            let maxN = Math.max(e.src, e.dst);
            let minN = Math.min(e.src, e.dst);
            if (Math.abs(e.src - e.dst) === 1) {
                let alpha = (4 * minN + 3) - 2 * (maxN % this.cols);
                let eToRemove = new Edge(alpha, alpha + 2 * this.cols, 1);
                let eToRemoveMirr = new Edge(alpha + 2 * this.cols, alpha, 1);
                let eToRemove2 = new Edge(alpha + 1, alpha + 1 + 2 * this.cols, 1);
                let eToRemove2Mirr = new Edge(
                    alpha + 1 + 2 * this.cols, alpha + 1, 1
                );
                if (this.allEdges.has(eToRemove)) {
                    this.SafeRemoveEdge(eToRemove);
                }
                if (this.allEdges.has(eToRemoveMirr)) {
                    this.SafeRemoveEdge(eToRemoveMirr);
                }
                if (this.allEdges.has(eToRemove2)) {
                    this.SafeRemoveEdge(eToRemove2);
                }
                if (this.allEdges.has(eToRemove2Mirr)) {
                    this.SafeRemoveEdge(eToRemove2Mirr);
                }
            } else {
                let alpha = (4 * minN + 2 * this.cols) - 2 * (maxN % this.cols);
                let eToRemove = new Edge(alpha, alpha + 1, 1);
                let eToRemoveMirr = new Edge(alpha + 1, alpha, 1);
                let eToRemove2 = new Edge(alpha + 2 * this.cols, alpha + 1 + 2 * this.cols, 1);
                let eToRemove2Mirr = new Edge(
                    alpha + 1 + 2 * this.cols, alpha + 2 * this.cols, 1
                );
                if (this.allEdges.has(eToRemove)) {
                    this.SafeRemoveEdge(eToRemove);
                }
                if (this.allEdges.has(eToRemoveMirr)) {
                    this.SafeRemoveEdge(eToRemoveMirr);
                }
                if (this.allEdges.has(eToRemove2)) {
                    this.SafeRemoveEdge(eToRemove2);
                }
                if (this.allEdges.has(eToRemove2Mirr)) {
                    this.SafeRemoveEdge(eToRemove2Mirr);
                }
            }
        }
    }
    SafeRemoveEdge(curEdge) {
        try {
            this.allEdges.delete(curEdge);
            
            if (this.nodes[curEdge.src].has(curEdge.dst)) {
                this.nodes[curEdge.src].delete(curEdge.dst);
            }
            if (this.nodes[curEdge.dst].has(curEdge.src)) {
                this.nodes[curEdge.dst].delete(curEdge.src);
            }
        } catch (error) {
            
            console.log("TreeSet should have contained this element!!");
            process.exit(1);
        }
    }
    CalculatePathsSequence(StartingNode) {
        let currentNode = StartingNode;
        let RemovedNodes = new Set();
        let movement = [];
        let PathSequence = [];
        movement.push(2 * this.cols);
        movement.push(-1);
        movement.push(-2 * this.cols);
        movement.push(1);
        let found = false;
        let prevNode = 0;
        for (let idx = 0; idx < 4; idx++) {
            if (this.nodes[currentNode].has(currentNode + movement[idx])) {
                prevNode = currentNode + movement[idx];
                found = true;
                break;
            }
        }
        if (!found) {
            return;
        }
        while (true) {
            if (currentNode !== StartingNode) {
                RemovedNodes.add(currentNode);
            }
            let offset = movement.indexOf(prevNode - currentNode);
            prevNode = currentNode;
            found = false;
            for (let idx = 0; idx < 4; idx++) {
                if (
                    this.nodes[prevNode].has(prevNode + movement[(idx + offset) % 4])
                && !(RemovedNodes.has(prevNode + movement[(idx + offset) % 4]))
                ) {
                    currentNode = prevNode + movement[(idx + offset) % 4];
                    found = true;
                    break;
                }
            }
            if (!found) {
                return;
            }
            if (this.nodes[currentNode].has(prevNode)) {
                this.nodes[currentNode].delete(prevNode);
            }
            if (this.nodes[prevNode].has(currentNode)) {
                this.nodes[prevNode].delete(currentNode);
            }
            let i = Math.floor(currentNode / (2 * this.cols));
            let j = currentNode % (2 * this.cols);
            let previ = Math.floor(prevNode / (2 * this.cols));
            let prevj = prevNode % (2 * this.cols);
            this.PathSequence.push([previ, prevj, i, j]);
        }
    }
}
class Edge {
    constructor(src, dst, cost) {
        this.src = src;
        this.dst = dst;
        this.cost = cost;
    }
}
import numpy as np;
import sys;
import cv2;

import time;
import random;
import os;
from numba import njit;
np.set_printoptions(threshold=sys.maxsize);
random.seed(1);
os.environ["PYTHONHASHSEED"] = str(1);
np.random.seed(1);
@njit(fastmath=True)
function assign(droneNo, rows, cols, GridEnv, MetricMatrix, A) {
    let ArrayOfElements = np.zeros(droneNo);
    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
            if (GridEnv[i][j] === -1) {
                let minV = MetricMatrix[0][i][j];
                let indMin = 0;
                for (let r = 0; r < droneNo; r++) {
                    if (MetricMatrix[r][i][j] < minV) {
                        minV = MetricMatrix[r][i][j];
                        indMin = r;
                    }
                }
                A[i][j] = indMin;
                ArrayOfElements[indMin] += 1;
            } else if (GridEnv[i][j] === -2) {
                A[i][j] = droneNo;
            }
        }
    }
    return [A, ArrayOfElements];
}
@njit(fastmath=True)
function inverse_binary_map_as_uint8(BinaryMap) {
    
    return np.logical_not(BinaryMap).astype(np.uint8);
}
@njit(fastmath=True)
function euclidian_distance_points2d(array1, array2) {
    
    return (((array1[0] - array2[0]) ** 2) + ((array1[1] - array2[1]) ** 2)) ** 0.5;
}
@njit(fastmath=True)
function constructBinaryImages(labels_im, robo_start_point, rows, cols) {
    let BinaryRobot = np.copy(labels_im);
    let BinaryNonRobot = np.copy(labels_im);
    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
            if (labels_im[i][j] === labels_im[robo_start_point]) {
                BinaryRobot[i][j] = 1;
                BinaryNonRobot[i][j] = 0;
            } else if (labels_im[i][j] !== 0) {
                BinaryRobot[i][j] = 0;
                BinaryNonRobot[i][j] = 1;
            }
        }
    }
    return [BinaryRobot, BinaryNonRobot];
}
@njit(fastmath=True)
function CalcConnectedMultiplier(rows, cols, dist1, dist2, CCvariation) {
    let returnM = np.zeros([rows, cols]);
    let MaxV = 0;
    let MinV = 2**30;
    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
            returnM[i][j] = dist1[i][j] - dist2[i][j];
            if (MaxV < returnM[i][j]) {
                MaxV = returnM[i][j];
            }
            if (MinV > returnM[i][j]) {
                MinV = returnM[i][j];
            }
        }
    }
    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
            returnM[i][j] = (returnM[i][j] - MinV) * (
                (2 * CCvariation) / (MaxV - MinV)
            ) + (1 - CCvariation);
        }
    }
    return returnM;
}


