class Cell {
    constructor(x, y) {
        this.x = x;
        this.y = y;
        this.isWall = false;
        this.isExplored = false;
    }
}
let map = {}
function initializeMaze(width, height, walls) {
    let maze = [];
    for (let x = 0; x < width; x++) {
        maze[x] = [];
        for (let y = 0; y < height; y++) {
            maze[x][y] = new Cell(x, y);
        }
    }
    for (let wall of walls) {
        maze[wall[0]][wall[1]].isWall = true;
    }
    return maze;
}

function heuristic(a, b) {
    return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
}

// Implement PriorityQueue or use a library
class PriorityQueue {
    constructor() {
        this.elements = [];
    }

    enqueue(priority, item) {
        this.elements.push({item, priority});
        this.elements.sort((a, b) => a.priority - b.priority);
    }

    dequeue() {
        return this.elements.shift().item;
    }

    isEmpty() {
        return this.elements.length === 0;
    }
}

function getNeighbors(maze, cell) {
    const directions = [[0, 1], [1, 0], [0, -1], [-1, 0]];
    let neighbors = [];
    for (let [dx, dy] of directions) {
        let x = cell.x + dx, y = cell.y + dy;
        if (x >= 0 && x < maze.length && y >= 0 && y < maze[0].length && !maze[x][y].isWall) {
            neighbors.push(maze[x][y]);
        }
    }
    return neighbors;
}

function aStarSearch(maze, start, goal) {
    let openSet = new PriorityQueue();
    openSet.enqueue(0, start);
    let cameFrom = new Map();
    cameFrom.set(start, null);
    let costSoFar = new Map();
    costSoFar.set(start, 0);

    while (!openSet.isEmpty()) {
        let current = openSet.dequeue();

        if (current === goal) {
            break;
        }

        for (let next of getNeighbors(maze, current)) {
            let newCost = costSoFar.get(current) + 1;
            if (!costSoFar.has(next) || newCost < costSoFar.get(next)) {
                costSoFar.set(next, newCost);
                let priority = newCost + heuristic(goal, next);
                openSet.enqueue(priority, next);
                cameFrom.set(next, current);
            }
        }
    }

    return reconstructPath(cameFrom, start, goal);
}
function exploreMaze(maze, start, goal) {
    let path = aStarSearch(maze, start, goal);
    if (!path) {
        console.log("No path found from start to goal.");
        return null;
    }
    for (let cell of path) {
        cell.isExplored = true;
    }
    return path;
}

function reconstructPath(cameFrom, start, goal) {
    let current = goal;
    let path = [];
    while (current !== start) {
        path.push(current);
        current = cameFrom.get(current);
    }
    path.push(start);
    path.reverse();
    return path;
}
function printMaze(maze) {
    for (let row of maze) {
        let rowString = '';
        for (let cell of row) {
            rowString += cell.isWall ? '#' : '.';
            rowString += ' ';
        }
        console.log(rowString);
    }
}

function getMovementInstructions(path) {
    const directions = { 'up': [-1, 0], 'right': [0, 1], 'down': [1, 0], 'left': [0, -1] };
    const directionChange = {
        'up-right': 'Right', 'right-down': 'Right', 'down-left': 'Right', 'left-up': 'Right',
        'up-left': 'Left', 'left-down': 'Left', 'down-right': 'Left', 'right-up': 'Left'
    };
    let instructions = [];
    let currentDirection = 'up';

    for (let i = 1; i < path.length; i++) {
        let prevCell = path[i - 1];
        let currentCell = path[i];
        let movement = [currentCell.x - prevCell.x, currentCell.y - prevCell.y];

        let foundDirection = Object.keys(directions).find(direction =>
            directions[direction][0] === movement[0] && directions[direction][1] === movement[1]);

        if (foundDirection !== currentDirection) {
            instructions.push(directionChange[currentDirection + '-' + foundDirection]);
            currentDirection = foundDirection;
        }
        instructions.push("Forward");
    }

    return instructions;
}

// Example usage
let walls = [[0, 5], [1, 5], [2, 5], [3, 5]];
let maze = initializeMaze(10, 10, walls);
let start = maze[6][9];
let goal = maze[0][0];
let path = exploreMaze(maze, start, goal);
printMaze(maze);
if (path) {
    let movementInstructions = getMovementInstructions(path);
    console.log("Movement Instructions:");
    for (let instruction of movementInstructions) {
        console.log(instruction);
    }
} else {
    console.log("No path found from start to goal.");
}
