const express = require('express');
const cors = require('cors');
const http = require('http');

// Initialize Express app
const app = express();
app.use(express.json());
app.use(cors());

// Initialize data structures for collaborative physical mapping (CPMs) and robot positions
let CPMs = {};
let robot_positions = [
    { name: "EPUCK1", position: ['0.375', '0.625', '0'], pathLength: null, path: null },
    { name: "EPUCK2", position: ['-0.625', '-0.125', '0'], pathLength: null, path: null },
    { name: "EPUCK3", position: ['0.125', '0.125', '0'], pathLength: null, path: null }
];
let assigned_robot = null
class Cell {
    constructor(x, y) {
        this.x = x;
        this.y = y;
        this.isWall = false;
        this.isExplored = false;
    }
}
let map = {}
const walls = [
    [0, 5], [1, 5], [2, 5], [3, 5], [6, 9], [6, 8], [6, 7],
    [0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7], [0, 8], [0, 9],
    [9, 0], [9, 1], [9, 2], [9, 3], [9, 4], [9, 5], [9, 6], [9, 7], [9, 8], [9, 9],
    [0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0], [8, 0], [9, 0],
    // [0, 9], [1, 9], [2, 9], [3, 9], [4, 9], [5, 9], [6, 9], [7, 9], [8, 9], [9, 9],
];
const maze = initializeMaze(10, 10, walls);
let current_goal = null

app.get('/update', (req, res) => {
    const { name, radius, speed } = req.query;
    CPMs[name] = { name, radius, speed };
    //console.log(CPMs);
    res.json({ response: 'received' });
});
app.get('/nav', (req, res) => {
    res.json(assigned_robot)
});

app.get('/retrieve', (req, res) => {
    res.json(CPMs);
});

app.get('/getposition', (req, res) => {
    res.json(robot_positions);
});

app.get('/position', (req, res) => {

    updateRobotPositions(req.query);
    res.json({ response: 'received' });
});

app.get('/target', (req, res) => {
    const x = Number(req.query.x);
    const y = Number(req.query.y);
    const goal = [x, y];
    current_goal = goal
    let minDist = 64;
    let minRobot = null;

    robot_positions.forEach(robot => {
        updateRobotPath(robot, goal);
        if (!minRobot || robot.pathLength < minDist) {
            minDist = robot.pathLength;
            minRobot = robot;
        }
    });
    assigned_robot = minRobot
    res.json({ message: `${minRobot.name} has the shortest path and will be sent to target location ` + minRobot.path });
});
app.get('/', (req, res) => {
    res.sendFile(__dirname + '/control.html')
})
app.get('/maze', (req, res) => {
    res.json(maze);
});
app.get('/explore', (req, res) => {
    http.get('http://127.0.0.1:5000/explore', (apiRes) => {
        let data = '';

        // Listen for data
        apiRes.on('data', (chunk) => {
            data += chunk;
        });

        // When the response has finished
        apiRes.on('end', () => {
            const directionChange = {
                'up-right': 'Right', 'right-down': 'Right', 'down-left': 'Right', 'left-up': 'Right',
                'up-left': 'Left', 'left-down': 'Left', 'down-right': 'Left', 'right-up': 'Left'
            }
            for (n of Object.keys(JSON.parse(data))) {
                path = JSON.parse(data)[n]
                val = []
                for (let i = 1; i < path.length; i++) {
                    val.push(directionChange[path[i - 1] + '-' + path[i]]);
                    val.push("Forward");
                }
                robot_positions.find(_ => _.name = n).path = val
            }
        }).on('error', (e) => {
            console.error(e);
            res.status(500).send('Something went wrong');
        });
    })
    res.send(robot_positions)// Send the data to the client
});

// Server configuration
const PORT = process.env.PORT || 4500;
app.listen(PORT, () => {
    console.log(`Server is running on port ${PORT}`);
});

// Helper functions

function updateRobotPositions(query) {
    // //console.log(query);
    robot_positions.forEach(robot => {
        // if (robot.name === query[robot.name.toLowerCase()]) {
        robot.position = [
            Number(query[robot.name + '_x']),
            Number(query[robot.name + '_y']),
            Number(query[robot.name + '_angle'])
        ];
        // }
        //console.log(robot.name, convertCoordinate(robot.position[0]), convertCoordinate(robot.position[1]));
        if (assigned_robot && robot.name === assigned_robot.name) {
            //console.log(convertCoordinate(robot.position[0]), convertCoordinate(robot.position[1]), current_goal)
            if (convertCoordinate(robot.position[0]) == current_goal[0]) {
                if (convertCoordinate(robot.position[0]) == current_goal[0]) {
                    assigned_robot = null
                }
            }
        }
    });

}

function updateRobotPath(robot, goal) {
    let start = [Number(robot.position[0]), Number(robot.position[1])];
    robot.path = exploreMaze(maze, start, goal);
    robot.pathLength = robot.path ? robot.path.filter(instruction => instruction === "Forward").length : null;
}

// functions to calculate instructions

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

class PriorityQueue {
    constructor() {
        this.elements = [];
    }

    enqueue(priority, item) {
        this.elements.push({ item, priority });
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
function convertCoordinate(inputValue) {
    // Normalize the input value to the range [0, 1.75]
    let normalizedValue = inputValue + 0.875;

    // Scale to the range [0, 7] and convert to integer
    let convertedValue = Math.floor(normalizedValue * 8 / 1.75);

    // Ensure the value is within the target range [0, 7]
    convertedValue = Math.max(0, Math.min(convertedValue, 7));
    return convertedValue;
}
function exploreMaze(maze, start, goal) {
    // //console.log(convertCoordinate(start[0]))
    // let mazeStart = maze[convertCoordinate(start[0])][convertCoordinate(start[1])];
    // let mazeGoal = maze[convertCoordinate(goal[0])][convertCoordinate(goal[1])];

    mazeStart = maze[convertCoordinate(start[0])][convertCoordinate(start[1])]
    mazeGoal = maze[goal[0]][goal[1]]
    let path = aStarSearch(maze, mazeStart, mazeGoal);
    if (!path) {
        //console.log("No path found from start to goal.");
        return null;
    }
    for (let cell of path) {
        cell.isExplored = true;
    }
    if (path) {
        var movementInstructions = getMovementInstructions(path);
        //console.log("Movement Instructions:");
        for (let instruction of movementInstructions) {
            //console.log(instruction);
        }
    } else {
        //console.log("No path found from start to goal.");
    }
    return movementInstructions;
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
        //console.log(rowString);
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
printMaze(maze)