// Pop = population

const canvas = document.getElementById("leCanvas");
const ctx = canvas.getContext("2d");
const pxPerUnit = 100;
mouseXPos = 0
mouseYPos = 0
keysPressed = new Set()

const teamColorsIn = ["lightgray", "lightgreen", "lightblue", "indianred"]
const teamColorsOut = ["black", "green", "blue", "red"]
const teamNames = ["Neutral", "Green", "Blue", "Red"]

teamThatWon = -1

// Game rules
const team = 1
const popIncreaseSpeed = 0.8 // seconds
const sizeFromPop = false
const defaultSize = 50
const neutralMaxPop = 10


testGameMap = [
    {
        x: 50,
        y: 80,
        pop: 10,
        team: 1
    },
    {
        x: 600,
        y: 600,
        pop: 10,
        team: 3
    },
    {
        x: 520,
        y: 200,
        pop: 10,
        team: 0
    },
    {
        x: 80,
        y: 400,
        pop: 10,
        team: 0
    },
    {
        x: 120,
        y: 200,
        pop: 10,
        team: 0
    },
    {
        x: 340,
        y: 420,
        pop: 10,
        team: 0
    }
]

setInterval(draw, 10); // Call draw every 10 milliseconds
setInterval(increasePop, popIncreaseSpeed * 1000)
setInterval(checkWinLooseCondition, 100)


function checkWinLooseCondition() {
    t = testGameMap[0].team
    for (var i = 0; i < testGameMap.length; i++)
    {
        if (testGameMap[i].team != t)
            return
    } 
    teamThatWon = t
}

function increasePop() {
    for (var i = 0; i < testGameMap.length; i++)
    {
        o = testGameMap[i];
        if (o.team == 0) {
            if (o.pop < neutralMaxPop) o.pop += 1
        }
        else {
            o.pop += 1
        }
    }
}

/* important! for alignment, you should make things
* relative to the canvas' current width/height.
*/
function draw() {
    ctx.canvas.width  = window.innerWidth;
    ctx.canvas.height = window.innerHeight;
    drawMap()
    drawTitle()

    canvas.style.background = "lightgrey";
    if (teamThatWon > 0) {
        canvas.style.background = teamColorsIn[teamThatWon];
        box(0, window.innerHeight/2 - 84, window.innerWidth, 100, teamColorsOut[teamThatWon], teamColorsOut[teamThatWon])
        text(teamNames[teamThatWon] + " Won!", window.innerWidth/2, window.innerHeight/2, "100px Arial", "black")
    }
}

function drawTitle() {
    text(teamNames[team] + " team", window.innerWidth / 2, 50, "30px Arial", teamColorsOut[team])
}

function drawMap() {
    for (var i = 0; i < testGameMap.length; i++)
    {
        o = testGameMap[i];
        o.size = defaultSize
        if (sizeFromPop) {
            o.size = o.pop
        }
        size = o.size

        if (Object.hasOwn(o, 'select') == false) {
            o.select = false
        }

        if (o.select == true) {
            size += 5;
        }

        circle(o.x, o.y, size, teamColorsIn[o.team], teamColorsOut[o.team]);
        text(o.pop, o.x, o.y-15, "30px Arial", "black")
    }
}


function circle(x, y, r, colorIn, colorOut) {
    ctx.beginPath();
    ctx.arc(x, y, r, 0, 2 * Math.PI);
    ctx.fillStyle = colorIn;
    ctx.fill();
    ctx.lineWidth = 5
    ctx.strokeStyle = colorOut 
    ctx.stroke();
}

function box(x,y,w,h,colorIn,colorOut) {
    ctx.fillStyle = colorIn;
    ctx.fillRect(x,y,w,h);
    ctx.lineWidth = 5;
    ctx.strokeStyle = colorOut
    ctx.strokeRect(x,y,w,h);
}

function text(text, x, y, font, color) {
    ctx.font = font;
    ctx.fillStyle = color
    ctx.fillText(text, x - ctx.measureText(text).width/2, y);
}


// Mouse
function select(x,y,right) {
    // Left click select
    // Right click move/attack
    // Can only select nodes on the same team
    // Shift click selects multiple units (doesn't deselect units)
    selectedObj = null
    for (var i = 0; i < testGameMap.length; i++)
    {
        o = testGameMap[i];
        dist = Math.sqrt(Math.pow(x - o.x, 2) + Math.pow(y - o.y, 2));
        if (dist <= o.size) {
            selectedObj = o;
        }
    }
    shift = key(16)
    if (right) { // right click is attack/move
        if (selectedObj == null) return;
        popToSend = 0;
        for (var i = 0; i < testGameMap.length; i++) {
            o = testGameMap[i];
            if (o.select == false)
                continue;
            popToSend += o.pop;
            o.pop = 0
        }
        if (selectedObj.team == team) {
            selectedObj.pop += popToSend
        }
        else {
            selectedObj.pop -= popToSend
            if (selectedObj.pop < 0) {
                selectedObj.pop = -selectedObj.pop
                selectedObj.team = team
                selectedObj.select = true
            }
        }
    }
    else { // left click is select
        if (!shift) { // shift is for selecting multiple nodes. not shifting means selecting 1 node
            for (var i = 0; i < testGameMap.length; i++)
            {
                testGameMap[i].select = false;
            }
        }
        if (selectedObj != null) {
            selectedObj.select = true;
        }
    }
}


function mouseDown(canvas, event) {
    let rect = canvas.getBoundingClientRect();
    let x = event.clientX - rect.left;
    let y = event.clientY - rect.top;
}

function mouseUp(canvas, event) {
    let rect = canvas.getBoundingClientRect();
    let x = event.clientX - rect.left;
    let y = event.clientY - rect.top;
    right = event.button == 2
    select(x,y, right)
}

function mouseMove(canvas, event) {
    let rect = canvas.getBoundingClientRect();
    mouseXPos = event.clientX - rect.left;
    mouseYPos = event.clientY - rect.top;
}

canvas.addEventListener("mousedown", function (e) {
    mouseDown(canvas, e);
});

canvas.addEventListener("mouseup", function (e) {
    mouseUp(canvas, e);
});

canvas.addEventListener("mousemove", function (e) {
    mouseMove(canvas, e);
});


// Keyboard
function keyDown(e) {
    keysPressed.add(e.keyCode);
}

function keyUp(e) {
    keysPressed.delete(e.keyCode);
}

function key(keyCode) {
    return keysPressed.has(keyCode)
}

window.addEventListener('keydown',keyDown,false);
window.addEventListener('keyup',keyUp,false);

// Remove context menu on right click
canvas.oncontextmenu = function (e) {
    e.preventDefault();
};