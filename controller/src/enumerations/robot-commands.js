const Robot = Object.freeze({
    WHEELS: Symbol('wheels'),
    FRONTDRUM: Symbol('frontdrum'),
    BACKDRUM: Symbol('backdrum'),
    FRONTARM: Symbol('frontarm'),
    BACKARM: Symbol('backarm'),
    AUTONOMY: Symbol('autonomy'),
    ALL: Symbol('all')
});

const Operation = Object.freeze({
    STOPWHEELS: 'stop',
    DRIVEFORWARD: 'forward',
    DRIVEBACKWARD: 'backward',
    TURNLEFT: 'left',
    TURNRIGHT: 'right',
    UP: 1,
    DOWN: -1,
    ROTATEOUTWARD: 1,
    ROTATEINWARD: -1,
    STOP: 0,

    // Autonomous Functions
    DRIVE: 1,
    DIG: 2,
    DUMP: 4,
    SELFRIGHT: 8,
    FULLAUTONOMY: 16
});

export {
    Robot, Operation
}