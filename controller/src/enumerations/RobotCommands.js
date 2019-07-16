const Robot = Object.freeze({
    WHEELS:     Symbol('wheels'),
    FRONTDRUM: Symbol('frontdrum'),
    BACKDRUM: Symbol('backdrum'),
    FRONTARM: Symbol('frontarm'),
    BACKARM: Symbol('backarm')
});

const Operation = Object.freeze({
    STOPWHEELS: 'stop',
    DRIVEFORWARD: 'forward',
    DRIVEBACKWARD: 'backward',
    TURNLEFT: 'left',
    TURNRIGHT: 'right',
    UP: 1,
    DOWN: -1,
    OUTWARD: 1,
    INWARD: -1,
    STOP: 0
});

export {
    Robot, Operation
}