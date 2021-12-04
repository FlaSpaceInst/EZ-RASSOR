const Robot = Object.freeze({
    WHEELS: Symbol('wheels'),
    ARM: Symbol('arm'),
    FRONTDRUM: Symbol('frontdrum'),
    BACKDRUM: Symbol('backdrum'),
    FRONTARM: Symbol('frontarm'),
    BACKARM: Symbol('backarm'),
    AUTONOMY: Symbol('autonomy'),
    ALL: Symbol('all')
});

const Operation = Object.freeze({
    // Body
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
    // Full Arm 
    ARMUP: 'armup',
    ARMDOWN: 'armdown',
    ARMLEFT: 'armleft',
    ARMRIGHT: 'armright',
    ARMFORWARD: 'armforward',
    ARMBACKWORD: 'armbackward',

    // End Arm
    GRABBERUP: 'grabberup',
    GRABBERDOWN: 'grabberdown',
    GRABBERLEFT: 'grabberleft',
    GRABBERRIGHT: 'grabberright',

    // rotate
    ROTATELEFT: 'rotateleft',
    ROTATERIGHT: 'rotateright',
    
    // Arm controls
    STOPARM: 'stoparm',

    // Autonomous Arm

    PICKUP: 'pickup',
    PLACE: 'place',
    HOME: 'home',

    close: 1,
    open: 0,

    // Autonomous Functions
    DRIVE: 1,
    DIG: 2,
    DUMP: 4,
    SELFRIGHT: 8,
    FULLAUTONOMY: 16,
});

export {
    Robot, Operation
}