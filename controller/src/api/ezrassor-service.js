import {Robot, Operation} from '../enumerations/RobotCommands';
import HTTP from './web-commands';

export default class EZRASSOR { 

    constructor(host, route = '') {
        this.host = host;
        this.route = route;
        this.allStop();
    }

    // Getters and Setters
    get host () {
        return this._host;
    }

    set host(value) {
        this._host = value;
    }

    get route() {
        return this._route;
    }

    set route(value) {
        if(value[0] === '/') {
            this._route = value.substring(1);
            return;
        }

        this._route = value;
    }

    // Build complete apiPath for HTTP requests
    get apiPath() {
        return 'http://' + this.host + '/' + this.route;
    } 

    // Return custom twist message
    get twistMsg() {
        return JSON.stringify(this._twistMsg);
    }

    // Update only the instruction needed
    updateTwistMsg(instruction) {
        console.log("Attempting to add instruction: " + instruction.toString());
        this._twistMsg = {twist:instruction}; 
    } 
    
    allStop = () => {
        this._twistMsg = { 
            autonomous_toggles:0,
            target_coordinate:{
                x:0,y:0
            },
            wheel_instruction: "none",
            front_arm_instruction:0,
            back_arm_instruction:0,
            front_drum_instruction:0,
            back_drum_instruction:0 
        }

        HTTP.doPost(this.apiPath, this.twistMsg);
    }

    executeRobotCommand(part, operation) {
        switch(part) {
            case Robot.FRONTARM:
                this.updateTwistMsg({front_arm_instruction:operation});
                break;
            case Robot.BACKARM:
                this.updateTwistMsg({back_arm_instruction:operation});
                break;
            case Robot.FRONTDRUM:
                this.updateTwistMsg({front_drum_instruction:operation});
                break;
            case Robot.BACKDRUM:
                this.updateTwistMsg({back_drum_instruction:operation});
                break;
            case Robot.WHEELS:
                this.updateTwistMsg({wheel_instruction:operation});
                break;
            default:
                console.log('Invalid robot part selected');
                return;
        }
        HTTP.doPost(this.apiPath, this.twistMsg);
    }
    
    // WHEEL OPERATIONS
    driveForward = () => {
        this.executeRobotCommand(Robot.WHEELS, Operation.DRIVEFORWARD)
    }

    turnLeft = () => {
        this.executeRobotCommand(Robot.WHEELS, Operation.TURNLEFT);
    }

    turnRight = () => {
        this.executeRobotCommand(Robot.WHEELS, Operation.TURNRIGHT);
    }

    driveBackward = () => {
       this.executeRobotCommand(Robot.WHEELS, Operation.DRIVEBACKWARD); 
    }

    wheelsStop = () => {
       this.executeRobotCommand(Robot.WHEELS, Operation.STOPWHEELS); 
    }

    // DRUM OPERATIONS
    frontDrumsRotateOutward = () => {
       this.executeRobotCommand(Robot.FRONTDRUM, Operation.OUTWARD); 
    }

    frontDrumsRotateInward = () => { 
       this.executeRobotCommand(Robot.FRONTDRUM, Operation.INWARD); 
    }

    frontDrumsStop = () => {
        this.executeRobotCommand(Robot.FRONTDRUM, Operation.STOP);
    }
    
    backDrumsRotateOutward = () => {
       this.executeRobotCommand(Robot.BACKDRUM, Operation.OUTWARD); 
    }
    
    backDrumsRotateInward = () => { 
       this.executeRobotCommand(Robot.BACKDRUM, Operation.INWARD); 
    }

    backDrumsStop = () => { 
        this.executeRobotCommand(Robot.BACKDRUM, Operation.STOP);
    }

    // ARM OPERATIONS
    frontArmUp = () => {
        this.executeRobotCommand(Robot.FRONTARM, Operation.UP);
    }

    frontArmDown = () => { 
        this.executeRobotCommand(Robot.FRONTARM, Operation.DOWN);
    }

    frontArmStop = () => { 
        this.executeRobotCommand(Robot.FRONTARM, Operation.STOP);
    }

    backArmUp = () => { 
        this.executeRobotCommand(Robot.BACKARM, Operation.UP);
    }

    backArmDown = () => {
        this.executeRobotCommand(Robot.BACKARM, Operation.DOWN); 
    }

    backArmStop = () => {
        this.executeRobotCommand(Robot.BACKARM, Operation.STOP); 
    }

}