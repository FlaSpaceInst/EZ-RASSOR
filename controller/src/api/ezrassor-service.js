import {Robot, Operation} from '../enumerations/robot-commands';
import HTTP from './web-commands';

export default class EZRASSOR { 

    constructor(host, route = '') {
        this.host = host;
        this.route = route;
        this.setCoordinate(0, 0);
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

    get coordinate() {
        return this._coordinate;
    }

    setCoordinate(x, y) {
        this._coordinate = {
            x: x,
            y: y
        }
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
        this._twistMsg = {twist:instruction}; 
    } 

    updateAutonomyTwistMsg(instruction) {
        if(instruction == Operation.DRIVE || instruction == Operation.FULLAUTONOMY) {
            this._twistMsg = {
                twist: instruction,
                target_coordinate: this.coordinate
            }
            return;
        }

        this._twistMsg = {twist:instruction};
    }
   
    // Stop all robot operations
    allStop = () => {
        this._twistMsg = { 
            autonomous_toggles:0,
            target_coordinate:this.coordinate,
            wheel_instruction: "none",
            front_arm_instruction:0,
            back_arm_instruction:0,
            front_drum_instruction:0,
            back_drum_instruction:0 
        }

        HTTP.doPost(this.apiPath, this.twistMsg);
    }

    // Execute the corresponding robot command from the enumeration items passed in
    executeRobotCommand(part, operation) {
        // Needed when a stop override needs to occur
        if (part == Robot.ALL && operation == Operation.STOP) {
            this.allStop();
            return;
        }

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
            case Robot.AUTONOMY:
                this.updateAutonomyTwistMsg(operation);
            default:
                console.log('Invalid robot part selected');
                return;
        }

        HTTP.doPost(this.apiPath, this.twistMsg);
    } 
}