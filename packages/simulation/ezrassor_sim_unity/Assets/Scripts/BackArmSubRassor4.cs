using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using backArm = RosMessageTypes.RosMessages.StdMessageMsg;

public class BackArmSubRassor4 : MonoBehaviour
{
    float armMsg;

    public void Start()
    {
        // create subsciber
        string backString = "/ezrassor4/back_arm_instructions";
        ROSConnection.instance.Subscribe<backArm>(backString, armMove);
    }

    void armMove(backArm msg)
    {
        // grab value from msg instance
        armMsg = msg.data; 

        HingeJoint hinge = GetComponent<HingeJoint>();
        JointMotor motor = hinge.motor;

        if(armMsg == 1f) // idle
        {
            motor.targetVelocity = 15;
            hinge.motor = motor;
        }
        if(armMsg == -1f) // digging
        { 
            motor.targetVelocity = -15;
            hinge.motor = motor;
        }
    }

}
