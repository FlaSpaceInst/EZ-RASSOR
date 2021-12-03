using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using backArm = RosMessageTypes.RosMessages.StdMessageMsg;

public class backArmSub : MonoBehaviour
{
   int armPos;
   float armMsg;

    public void Start()
    {
        string backString = "/ezrassor1/back_arm_instructions";
        ROSConnection.instance.Subscribe<backArm>(backString, wheelMove);

    }

    void wheelMove(backArm msg)
    {
        
        Debug.Log(msg);
        
        // parses input from subscriber
        string Stringmsg = msg.ToString();
        string value = "";
        for (int i = 0; i < Stringmsg.Length; i++)
        {

            if (System.Char.IsDigit(Stringmsg[i]))
            {
                value += Stringmsg[i];
            }
        }
        Debug.Log("LOOK HERE " + value);

        // converts string to int then changes the arm's rotation.
        armMsg = msg.data;
        HingeJoint hinge = GetComponent<HingeJoint>();
        JointMotor motor = hinge.motor;
        //armMsg = 1; 

        if(armMsg == 1f) //arm up
        {
            //Debug.Log("motor" + hinge.motor.targetVelocity);
            motor.targetVelocity = 15;
            hinge.motor = motor;
            //Debug.Log("motor" + hinge.motor.targetVelocity);

        }

        if(armMsg == -1f) //arm down
        { 
            //Debug.Log("motor" + hinge.motor.targetVelocity);
            motor.targetVelocity = -15;
            hinge.motor = motor;
            //Debug.Log("motor" + hinge.motor.targetVelocity);
        }

    }

    
}
