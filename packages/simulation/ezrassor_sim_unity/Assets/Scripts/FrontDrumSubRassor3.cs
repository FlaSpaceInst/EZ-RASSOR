using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using frontDrum = RosMessageTypes.RosMessages.StdMessageMsg;

public class FrontDrumSubRassor3 : MonoBehaviour
{
    float drumMsg;
    public GameObject SH;
    public GameObject ez;

    void Start()
    {
        // set references
        SH = GameObject.Find("ScriptHolder");
        ez = GameObject.Find("ezrassor1");
        // create subscriber instance
        ROSConnection.instance.Subscribe<frontDrum>("/ezrassor3/front_drum_instructions", drumMove);
    }

    void drumMove(frontDrum msg)
    {
        // grab data from msg
        drumMsg = msg.data;

        // references the hinge joint motor and sets its rotation.
        JointMotor jm = GetComponent<HingeJoint>().motor;
        // set velocity of drum to rotate
        jm.targetVelocity = drumMsg * 350;
        GetComponent<HingeJoint>().motor = jm;
    }

}
