using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections;
using System.Collections.Generic;
using backDrum = RosMessageTypes.RosMessages.StdMessageMsg;

public class BackDrumSubRassor2 : MonoBehaviour
{
    float drumMsg;
    public GameObject SH;
    public GameObject ez;

    void Start()
    {
        // set refernces
        SH = GameObject.Find("ScriptHolder");
        ez = GameObject.Find("ezrassor2");
        // create subscriber instance
        ROSConnection.instance.Subscribe<backDrum>("/ezrassor2/back_drum_instructions", drumMove);
    }

    void drumMove(backDrum msg)
    {
        // grab value from msg instance
        drumMsg = msg.data;

        // references the hinge joint motor and sets its rotation.
        JointMotor jm = GetComponent<HingeJoint>().motor;
        jm.targetVelocity = drumMsg *350;
        GetComponent<HingeJoint>().motor = jm;

        // Dig dump functionality
        if (drumMsg > 0) // drum DIG
        {
            Debug.Log("Digging");
            SH.GetComponent<RoverDigDumpScript>().Dig(ez);
        }
        else if (drumMsg < 0) // drum DUMP
        {
            Debug.Log("Dumpping");
            SH.GetComponent<RoverDigDumpScript>().Dump(ez);
        }
        else // drum IDLE
        {
            Debug.Log("Drum idle.");
        }
    }
    
}
