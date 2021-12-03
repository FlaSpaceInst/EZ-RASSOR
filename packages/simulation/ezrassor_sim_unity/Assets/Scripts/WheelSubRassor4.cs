using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosColor = RosMessageTypes.RosMessages.GeometryMessageMsg;
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class WheelSubRassor4 : MonoBehaviour
{
    public double linearInfo;
    public double angularInfo;

    public GameObject Gameobject;

    void Start()
    {
        // set subscriber instance
        ROSConnection.instance.Subscribe<RosColor>("/ezrassor4/wheel_instructions", wheelMove);
    }

    void wheelMove(RosColor msg)
    {
        // grab linear.x and angular.z from msg
        linearInfo = msg.linear.x;
        angularInfo = msg.angular.z;
    }

}
