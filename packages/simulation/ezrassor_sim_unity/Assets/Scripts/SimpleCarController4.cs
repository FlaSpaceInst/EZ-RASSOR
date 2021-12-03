using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[System.Serializable]
public class AxleInfo4 {
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}
     
public class SimpleCarController4 : MonoBehaviour {
    public List<AxleInfo4> axleInfos; 
    public float maxMotorTorque;
    public float maxSteeringAngle;

    //reference to EZ-RASSOR prefab and scriptHolder script
    //public GameObject EZRASSOR;
    public GameObject ScriptHolder;
    public GameObject Rover; 
    public double wheelSpeed = 0;
    public double roverRotation = 0;
    public float speed;
    public float rotateSpeed;
    public Rigidbody rb;
    public int rbSpeed;
    public Vector3 linearMotion = new Vector3(0f,0f,0f);
    public Vector3 angularMotion = new Vector3(0f,0f,0f);

   
    

    void Start()
    {
        ScriptHolder = GameObject.Find("ScriptHolder");
        Rover = GameObject.Find("ezrassor4");
        rb = Rover.GetComponent<Rigidbody>();
    }
     
    // finds the corresponding visual wheel
    // correctly applies the transform
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0) {
            return;
        }
     
        Transform visualWheel = collider.transform.GetChild(0);
     
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
     
        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }
     
    public void FixedUpdate()
    {
        float motor = maxMotorTorque * Input.GetAxis("Vertical");
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");

        //gets wheel instructions from subscriber Scripts
        wheelSpeed = ScriptHolder.GetComponent<WheelSubRassor4>().linearInfo;
        roverRotation = ScriptHolder.GetComponent<WheelSubRassor4>().angularInfo;

        rb.angularVelocity = new Vector3(rb.angularVelocity.x,(float)roverRotation,rb.angularVelocity.z);
        Rover.transform.Translate(Vector3.forward * (float)wheelSpeed * Time.deltaTime);
        
        speed = rb.velocity.z; 
        rotateSpeed = rb.angularVelocity.y;

        foreach (AxleInfo4 axleInfo in axleInfos) {
             ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
         }


    }
}