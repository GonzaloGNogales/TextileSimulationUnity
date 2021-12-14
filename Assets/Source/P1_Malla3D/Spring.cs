using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spring : MonoBehaviour {

    public Node nodeA, nodeB;

    public float Length0;
    public float Length;

    public float stiffness;

    // Use this for initialization
    void Start () {
        UpdateLength();
        Length0 = Length;
    }
	
	// Update is called once per frame
	void Update () {
        transform.localScale = new Vector3(transform.localScale.x, Length / 2.0f, transform.localScale.z);
        transform.position = 0.5f * (nodeA.pos + nodeB.pos);

        Vector3 u = nodeA.pos - nodeB.pos;
        u.Normalize();
        transform.rotation = Quaternion.FromToRotation(Vector3.up, u);
    }

    public void UpdateLength ()
    {
        Length = (nodeA.pos - nodeB.pos).magnitude;
    }

    public void ComputeForces()
    {
        Vector3 u = nodeA.pos - nodeB.pos;
        u.Normalize();
        Vector3 force = - stiffness * (Length - Length0) * u;
        nodeA.force += force;
        nodeB.force -= force;
    }
}
