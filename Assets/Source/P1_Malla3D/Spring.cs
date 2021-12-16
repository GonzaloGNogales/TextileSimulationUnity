using UnityEngine;

public class Spring : MonoBehaviour {
    public Node nodeA, nodeB;
    public float length0;
    public float length;
    public float stiffness;

    // Use this for initialization
    void Start () {
        UpdateLength();
        length0 = length;
    }
	
	// Update is called once per frame
	void Update () {
        transform.localScale = new Vector3(transform.localScale.x, length / 2.0f, transform.localScale.z);
        transform.position = 0.5f * (nodeA.pos + nodeB.pos);
        
        Vector3 u = nodeA.pos - nodeB.pos;
        u.Normalize();
        transform.rotation = Quaternion.FromToRotation(Vector3.up, u);
    }

    public void UpdateLength () {
        length = (nodeA.pos - nodeB.pos).magnitude;
    }

    public void SubstepStartLengtUpdate()
    {
        UpdateLength();
        length0 = length;
    }

    public void ComputeForces() {
        // Calculate spring elastic force using Hooke's Law for node A
        Vector3 u = nodeA.pos - nodeB.pos;
        u.Normalize();
        Vector3 force = - stiffness * (length - length0) * u;
        
        // Add damping force for the spring to slow down correctly (it is a subtraction as the damping factor is negative)
        force += - 0.01f * stiffness * Vector3.Dot(u,nodeA.vel - nodeB.vel) * u;
        nodeA.force += force;
        nodeB.force -= force;
    }
}
