using UnityEngine;

public class Node : MonoBehaviour {
    public Vector3 pos;
    public Vector3 vel;
    public Vector3 force;
    public float mass;
    public bool isFixed;

    // Use this for initialization
    private void Awake() {
        pos = transform.position;
        vel = Vector3.zero;
    }

    void Start () {
	}
	
	// Update is called once per frame
	void Update () {
        transform.position = pos;
	}

    public void ComputeForces() {
        // Added damping force in this node proportional to it's mass (40% of the mass) 
        // This damping factor is used to scale the actual node velocity in order to subtract it
        // from the => Total Node Force (Fa) = 2nd Newton Law: m*g - Damping: damping * velocity
        force += mass * transform.parent.GetComponent<MassSpringCloth>().gravity +                   // Gravity
                 mass * transform.parent.GetComponent<MassSpringCloth>().wind                        // Wind force
                 - 0.4f * mass * vel;															     // Node Damping
    }
}
