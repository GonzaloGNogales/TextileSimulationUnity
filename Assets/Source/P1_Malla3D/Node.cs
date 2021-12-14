using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node : MonoBehaviour {

    public Vector3 pos;
    public Vector3 vel;
    public Vector3 force;

    public float mass;
    public bool isFixed;

    // Use this for initialization
    private void Awake()
    {
        this.pos = transform.position;
        this.vel = Vector3.zero;
    }

    void Start () {
	}
	
	// Update is called once per frame
	void Update () {
        transform.position = pos;
	}

    public void ComputeForces()
    {
        force += mass * transform.parent.GetComponent<MassSpringCloth>().gravity;
    }
}
