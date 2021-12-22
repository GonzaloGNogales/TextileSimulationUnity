using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Collideable : MonoBehaviour {
    private MassSpringCloth[] _cloths;
    private Mesh _mesh;
    private Vector3[] _collideableNormals;
    private MatrixXD _n;
    private float _height;
    
    // Start is called before the first frame update
    void Start() {
        // Get the mesh filter of the collideable object and allocate it's normals and vertices
        _mesh = GetComponent<MeshFilter>().mesh;
        _collideableNormals = _mesh.normals;
        _height = transform.position.y + 0.04f;
        Debug.Log("Ground is at height => " + _height);

        MatrixXD n = new DenseMatrixXD(3, 1);
        n[0, 0] = _collideableNormals[0][0];
        n[1, 0] = _collideableNormals[0][1];
        n[2, 0] = _collideableNormals[0][2];
        
        // Transpose computation
        MatrixXD nt = n.Transpose();
        
        // Normal matrix multiplication
        _n = n.Multiply(nt);
        
        // Get all the MassSpringCloths of the scene
        _cloths = FindObjectsOfType<MassSpringCloth>();
        Debug.Log(_cloths.Length + " collideable cloths found in the scene...");
    }

    // Update is called once per frame
    void FixedUpdate() {
        // Add implicit penalty force for modeling collisions
        // For achieving that, it is necessary to loop through every collision possible GameObject (Colliders)
        // Implicit Method
        foreach (MassSpringCloth cloth in _cloths)
        {
            float h = cloth.timeStep;
            List<Spring> springs = cloth.springs;
            bool collision = false;
            foreach (Node node in cloth.nodes)
            {
                float delta = _height - node.pos.y;
                if (delta > 0)
                {
                    // Contact stiffness initialization when collision
                    float contactStiffness = 1000000.0f;
                    
                    // Penalty force differential with respect to node position
                    MatrixXD penaltyDiff = - contactStiffness * _n;
                    MatrixXD I = DenseMatrixXD.CreateIdentity(3);
                    
                    // Penalty force computation
                    Vector3 penetration = node.pos -  new Vector3(0.0f, _height, 0.0f);
                    VectorXD penetrationDense = new DenseVectorXD(3);
                    penetrationDense[0] = penetration[0];
                    penetrationDense[1] = penetration[1];
                    penetrationDense[2] = penetration[2];
                    VectorXD penaltyForceDense = penaltyDiff * penetrationDense;
                    Vector3 penaltyForce = new Vector3((float) penaltyForceDense[0], (float) penaltyForceDense[1], (float) penaltyForceDense[2]);
                    
                    // node.force is a prediction of the future forces calculated by approximation
                    // node.force = F(0) + F(h) (collisions implicit penalty forces)
                    node.force += penaltyForce;
                    
                    // Collision implicit operand calculation
                    MatrixXD C = I - h * h / node.mass * penaltyDiff;
                    
                    Vector3 antF = node.vel + h / node.mass * node.force;
                    VectorXD v0 = new DenseVectorXD(3);
                    v0[0] = antF[0];
                    v0[1] = antF[1];
                    v0[2] = antF[2];
                    VectorXD newVelDense = C.Inverse() * v0;
                    Vector3 newVel = new Vector3((float) newVelDense[0], (float) newVelDense[1], (float) newVelDense[2]);
                    
                    // New velocity assignment
                    node.vel = newVel;
                    node.pos += h * node.vel;
                    
                    // Set this node in implicit collision integration mode to stop symplectic positions integration method
                    node.inCollision = true;
                    collision = true;
                }
                else
                {
                    // Reset collision state for this node if it is not colliding any more
                    node.inCollision = false;
                }
            }
            if (collision)
            {
                // Update the length of each spring after this step
                foreach (Spring spring in springs)
                {
                    spring.UpdateLength();
                }
            }
        }
    }
}
