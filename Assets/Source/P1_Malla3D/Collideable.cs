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
    private MatrixXD _N;
    private float _height;
    
    // Start is called before the first frame update
    void Start() {
        // Get the mesh filter of the collideable object and allocate it's normals and vertices
        _mesh = GetComponent<MeshFilter>().mesh;
        _collideableNormals = _mesh.normals;
        _height = transform.position.y;
        Debug.Log("Ground is at height => " + _height);

        MatrixXD N = new DenseMatrixXD(3, 1);
        N[0, 0] = _collideableNormals[0][0];
        N[1, 0] = _collideableNormals[0][1];
        N[2, 0] = _collideableNormals[0][2];
        
        // Transpose computation
        MatrixXD NT = N.Transpose();
        
        // Normal matrix multiplication
        _N = N.Multiply(NT);
        
        // Get all the MassSpringCloths of the scene
        _cloths = FindObjectsOfType<MassSpringCloth>();
        Debug.Log(_cloths.Length + " collideable cloths found in the scene...");
    }

    // Update is called once per frame
    void Update() {
        // Add implicit penalty force for modeling collisions
        // For achieving that, it is necessary to loop through every collision possible GameObject (Colliders)
        // Implicit Method
        foreach (MassSpringCloth cloth in _cloths)
        {
            foreach (Node node in cloth.nodes)
            {
                float delta = _height - node.pos.y;
                if (delta > 0)
                {
                    float contactStiffness = 1000.0f;
                    MatrixXD penaltyDiff = - contactStiffness * _N;
                    MatrixXD I = DenseMatrixXD.CreateIdentity(3);
                    MatrixXD C = I - 0.0001f / node.mass * penaltyDiff;
                    VectorXD v0 = new DenseVectorXD(3);
                    v0[0] = node.vel[0];
                    v0[1] = node.vel[1];
                    v0[2] = node.vel[2];
                    VectorXD penaltyForce = C.Inverse() * v0;
                    Vector3 force = new Vector3((float) penaltyForce[0], (float) penaltyForce[1], (float) penaltyForce[2]);
                    Debug.Log("Computed collision force => " + penaltyForce);
                    //node.vel -= force;
                    node.force += force;
                }
            }
        }
    }
}
