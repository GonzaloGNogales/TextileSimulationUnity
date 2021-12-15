using UnityEngine;

public class Collideable : MonoBehaviour
{
    private MassSpringCloth[] _cloths;
    private Mesh _mesh;
    private Collider _collider;
    
    // Start is called before the first frame update
    void Start()
    {
        // Get the mesh filter of the collideable object
        _mesh = GetComponent<MeshFilter>().mesh;
        
        // Retrieve the collideable object collider component
        _collider = GetComponent<Collider>();
        
        // Get all the MassSpringCloths of the scene
        _cloths = FindObjectsOfType<MassSpringCloth>();
        Debug.Log(_cloths.Length + " collideable cloths found in the scene...");
    }

    // Update is called once per frame
    void Update()
    {
        // Add penalty force for modeling collisions
        // For achieving that, it is necessary to loop through every collision possible GameObject (Colliders)
        // Just multiply => Contact Normal: n * Delta Penetration Quantity: Vector3.Dot(Contact Normal, (Collider Collision Point - Node Position)) * Contact Stiffness: Relative to Mass?
        
        /*** P4_1Physics
        for (int w = 0; w < nWalls; ++w) {
            float delta = PVector.dot(wallNormal[w], PVector.sub(wallPoint[w], nodePos[i]));
            if (delta > 0) {
                nodeForces[i].add(ContactPenaltyForce(wallNormal[w], delta, contactStiffness));
            }
        }
        /**/

        foreach (MassSpringCloth cloth in _cloths)
        {
            foreach (Node node in cloth.nodes)
            {
                Debug.Log("Collider center => " + _collider.bounds.center);
                // Transform the node position from local coordinates to global with the collider object transform
                Vector3 nodeLocalPosition = transform.InverseTransformPoint(node.pos);  // Local position of the node with respect to the collider
                Vector3 nodeGlobalPosition = transform.TransformPoint(nodeLocalPosition);  // Global position of the node with respect to the collider
                if (_collider.bounds.Contains(nodeGlobalPosition))
                {
                    Vector3[] meshNormals = _mesh.normals;
                    Debug.Log("Collision Detected !!!!!!");
                    Vector3 collisionPoint = _collider.ClosestPointOnBounds(nodeGlobalPosition);
                    
                    // DEBUG COLLISION MESSAGES
                    Debug.Log("The collision point of the sphere is => " + collisionPoint);
                    Debug.Log("The position of the collided global node is => " + nodeGlobalPosition);
                    Debug.Log("The position of the collided local node is => " + nodeLocalPosition);
                    Debug.Log("The position of the collided pre node is => " + node.pos);
                    
                    // Vector3 collisionPenetration = collisionPoint - pos;
                    // float delta = collisionPenetration.magnitude;  // Delta Penetration Quantity
                    // collisionPenetration.Normalize();  // Contact Normal
                    // float contactStiffness = 0.5f * mass;  // Contact Stiffness
                    // node.force += contactStiffness * delta * collisionPenetration;  // Add the collision penalty force
                }
            }
        }
    }
}
