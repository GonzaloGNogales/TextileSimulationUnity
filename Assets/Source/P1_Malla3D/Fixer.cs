using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Fixer : MonoBehaviour
{
    private BoxCollider _fixerCollider;
    private MassSpringCloth[] _cloths;
    private Vector3 _initialFixerPosition;
    
    // Start is called before the first frame update
    void Start()
    {
        // Get the BoxCollider component of every fixer in the scene
        _fixerCollider = GetComponent<BoxCollider>();
        _initialFixerPosition = transform.position;  // Save the initial fixer position to compute movement delta
        Debug.Log("Fixer in position " + _initialFixerPosition + " initialized!!!");
        
        // Get all the MassSpringCloths of the scene
        _cloths = FindObjectsOfType<MassSpringCloth>();
        Debug.Log(_cloths.Length + " cloths found in the scene...");
    }

    // Update is called once per frame
    void Update()
    {
        // Get the fixer position in global coordinates
        Vector3 finalFixerPosition = transform.position;

        // Check if nodes are contained inside the fixer collider bounds
        foreach (MassSpringCloth cloth in _cloths)
        {
            List<Node> clothNodes = cloth.nodes;
            foreach (Node node in clothNodes)
            {
                // Transform the node position from local coordinates to global with the fixer transform
                Vector3 nodeLocalPosition = transform.InverseTransformPoint(node.pos);  // Local position of the node with respect to the fixer
                Vector3 nodeGlobalPosition = transform.TransformPoint(nodeLocalPosition);  // Global position of the node with respect to the fixer
                if (_fixerCollider.bounds.Contains(nodeGlobalPosition))
                {            
                    if (!node.isFixed) node.isFixed = true; 
                    Vector3 movementQuantity = finalFixerPosition - _initialFixerPosition;  // Measure the fixer movement in xyz axis
                    node.pos += movementQuantity;  // Add the movement quantity measured to the fixed nodes position
                }
                else
                {
                    if (node.isFixed) node.isFixed = false;
                }
            }
        }
        
        _initialFixerPosition = finalFixerPosition; // Assign the final position of the fixer
    }
}
