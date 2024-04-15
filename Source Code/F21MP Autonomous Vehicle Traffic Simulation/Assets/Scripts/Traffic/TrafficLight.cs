using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrafficLight : MonoBehaviour
{
    public List<MeshRenderer> redRendered, greenRenderer;

    public void SetLight(bool input)
    {
        if (input)
        {
            foreach(MeshRenderer rend in greenRenderer)
            {
                rend.material.SetColor("_Color", Color.green);
            }
            foreach (MeshRenderer rend in redRendered)
            {
                rend.material.SetColor("_Color", Color.white);
            }
        }
        else 
        {
            foreach (MeshRenderer rend in greenRenderer)
            {
                rend.material.SetColor("_Color", Color.white);
            }
            foreach (MeshRenderer rend in redRendered)
            {
                rend.material.SetColor("_Color", Color.red);
            }
        }
    }
}
