using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MultipleCamera : MonoBehaviour
{
    public List<Camera> cameras = new List<Camera>();

    private List<Camera> displayedCameras = new List<Camera>();
    private int currentIndex = 0;

    private void Start()
    {
        // Deactivate all cameras at the start
        //DeactivateAllCameras();
    }

    private void Update()
    {
        // Check for key press to switch cameras
        if (Input.GetKeyDown(KeyCode.Space))
        {
            SwitchCameraRandomly();
        }
    }

    private void SwitchCameraRandomly()
    {
        // Activate a random camera that hasn't been displayed yet
        if (displayedCameras.Count < cameras.Count)
        {
            Camera randomCamera;
            do
            {
                randomCamera = cameras[Random.Range(0, cameras.Count)];
            } while (displayedCameras.Contains(randomCamera));

            // Display the selected camera
            ActivateCamera(randomCamera);
            displayedCameras.Add(randomCamera);
        }
        else
        {
            // All cameras have been displayed, allow repetitions
            currentIndex = Random.Range(0, cameras.Count);
            ActivateCamera(cameras[currentIndex]);
        }
    }

    private void ActivateCamera(Camera camera)
    {
        // Deactivate all cameras first
        DeactivateAllCameras();

        // Activate the selected camera
        camera.gameObject.SetActive(true);
    }

    private void DeactivateAllCameras()
    {
        foreach (Camera cam in cameras)
        {
            cam.gameObject.SetActive(false);
        }
    }
}
