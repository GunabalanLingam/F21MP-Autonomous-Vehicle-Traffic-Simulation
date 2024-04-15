using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public static CameraFollow instance;
    public Transform carAI;
    Vector3 currentPosition;
    Vector3 currentAngle;

    int theScreenWidth;
    int theScreenHeight;


    public Transform cameraObject;
    public Transform camParent;
    Vector3 _LocalRotation;

    [Header("Camera")]
    public float _CameraDistance = 10f;
    public float MouseSensitivity = 4f;
    public float ScrollSensitvity = 2f;
    public float OrbitDampening = 10f;
    public float ScrollDampening = 6f;

    public float minRotationY, maxRotationY;

    public float minZoom, maxZoom;
    private void LateUpdate()
    {
        AttachCameraWithPlayer();
    }
    private void AttachCameraWithPlayer()
    {
        if (Input.GetMouseButton(2))
        {
            if (Input.GetAxis("Mouse X") != 0 || Input.GetAxis("Mouse Y") != 0)
            {
                _LocalRotation.x += Input.GetAxis("Mouse X") * MouseSensitivity;
                _LocalRotation.y -= Input.GetAxis("Mouse Y") * MouseSensitivity;

                if (_LocalRotation.y < minRotationY)
                    _LocalRotation.y = minRotationY;
                else if (_LocalRotation.y > maxRotationY)
                    _LocalRotation.y = maxRotationY;

            }
        }

        if (Input.GetAxis("Mouse ScrollWheel") != 0f)
        {
            float ScrollAmount = Input.GetAxis("Mouse ScrollWheel") * ScrollSensitvity;

            ScrollAmount *= (this._CameraDistance * 0.3f);

            this._CameraDistance += ScrollAmount * -1f;

            this._CameraDistance = Mathf.Clamp(this._CameraDistance, minZoom, maxZoom);



        }

        Quaternion QT = Quaternion.Euler(_LocalRotation.y, _LocalRotation.x, 0);
        this.camParent.rotation = Quaternion.Lerp(this.camParent.rotation, QT, Time.deltaTime * OrbitDampening);

        if (this.cameraObject.localPosition.y != this._CameraDistance * 1f)
        {
            this.cameraObject.localPosition = new Vector3(0f, Mathf.Lerp(this.cameraObject.localPosition.y, this._CameraDistance * 1f, Time.deltaTime * ScrollDampening), 0f);
        }

        Vector3 smoothpos = Vector3.Lerp(camParent.transform.position, new Vector3(carAI.transform.position.x, camParent.transform.position.y, carAI.transform.position.z), 1f);
        camParent.transform.position = smoothpos;
    }


}
[System.Serializable]
public enum CameraState
{
    normal, bombCam, inivisibleCam, hackCam
}
