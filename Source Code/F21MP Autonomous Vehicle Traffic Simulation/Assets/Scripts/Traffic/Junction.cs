using System;
using UnityEngine;

public class Junction : Road
{

    [Header("Junction")]
    public Phase[] phases;
    public float phaseInterval = 5;

    float phaseTimer;
    bool isPhaseEnded;
    private int currentPhaseIndex;

    public override void Start()
    {
        base.Start();
        InitializePhases();
    }


    private void InitializePhases()
    {
        if (phases.Length > 0)
            phases[0].Enable();
    }

    private void Update()
    {
        HandleTimedPhase();
    }

    private void HandleTimedPhase()
    {
        phaseTimer += Time.deltaTime;
        if (!isPhaseEnded && phaseTimer > phaseInterval * 0.5f)
            EndPhase();

        if (phaseTimer > phaseInterval)
            ChangePhase();
    }
    private void EndPhase()
    {
        isPhaseEnded = true;
        phases[currentPhaseIndex].End();
    }

    public void ChangePhase()
    {
        phaseTimer = 0;
        isPhaseEnded = false;
        IncrementPhaseIndex();
        phases[currentPhaseIndex].Enable();
    }
    private void IncrementPhaseIndex()
    {
        currentPhaseIndex = (currentPhaseIndex + 1) % phases.Length;
    }
    public void TryChangePhase()
    {
        ChangePhase();
    }


    private Mesh cube;
    public Mesh Cube
    {
        get
        {
            if (cube == null)
            {
                cube = CreateCubeMesh();
            }
            return cube;
        }
    }

    private Mesh CreateCubeMesh()
    {
        GameObject tempCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Mesh mesh = tempCube.GetComponent<MeshFilter>().sharedMesh;
        DestroyImmediate(tempCube);
        return mesh;
    }


    private void OnDrawGizmos()
    {
        if (TrafficSystem.Instance.GizmoOnSceneView)
        {
            Phase phase = phases[currentPhaseIndex];
            foreach (WaitZone zone in phase.positiveZones)
            {
                Gizmos.color = zone.canPass ? Color.green : Color.red;
                DrawAreaGizmo(zone.transform);
            }
            Gizmos.color = Color.red;
            foreach (WaitZone zone in phase.negativeZones)
                DrawAreaGizmo(zone.transform);
        }
    }

    private void DrawAreaGizmo(Transform t)
    {
        Matrix4x4 rotationMatrix1 = Matrix4x4.TRS(t.position - new Vector3(0, t.localScale.y * 0.5f, 0), t.rotation, Vector3.Scale(t.lossyScale, new Vector3(1f, 0.1f, 1f)));
        Gizmos.matrix = rotationMatrix1;
        Gizmos.DrawWireMesh(cube, Vector3.zero, Quaternion.identity);
    }

    [Serializable]
    public class Phase
    {
        public WaitZone[] positiveZones;
        public WaitZone[] negativeZones;
        public TrafficLight[] positiveLights;
        public TrafficLight[] negativeLights;

        public void Enable()
        {
            foreach (WaitZone zone in positiveZones)
                zone.canPass = true;
            foreach (TrafficLight light in positiveLights)
                light.SetLight(true);
            foreach (WaitZone zone in negativeZones)
                zone.canPass = false;
            foreach (TrafficLight light in negativeLights)
                light.SetLight(false);
        }

        public void End()
        {
            foreach (WaitZone zone in positiveZones)
                zone.canPass = false;
        }
    }
}

