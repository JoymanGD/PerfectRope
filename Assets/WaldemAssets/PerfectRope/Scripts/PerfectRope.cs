using System.Collections.Generic;
using UnityEngine;
using DG.Tweening;
using External.MyBox;
using UnityEditor;
using System;

namespace Waldem.PerfectRope
{
    [RequireComponent(typeof(LineRenderer))]
    public class PerfectRope : MonoBehaviour
    {
    #pragma warning disable 0649
        [Header("Rope")]
        [SerializeField]
        private bool initOnAwake = true;
        [SerializeField]
        private bool simulateOnUpdate = true;
        [SerializeField]
        private bool fixZ = false;
        [SerializeField]
        private bool createBySegmentsCount = true;
        [SerializeField]
        [ConditionalField("createBySegmentsCount")]
        private int segmentsCount = 10;
        [Range(1, 100)]
        [SerializeField]
        private float minimumSegmentsDistance = 1;
        [Range(1, 10000)]
        [SerializeField]
        private float maximumSegmentsDistance = 300;
        [SerializeField]
        private Transform startPoint;
        [SerializeField]
        private Transform endPoint;
        [SerializeField]
        private Material material;
        [SerializeField]
        private float lineWidth = 0.04f;
        [SerializeField]
        private bool fixFirst = true;
        [SerializeField]
        private bool fixLast = false;
        [SerializeField]
        private bool controlPointsDistance = false;

        [Header("Physics")]
        [SerializeField]
        private bool dontSimulateWhileScaling = false;
        [SerializeField]
        private Vector3 gravityForce = new Vector2(0, -.12f);
        [Range(0, 10)]
        [SerializeField]
        private float inputForceRatio = 2f;
        [SerializeField]
        private float segmentMass = 1;
        [SerializeField]
        private float lastSegmentMass = 10;
        [SerializeField]
        private float connectionTime = 10;
        [SerializeField]
        private float ropeScalingSpeed = 15f;
        [SerializeField]
        private float endPointRotationLerp = .3f;
        
        [Space]
        [Header("Debug")]
        [SerializeField]
        private bool drawGizmos = true;
        [SerializeField]
        private float segmentGizmoRadius = 0.4f;
    #pragma warning restore 0649

        private const float CALCULATED_SEGMENTS_DISTANCE_FACTOR = 1000f;
        private const float DISTANCE_CONTROL_ERROR = 0.2f;
        private float calculatedMinimumSegmentsDistance => minimumSegmentsDistance / CALCULATED_SEGMENTS_DISTANCE_FACTOR;
        private float calculatedMaximumSegmentsDistance => maximumSegmentsDistance / CALCULATED_SEGMENTS_DISTANCE_FACTOR;
        private LineRenderer lineRenderer;
        private List<RopeSegment> ropeSegments = new List<RopeSegment>();
        private Transform currentSelectedPoint;
        private bool scaling;
        private ScalingType currentScalingType = ScalingType.None;
        private bool isInited = false;
        private Transform targetTransform;
        private bool isConnectedToTransform = false;
        private float minRopeLength = 0;
        private float maxRopeLength;

        public Vector3 FocusPoint { get; private set; }
        public float PointsDistance { get; private set; }
        public Transform StartPoint => startPoint;
        public Transform EndPoint => endPoint;
        public float CalculatedMaximumRopeDistance => (ropeSegments.Count - 1) * calculatedMaximumSegmentsDistance;

        private void Awake()
        {
            if(initOnAwake)
            {
                Init();
            }
        }

        private void Update()
        {
            if(simulateOnUpdate)
            {
                DoUpdate();
            }
        }

        private void FixedUpdate()
        {
            if(simulateOnUpdate)
            {
                DoFixedUpdate();
            }
        }

        public void Init(int segmentsCount = 0)
        {
        #if UNITY_EDITOR
            Selection.selectionChanged += DefineCurrentSelectedPoint;
        #endif

            currentSelectedPoint = startPoint;

            lineRenderer = GetComponent<LineRenderer>();
            if(material)
            {
                lineRenderer.material = material;
            }

            CalculateFocusPoint();
            CalculatePointsDistance();

            if(segmentsCount < 3)
            {
                CalculateAndCreateRope();
            }
            else
            {
                CreateRope(segmentsCount);
            }

            isInited = true;
        }

        private void CalculateAndCreateRope()
        {
            int currentSegmentsCount = segmentsCount;

            if(!createBySegmentsCount)
            {
                var distance = Vector3.Distance(startPoint.position, endPoint.position);
                currentSegmentsCount = (int)(distance/calculatedMaximumSegmentsDistance);
            }

            CreateRope(currentSegmentsCount);
        }

        public void Disconnect()
        {
            startPoint.DOKill();

            isConnectedToTransform = false;
            fixFirst = false;

            targetTransform = null;
        }

        public void ConnectToPosition(Vector3 position)
        {
            fixFirst = true;

            startPoint.DOMove(position, connectionTime);
        }

        public void ConnectToTransform(Transform targetTransform, Action callback)
        {
            fixFirst = true;

            SetTargetTransform(targetTransform);

            var moveTween = startPoint.DOMove(targetTransform.position, connectionTime);
            moveTween.onComplete += ConnectedToTransformHandler;

            if(callback != null)
            {
                moveTween.onComplete += callback.Invoke;
            }
        }

        private void ConnectedToTransformHandler()
        {
            isConnectedToTransform = true;
        }

        private void SetTargetTransform(Transform targetTransform)
        {
            this.targetTransform = targetTransform;
        }

        public void CreateRope(int segmentsCount)
        {
            ropeSegments.Clear();

            Vector3 newSegmentPosition = startPoint.position;

            for (int i = 0; i < segmentsCount; i++)
            {
                float newSegmentMass = segmentMass;

                if(i == segmentsCount - 1)
                {
                    newSegmentMass = lastSegmentMass;
                }

                var newSegment = new RopeSegment(newSegmentPosition, newSegmentMass);
                ropeSegments.Add(newSegment);
                newSegmentPosition.y -= calculatedMaximumSegmentsDistance;
            }
        }

        public void DoUpdate()
        {
            if(isInited)
            {
                DrawRope();
                
                if(controlPointsDistance)
                {
                    ControlPointsDistance();
                }

                if(fixFirst && fixLast)
                {
                    if(!controlPointsDistance)
                    {
                        ClampPointsDistance();
                    }
                }
                
                CalculateFocusPoint();
                CalculatePointsDistance();
            }
        }

        private void CalculateFocusPoint()
        {
            FocusPoint = (endPoint.position + startPoint.position) / 2;
        }

        private void CalculatePointsDistance()
        {
            PointsDistance = Vector3.Distance(startPoint.position, endPoint.position);
        }

        private void ClampPointsDistance()
        {
            if(currentSelectedPoint)
            {
                var otherPoint = currentSelectedPoint == endPoint ? startPoint : endPoint;
                var direction = otherPoint.position - currentSelectedPoint.position;
                var distance = direction.magnitude;
                var maxDistance = CalculatedMaximumRopeDistance;

                if(distance > maxDistance)
                {
                    otherPoint.position = currentSelectedPoint.position + direction.normalized * maxDistance;
                }
            }
        }

        private void ControlPointsDistance()
        {
            var distance = Vector3.Distance(startPoint.position, endPoint.position);
            var maxDistance = CalculatedMaximumRopeDistance;

            if(distance - maxDistance > DISTANCE_CONTROL_ERROR)
            {
                AddPenultPoint();
            }
            else if(maxDistance - distance > DISTANCE_CONTROL_ERROR)
            {
                int deletedPoints = 0;
                while(maxDistance - distance > DISTANCE_CONTROL_ERROR && ropeSegments.Count > 2)
                {
                    ropeSegments.Remove(GetLastSegment());
                    deletedPoints++;
                    distance = Vector3.Distance(startPoint.position, endPoint.position);
                    maxDistance = CalculatedMaximumRopeDistance;
                }
                var pointsLeft = ropeSegments.Count - deletedPoints;
                Debug.Log($"Points deleted: {deletedPoints}; Points left: {pointsLeft}");
            }
        }

        private void DefineCurrentSelectedPoint()
        {
        #if UNITY_EDITOR
            currentSelectedPoint = null;
            
            var currentSelectedTransform = Selection.activeTransform;

            if(currentSelectedTransform)
            {
                if(currentSelectedTransform == startPoint || currentSelectedTransform == endPoint)
                {
                    currentSelectedPoint = currentSelectedTransform;
                    Debug.Log($"{currentSelectedTransform.name} was selected");
                }
            }
        #endif
        }

        public void DoFixedUpdate()
        {
            if(isInited)
            {
                ManageScaling();
                Simulate();
                RotateLastPoint();
                ConnectSegments();
            }
        }

        public void StartShrinking(float minLengthSegments = 0)
        {
            minRopeLength = minLengthSegments * calculatedMaximumSegmentsDistance;
            scaling = true;
            currentScalingType = ScalingType.Shrink;
        }

        public void StartExpanding(float maxLengthSegments = 0)
        {
            maxRopeLength = maxLengthSegments * calculatedMaximumSegmentsDistance;
            scaling = true;
            currentScalingType = ScalingType.Expand;
        }

        public void StopScaling()
        {
            scaling = false;
            currentScalingType = ScalingType.None;
        }

        private void ConnectSegments()
        {
            for (int i = 0; i < 50; i++)
            {
                ApplyPointsConstraints();
                ApplyConstraint();
            }
        }

        private void Simulate()
        {
            int scalingModificator = scaling ? 0 : 1;
            // SIMULATION
            for (int i = 0; i < ropeSegments.Count; i++)
            {
                RopeSegment currentSegment = ropeSegments[i];
                Vector3 velocity = currentSegment.posNow - currentSegment.posOld;
                currentSegment.posOld = currentSegment.posNow;
                currentSegment.posNow += (velocity + gravityForce * currentSegment.mass * Time.fixedDeltaTime) * scalingModificator;
            }
        }

        private void RotateLastPoint()
        {
            var direction = GetLasPenultDirection();
            direction.Normalize();

            var angle = Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg;
            var newRotation = Quaternion.AngleAxis(angle, Vector3.forward);
            var lastPointTransform = endPoint;
            lastPointTransform.rotation = Quaternion.Lerp(lastPointTransform.rotation, newRotation, endPointRotationLerp);
        }

        private void ResetAllSegments()
        {
            foreach (var item in ropeSegments)
            {
                item.posOld = item.posNow;
            }
        }

    #if UNITY_EDITOR
        private void OnDrawGizmos() {
            if(drawGizmos)
            {
                foreach (var item in ropeSegments)
                {
                    Gizmos.DrawSphere(item.posNow, segmentGizmoRadius);
                }
            }
        }
    #endif

        public void ApplyForceToLastSegment(Vector3 force)
        {
            if(!fixLast)
            {
                var lastSegment = GetLastSegment();
                lastSegment.posNow += force * Time.deltaTime * inputForceRatio;
            }
        }

        private void ApplyPointsConstraints()
        {
            //Constrant to First Point
            RopeSegment firstSegment = GetFirstSegment();

            if(fixFirst)
            {
                if(isConnectedToTransform)
                {
                    startPoint.position = targetTransform.position;
                }

                firstSegment.posNow = startPoint.position;
            }
            else
            {
                startPoint.position = firstSegment.posNow;
            }

            //Constrant to Last Point
            RopeSegment endSegment = GetLastSegment();

            if(fixLast)
            {
                endSegment.posNow = endPoint.position;
            }
            else
            {
                endPoint.position = endSegment.posNow;
            }
        }

        private void ApplyConstraint()
        {
            for (int i = 0; i < ropeSegments.Count - 1; i++)
            {
                bool penult = i == ropeSegments.Count - 2;
                RopeSegment firstSeg = ropeSegments[i];
                RopeSegment secondSeg = ropeSegments[i + 1];

                float dist = (firstSeg.posNow - secondSeg.posNow).magnitude;
                float error = Mathf.Abs(dist - calculatedMaximumSegmentsDistance);
                Vector3 changeDir = Vector3.zero;

                if (dist > calculatedMaximumSegmentsDistance)
                {
                    changeDir = (firstSeg.posNow - secondSeg.posNow).normalized;
                }

                Vector3 changeAmount = changeDir * error;

                firstSeg.posNow -= changeAmount * 0.5f;
                secondSeg.posNow += changeAmount * 0.5f;
                
                if(fixZ)
                {
                    firstSeg.posNow.z = 0;
                    secondSeg.posNow.z = 0;
                }
            }
        }

        private void DrawRope()
        {
            lineRenderer.startWidth = lineWidth;
            lineRenderer.endWidth = lineWidth;

            Vector3[] ropePositions = new Vector3[ropeSegments.Count];
            for (int i = 0; i < ropeSegments.Count; i++)
            {
                ropePositions[i] = ropeSegments[i].posNow;
            }

            lineRenderer.positionCount = ropePositions.Length;
            lineRenderer.SetPositions(ropePositions);
        }

        private void ManageScaling()
        {
            if(scaling && currentScalingType != ScalingType.None)
            {
                var lastPoint = GetLastSegment();
                var penultPoint = GetPenultSegment();
                var newPos = lastPoint.posNow;
                switch (currentScalingType)
                {
                    case ScalingType.Expand:
                        var expandVector = GetAverageRopeVector();
                        newPos = lastPoint.posNow + (expandVector.normalized * ropeScalingSpeed/2 * Time.fixedDeltaTime);

                        if(CalculatedMaximumRopeDistance >= maxRopeLength)
                        {
                            scaling = false;
                            currentScalingType = ScalingType.None;
                        }

                        break;
                    case ScalingType.Shrink:
                        Vector3 shrinkEndPos = penultPoint.posNow;
                        if(ropeSegments.Count < 3)
                        {
                            shrinkEndPos = penultPoint.posNow + (lastPoint.posNow - penultPoint.posNow).normalized * calculatedMinimumSegmentsDistance;
                        }
                        newPos = Vector3.MoveTowards(lastPoint.posNow, shrinkEndPos, ropeScalingSpeed * Time.fixedDeltaTime);

                        if(CalculatedMaximumRopeDistance <= minRopeLength)
                        {
                            scaling = false;
                            currentScalingType = ScalingType.None;
                        }

                        break;
                }

                lastPoint.posNow = newPos;

                ControlRopeLength();
            }
        }

        private Vector3 GetAverageRopeVector()
        {
            var sumVector = Vector3.zero;
            int vectorsCount = 0;
            for (int i = 0; i < ropeSegments.Count - 1; i++)
            {
                var firstSegment = ropeSegments[i];
                var secondSegment = ropeSegments[i+1];
                sumVector += (secondSegment.posNow - firstSegment.posNow).normalized;
                vectorsCount++;
            }

            return sumVector / vectorsCount;
        }

        private void ControlRopeLength()
        {
            var distance = GetLastPenultDistance();

            if(distance > calculatedMaximumSegmentsDistance)
            {
                while (distance > calculatedMaximumSegmentsDistance) //while loop is needed for more accurate check
                {            
                    AddPenultPoint();
                    distance = GetLastPenultDistance();
                }
            }
            else if(distance <= calculatedMinimumSegmentsDistance)
            {
                while (distance <= calculatedMinimumSegmentsDistance && ropeSegments.Count > 2) //while loop is needed for more accurate check
                {            
                    ropeSegments.Remove(GetLastSegment());
                    distance = GetLastPenultDistance();
                }
            }

            if(dontSimulateWhileScaling)
            {
                ResetAllSegments();
            }
        }

        private void AddPenultPoint()
        {
            var lastPoint = GetLastSegment();
            var penultPoint = GetPenultSegment();
            //calculate newPosition so new point wouldn't remove instantly
            var newPos = penultPoint.posNow + (lastPoint.posNow - penultPoint.posNow).normalized * calculatedMaximumSegmentsDistance;
            var newPoint = new RopeSegment(newPos, segmentMass);
            ropeSegments.Remove(lastPoint);
            ropeSegments.Add(newPoint);
            ropeSegments.Add(lastPoint);
        }

        private float GetLastPenultDistance()
        {
            var direction = GetLasPenultDirection();

            return direction.magnitude;
        }

        private Vector3 GetLasPenultDirection()
        {
            var lastPoint = GetLastSegment();
            var penultPoint = GetPenultSegment();

            return lastPoint.posNow - penultPoint.posNow;
        }

        private RopeSegment GetFirstSegment()
        {
            return ropeSegments[0];
        }

        private RopeSegment GetLastSegment()
        {
            return ropeSegments[ropeSegments.Count - 1];
        }
        
        private RopeSegment GetPenultSegment()
        {
            return ropeSegments[ropeSegments.Count - 2];
        }

        public enum ScalingType
        {
            Expand, Shrink, None
        }
    }
}