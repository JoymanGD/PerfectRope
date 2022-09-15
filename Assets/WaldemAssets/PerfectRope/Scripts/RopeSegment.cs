using UnityEngine;

namespace Waldem.PerfectRope
{
    public class RopeSegment
    {
        public Vector3 posNow;
        public Vector3 posOld;
        public float mass;

        public RopeSegment(Vector3 pos, float mass)
        {
            posNow = pos;
            posOld = pos;
            this.mass = mass;
        }
    }
}