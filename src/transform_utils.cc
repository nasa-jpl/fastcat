#include "fastcat/transform_utils.h"

quat QuatConj(quat a)
{
  quat c = {a.u, -a.x, -a.y, -a.z};
  return c;
}

// Multiply quatern by vector and return quatern
quat QuatMulVec3(quat q, vec3 v)
{
  quat u = {0 - q.x * v.x - q.y * v.y - q.z * v.z,
            q.u * v.x + 0 + q.y * v.z - q.z * v.y,
            q.u * v.y - q.x * v.z + 0 + q.z * v.x,
            q.u * v.z + q.x * v.y - q.y * v.x + 0};
  return u;
}

// Multiply vector by quatern and return vector
vec3 Vec3MulQuat(quat v, quat q)
{
  vec3 u = {-v.u * q.x + v.x * q.u - v.y * q.z + v.z * q.y,
            -v.u * q.y + v.x * q.z + v.y * q.u - v.z * q.x,
            -v.u * q.z - v.x * q.y + v.y * q.x + v.z * q.u};
  return u;
}

vec3 Vec3Sub(vec3 a, vec3 b)
{
  vec3 c = {a.x - b.x, a.y - b.y, a.z - b.z};
  return c;
}

vec3 Vec3Cross(vec3 a, vec3 b)
{
  vec3 c;
  c.x = a.y * b.z - a.z * b.y;
  c.y = -a.x * b.z + a.z * b.x;
  c.z = a.x * b.y - a.y * b.x;
  return c;
}

vec3 Vec3Rotate(quat q, vec3 v) { return Vec3MulQuat(QuatMulVec3(q, v), q); }

quat QuatFromRpy(double roll, double pitch, double yaw)
{
  quat   a;
  double phi, the, psi;
  phi = roll / 2;
  the = pitch / 2;
  psi = yaw / 2;
  a.u = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
  a.x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
  a.y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
  a.z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
  return a;
}

wrench WrenchTransform(wrench w, transform tf)
{
  wrench b;
  b.forces  = Vec3Rotate(QuatConj(tf.rotation), w.forces);
  b.torques = Vec3Sub(
      Vec3Rotate(QuatConj(tf.rotation), w.torques),
      Vec3Rotate(QuatConj(tf.rotation), Vec3Cross(tf.position, w.forces)));
  return b;
}