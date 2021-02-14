#ifndef FASTCAT_TRANSFORM_UTILS_H_
#define FASTCAT_TRANSFORM_UTILS_H_

// Include c then c++ libraries
#include <cmath>

struct vec3 {
  double x, y, z;
};

struct quat {
  double u, x, y, z;
};

struct wrench {
  vec3 forces;
  vec3 torques;
};

struct transform {
  vec3 position;
  quat rotation;
};

quat QuatConj(quat a);

quat   QuatMulVec3(quat q,
                   vec3 v);  // Multiply quatern by vector and return quatern
vec3   Vec3MulQuat(quat v,
                   quat q);  // Multiply vector by quatern and return vector
vec3   Vec3Sub(vec3 a, vec3 b);
vec3   Vec3Cross(vec3 a, vec3 b);
vec3   Vec3Rotate(quat q, vec3 v);
quat   QuatFromRpy(double roll, double pitch, double yaw);
wrench WrenchTransform(wrench w, transform tf);

#endif