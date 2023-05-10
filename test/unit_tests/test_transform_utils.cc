#include <gtest/gtest.h>

#include "fastcat/transform_utils.h"

namespace
{
class TransformUtilsTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    mass_wrench.forces  = {0, 0, -1};
    mass_wrench.torques = {1, 0, 0};

    tf_wrench.forces  = {1, 0, 0};
    tf_wrench.torques = {0, 0, 2};

    test_tf.position = {0, 1, 0};
    test_tf.rotation = {cos(M_PI / 4), 0, sin(M_PI / 4), 0};
  }

  wrench    mass_wrench;
  wrench    tf_wrench;
  transform test_tf;
};

TEST_F(TransformUtilsTest, RpyToQuat)
{
  quat roll_only = QuatFromRpy(M_PI / 2, 0, 0);
  EXPECT_NEAR(roll_only.u, cos(M_PI / 4), 1e-15);
  EXPECT_NEAR(roll_only.x, sin(M_PI / 4), 1e-15);
  EXPECT_NEAR(roll_only.y, 0, 1e-15);
  EXPECT_NEAR(roll_only.z, 0, 1e-15);

  quat pitch_only = QuatFromRpy(0, M_PI / 2, 0);
  EXPECT_NEAR(pitch_only.u, cos(M_PI / 4), 1e-15);
  EXPECT_NEAR(pitch_only.x, 0, 1e-15);
  EXPECT_NEAR(pitch_only.y, sin(M_PI / 4), 1e-15);
  EXPECT_NEAR(pitch_only.z, 0, 1e-15);

  quat yaw_only = QuatFromRpy(0, 0, M_PI / 2);
  EXPECT_NEAR(yaw_only.u, cos(M_PI / 4), 1e-15);
  EXPECT_NEAR(yaw_only.x, 0, 1e-15);
  EXPECT_NEAR(yaw_only.y, 0, 1e-15);
  EXPECT_NEAR(yaw_only.z, sin(M_PI / 4), 1e-15);
}

TEST_F(TransformUtilsTest, TransformWrench)
{
  wrench result = WrenchTransform(mass_wrench, test_tf);
  EXPECT_NEAR(result.forces.x, tf_wrench.forces.x, 1e-15);
  EXPECT_NEAR(result.forces.y, tf_wrench.forces.y, 1e-15);
  EXPECT_NEAR(result.forces.z, tf_wrench.forces.z, 1e-15);
  EXPECT_NEAR(result.torques.x, tf_wrench.torques.x, 1e-15);
  EXPECT_NEAR(result.torques.y, tf_wrench.torques.y, 1e-15);
  EXPECT_NEAR(result.torques.z, tf_wrench.torques.z, 1e-15);
}
}  // namespace
