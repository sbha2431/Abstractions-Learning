#ifndef INVERSEKINEMATICS2_Q048XIUC
#define INVERSEKINEMATICS2_Q048XIUC

/**
 * @file InverseKinematic.h
 * @author Alexander H�rtl
 * @author jeff
 * Edited by Sam Barrett
 */

#include "math/Vector2.h"
#include "math/Range.h"
#include "math/Pose3D.h"
#include "math/Common.h"
#include "common/RobotDimensions.h"
#include "common/RobotInfo.h"
#include <cassert>


class InverseKinematics {
public:
  /**
  * The method calculates the joint angles for the legs of the robot from a Pose3D for each leg
  * @param positionLeft The desired position (translation + rotation) of the left foots ankle point
  * @param positionRight The desired position (translation + rotation) of the right foots ankle point
  * @param jointData The instance of JointData where the resulting joints are written into
  * @param robotDimensions The Robot Dimensions needed for calculation
  * @param ratio The ratio of
  * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target)
  */
  static bool calcLegJoints(const Pose3D& positionLeft, const Pose3D& positionRight, Joints jointAngles, const RobotDimensions& robotDimensions, float ratio = 0.5f);

//private:
  /**
  * The method calculates the joint angles of one leg of the robot from a Pose3D
  * @param position The desired position (translation + rotation) of the foots ankle point
  * @param jointData The instance of JointData where the resulting joints are written into
  * @param left Determines if the left or right leg is calculated
  * @param robotDimensions The Robot Dimensions needed for calculation
  * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target)
  */
  static bool calcLegJoints(const Pose3D& position, Joints jointAngles, bool left, const RobotDimensions& robotDimensions);

  /**
  * The method calculates the joint angles of one leg of the Nao from a Pose3D with a fixed first joint
  * This is necessary because the Nao has mechanically connected hip joints, hence not every
  * combination of foot positions can be reached and has to be recalculated with equal joint0 for both legs
  * the rotation of the foot around the z-axis through the ankle-point is left open as "failure"
  * @param position The desired position (translation + rotation) of the foots ankle point
  * @param jointData The instance of JointData where the resulting joints are written into
  * @param joint0 Fixed value for joint0 of the respective leg
  * @param left Determines if the left or right leg is calculated
  * @param robotDimensions The Robot Dimensions needed for calculation
  * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target)
  */
  static bool calcLegJoints(const Pose3D& position, Joints jointAngles, float joint0, bool left, const RobotDimensions& robotDimensions);

public:
  static bool calcArmJoints(const Pose3D& left, const Pose3D& right, Joints jointAngles, const RobotDimensions& theRobotDimensions);

  static bool calcArmJoints(Vector3<float> target, Vector3<float> targetDir, int side, Joints jointAngles, const RobotDimensions& theRobotDimensions);

  static bool calcElbowPosition(Vector3<float> &target, const Vector3<float> &targetDir, int side, Vector3<float> &elbow, const RobotDimensions& theRobotDimensions);

  static void calcJointsForElbowPos(const Vector3<float> &elbow, const Vector3<float> &target, Joints jointAngles, int offset, const RobotDimensions& theRobotDimensions);

  /**
  * Solves the inverse kinematics for the arms of the Nao with arbitrary elbow yaw.
  * @param position Position of the arm in cartesian space relative to the robot origin.
  * @param elbowYaw The fixed angle of the elbow yaw joint.
  * @param jointData The instance of JointData where the resulting joints are written into.
  * @param left Determines whether the left or right arm is computed.
  * @param robotDimensions The robot dimensions needed for the calculation.
  */
  static void calcArmJoints(const Vector3<float>& position, const float elbowYaw, Joints jointAngles, bool left, const RobotDimensions& robotDimensions);
};

#endif /* end of include guard: INVERSEKINEMATICS2_Q048XIUC */
