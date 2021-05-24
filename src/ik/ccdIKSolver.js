import { Vector3, Quaternion } from 'three';

const endEffectorWorldPosition = new Vector3();
const endEffectorWorldToLocalPosition = new Vector3();
const targetWorldToLocalPosition = new Vector3();
const fromToQuaternion = new Quaternion();
const inverseQuaternion = new Quaternion();
const jointAxisAfterRotation = new Vector3();

function ccdIKSolver(ikChain, targetPosition, tolerance, maxNumOfIterations) {
  const { ikJoints, endEffector } = ikChain;

  let endEffectorTargetDistance = endEffector
    .worldToLocal(targetWorldToLocalPosition.copy(targetPosition))
    .length();
  let numOfIterations = 0;

  while (
    endEffectorTargetDistance > tolerance &&
    numOfIterations <= maxNumOfIterations
  ) {
    for (let idx = ikJoints.length - 2; idx >= 0; idx--) {
      const ikJoint = ikJoints[idx];
      if (ikJoint.isFixed) {
        ikJoint.updateMatrixWorld();
        continue;
      }

      endEffector.getWorldPosition(endEffectorWorldPosition);

      // Rotate the joint from end effector to goal, so that the end-effector
      // can meet the target.
      // https://sites.google.com/site/auraliusproject/ccd-algorithm

      // Get the direction from current joint to end effector:
      // direction = endEffector.position - joint.position.
      // Since the position of end effector we get is a world position,
      // we can either get current joint's world position to compute the
      // direction, or transform end effector's world position
      // to current joint's local space and then compute the direction.
      // Since later we need to use the direction to compute the quaternion
      // that will apply to current joint and it is easier to apply local
      // quaternion, we choose the latter method.
      const directionToEndEffector = ikJoint
        .worldToLocal(
          endEffectorWorldToLocalPosition.copy(endEffectorWorldPosition)
        )
        .normalize();

      // Get the direction from current joint to target.
      const directionToTarget = ikJoint
        .worldToLocal(targetWorldToLocalPosition.copy(targetPosition))
        .normalize();

      fromToQuaternion.setFromUnitVectors(
        directionToEndEffector,
        directionToTarget
      );
      ikJoint.quaternion.multiply(fromToQuaternion);

      // Constrain the joint rotation to its hinge axis.
      // Let `axis[i]` denote the current hinge axis. Since the current joint
      // has been rotated in the previous step, we apply same rotation to `axis[i]`,
      // which gives us the new axis `axis[i]'`. To constrain the rotation to the
      // specified axis, we can rotate back the current joint by the rotation defined
      // between `axis[i]` and `axis[i]'`.

      // We can compute the amount that we need to rotate back the current joint
      // without inverting the rotation that has been applied to that joint, but
      // that will end up with awkward rotation of the joint before the final rotation
      // is found.
      if (ikJoint.isHinge || ikJoint.isRootJoint) {
        inverseQuaternion.copy(ikJoint.quaternion).invert();
        jointAxisAfterRotation
          .copy(ikJoint.axis)
          .applyQuaternion(inverseQuaternion);

        fromToQuaternion.setFromUnitVectors(
          ikJoint.axis,
          jointAxisAfterRotation
        );
        ikJoint.quaternion.multiply(fromToQuaternion);
      }

      // Apply hinge limits.

      if (ikJoint.limit) {
        const ikJointRotationAngle = getIKJointRotationAngle(ikJoint);
        const clampedIKJointRotationAngle = clampIKJointRotationAngle(
          ikJointRotationAngle,
          ikJoint.limit
        );

        ikJoint.quaternion.setFromAxisAngle(
          ikJoint.axis,
          clampedIKJointRotationAngle
        );
      }

      ikJoint.updateMatrixWorld();
    }

    endEffectorTargetDistance = endEffector
      .worldToLocal(targetWorldToLocalPosition.copy(targetPosition))
      .length();
    numOfIterations++;
  }
}

function getIKJointRotationAngle(ikJoint) {
  const { axisName, axis } = ikJoint;
  // Given an axis [a_x, a_y, a_z] and angle theta,
  // Quaternion = [a_x * sin(theta / 2), a_y * sin(theta / 2), a_z * sin(theta / 2), cos(theta / 2)]
  return Math.asin(ikJoint.quaternion[axisName] / axis[axisName]) * 2;
}

function clampIKJointRotationAngle(ikJointRotationAngle, limit) {
  const { lower, upper } = limit;
  if (ikJointRotationAngle < lower) {
    return lower;
  }

  if (ikJointRotationAngle > limit.upper) {
    return upper;
  }

  return ikJointRotationAngle;
}

export default ccdIKSolver;
