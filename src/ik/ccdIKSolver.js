import { Vector3, Quaternion } from 'three';

function ccdIKSolver(ikChain, targetPosition, tolerance, maxNumOfIterations) {
  const { ikJoints, endEffector } = ikChain;

  const endEffectorWorldPosition = new Vector3();

  const fromToQuaternion = new Quaternion();

  let endEffectorTargetDistance = endEffector
    .worldToLocal(targetPosition.clone())
    .length();
  let numOfIterations = 0;

  while (
    endEffectorTargetDistance > tolerance &&
    numOfIterations <= maxNumOfIterations
  ) {
    for (let idx = ikJoints.length - 2; idx >= 0; idx--) {
      const ikJoint = ikJoints[idx];
      // ikJoint.updateMatrixWorld();
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
        .worldToLocal(endEffectorWorldPosition.clone())
        .normalize();

      // Get the direction from current joint to target.
      const directionToTarget = ikJoint
        .worldToLocal(targetPosition.clone())
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
        const inverseRotation = ikJoint.quaternion.clone().invert();
        const hingeAxisAfterRotation = ikJoint.axis
          .clone()
          .applyQuaternion(inverseRotation);
        fromToQuaternion.setFromUnitVectors(
          ikJoint.axis,
          hingeAxisAfterRotation
        );
        ikJoint.quaternion.multiply(fromToQuaternion);
      }

      // Apply hinge limits.

      if (ikJoint.limit) {
        const {
          quaternion,
          initialQuaternion,
          limit: { lower, upper },
          axisName,
          axisIsNegative,
        } = ikJoint;

        let jointAxisQuaternion = quaternion[axisName];
        jointAxisQuaternion = axisIsNegative
          ? -jointAxisQuaternion
          : jointAxisQuaternion;
        let rotationAngle = quaternion.angleTo(initialQuaternion);
        rotationAngle =
          jointAxisQuaternion < 0 ? -rotationAngle : rotationAngle;

        let clampedRotationAngle = rotationAngle;
        if (rotationAngle < lower) {
          clampedRotationAngle = lower;
        } else if (rotationAngle > upper) {
          clampedRotationAngle = upper;
        }

        ikJoint.quaternion.setFromAxisAngle(ikJoint.axis, clampedRotationAngle);
      }

      ikJoint.updateMatrixWorld();
    }

    endEffectorTargetDistance = endEffector
      .worldToLocal(targetPosition.clone())
      .length();
    numOfIterations++;
  }
}

export default ccdIKSolver;
