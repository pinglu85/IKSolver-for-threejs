import { Vector3 } from 'three';

// The distances between each joint d[i] = |p[i + 1] - p[i]| For i = 0,..., n - 1.
const jointDistances = [];
let rootEndEffectorDistance;

function IKSolver(joints, target) {
  // console.log(joints);
  // console.log(target);
  const jointPositions = Array(joints.length);

  for (let idx = 0; idx < joints.length; idx++) {
    const joint = joints[idx];
    const jointPosition = new Vector3();
    joint.getWorldPosition(jointPosition);
    jointPositions[idx] = jointPosition;

    if (jointDistances.length === joints.length - 1) continue;

    if (idx < joints.length - 1) {
      const childJoint = joints[idx + 1];
      const childJointPosition = new Vector3();
      childJoint.getWorldPosition(childJointPosition);
      const childJointJointDistance = childJointPosition.distanceTo(
        jointPosition
      );
      jointDistances[idx] = childJointJointDistance;
    }
  }

  if (rootEndEffectorDistance === undefined) {
    rootEndEffectorDistance = jointDistances.reduce(
      (sum, distance) => sum + distance
    );
  }
  // Debug
  // for (let idx = 0; idx < jointPositions.length; idx++) {
  //   console.log('joint pos: ', jointPositions[idx]);
  //   if (idx < jointDistances.length) {
  //     console.log('joint distance: ', jointDistances[idx]);
  //   }
  // }

  return fabrikIKSolver(jointPositions, target.position);
}

// Minimum distance between the end effector and the target, if the
// target is reachable. The algorithm will stop solving if the distance
// between the end effector and the target is smaller than the tolerance,
// meaning the end effector reaches the target or gets sufficiently close.
// If tolerance is zero, the algorithm will iterate until `MAX_ITERATIONS`.
const TOLERANCE = 0.1;
const MAX_ITERATIONS = 10;

/**
 * JavaScript implementation of FABRIK algorithm: http://www.andreasaristidou.com/FABRIK.html
 * @param {Vector3[]} jointPositions The joint positions p[i] for i = 0,. . .,n.
 * @param {Vector3} targetPosition The target position
 * @returns {Vector3[]} The new joint positions pi for i = 0, ..., n.
 */

function fabrikIKSolver(jointPositions, targetPosition) {
  // The distance between root and target.
  // dist = |p[0] - t|
  const rootTargetDistance = jointPositions[0].distanceTo(targetPosition);

  // Check whether the target is within reach.
  // The target is unreachable.
  if (rootTargetDistance > rootEndEffectorDistance) {
    // For i = 0, ..., n - 1 do
    for (let idx = 0; idx < jointDistances.length; idx++) {
      // Find the distance r[i] between the target t and the joint position p[i].
      const jointPosition = new Vector3().copy(jointPositions[idx]);

      // r[i] = |t - p[i]|
      const targetJointDistance = targetPosition.distanceTo(jointPosition);
      const lambda = jointDistances[idx] / targetJointDistance;

      // Find the new joint positions p[i].
      const copiedTargetPosition = new Vector3().copy(targetPosition);

      // p[i + 1] = (1 - lambda[i]) * p[i] + lambda[i] * t
      jointPositions[idx + 1] = jointPosition
        .multiplyScalar(1 - lambda)
        .add(copiedTargetPosition.multiplyScalar(lambda));
    }
  }
  // The target is reachable...
  else {
    //...thus, store the initial base position, which is position of the joint p[0].
    const initialBasePosition = new Vector3().copy(jointPositions[0]);

    let endEffectorPosition = jointPositions[jointPositions.length - 1];
    // dif[A] = \p[n] - t|
    let endEffectorTargetDistance = endEffectorPosition.distanceTo(
      targetPosition
    );

    let numOfIterations = 0;

    // Check whether the distance between the end effector p[n]
    // and the target t is greater than a tolerance and maximum number
    // of iterations is reached.
    while (
      endEffectorTargetDistance > TOLERANCE &&
      numOfIterations <= MAX_ITERATIONS
    ) {
      // *** STAGE 1: FORWARD REACHING ***
      // 1) Move the end effector p[n] to the target.
      // 2) Find the position of the joint p[n - 1], which lies on the line
      //    that passes through p[n] and p[n - 1] and has distance d[n - 1]
      //    from p[n].
      // 3) continue the algorithm for the rest of the joints.

      // Move the end effector p[n] to target t.
      jointPositions[jointPositions.length - 1] = targetPosition;

      // For i = n - 1, ..., 0  do
      for (let idx = jointDistances.length - 1; idx >= 0; idx--) {
        // Find the distance r[i] between the new joint position
        // p[i + 1] and the joint p[i].
        // r[i] = |p[i + 1] - p[i]|
        const childJointJointDistance = jointPositions[idx + 1].distanceTo(
          jointPositions[idx]
        );
        const lambda = jointDistances[idx] / childJointJointDistance;

        const copiedJointPosition = new Vector3().copy(jointPositions[idx]);
        const copiedChildJointPosition = new Vector3().copy(
          jointPositions[idx + 1]
        );
        // Find the new joint positions p[i].
        jointPositions[idx] = copiedChildJointPosition
          .multiplyScalar(1 - lambda)
          .add(copiedJointPosition.multiplyScalar(lambda));
      }

      // *** STAGE 2: BACKWARD REACHING ***
      // 1) Move the root joint p[0] to its initial position.
      // 2) Find the position of joint p[1], which lies on the line that passes
      //    through p[0] and p[1] and has distance d[0] from p[0].
      // 3) continue the algorithm for the rest of the joints.

      // The two stages algorithm is repeated until the position of the end
      // effector reaches the target or gets sufficiently close.

      // Set the root p[0] to its initial position, the initial base position.
      jointPositions[0] = initialBasePosition;

      // For i = 1, ..., n - 1 do
      for (let idx = 0; idx < jointDistances.length; idx++) {
        // Find the distance r[i] between the new joint position p[i]
        // and the joint p[i + 1].
        // r[i] = |p[i + 1] - p[i]|
        const childJointJointDistance = jointPositions[idx + 1].distanceTo(
          jointPositions[idx]
        );
        const lambda = jointDistances[idx] / childJointJointDistance;

        const copiedJointPosition = new Vector3().copy(jointPositions[idx]);
        const copiedChildJointPosition = new Vector3().copy(
          jointPositions[idx + 1]
        );
        // Find the new joint positions p[i].
        jointPositions[idx + 1] = copiedJointPosition
          .multiplyScalar(1 - lambda)
          .add(copiedChildJointPosition.multiplyScalar(lambda));
      }

      // Update the distance between the end effector p[n] and the target.
      endEffectorPosition = jointPositions[jointPositions.length - 1];
      // dif[A] = \p[n] - t|
      endEffectorTargetDistance = endEffectorPosition.distanceTo(
        targetPosition
      );

      numOfIterations++;
    }
  }

  return jointPositions;
}

export default IKSolver;
