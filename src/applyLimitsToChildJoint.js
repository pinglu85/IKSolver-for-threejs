import { Vector3, MathUtils } from 'three';

import CoordinateFrame from './CoordinateFrame';

const upAxis = new Vector3(0, 1, 0);
// const constraint = MathUtils.degToRad(40);
// const jointConstraints = {
//   leftAngle: constraint,
//   rightAngle: constraint,
//   upAngle: constraint,
//   downAngle: constraint,
// };

function applyLimitsToChildJoint(
  jointPosition,
  childJointPosition,
  newChildJointPosition,
  jointConstraints
) {
  const coordinateFrame = new CoordinateFrame(
    jointPosition,
    childJointPosition,
    upAxis
  );

  // The 3D vector of the cone.
  const dirFromJointToChildJoint = childJointPosition
    .clone()
    .sub(jointPosition);

  const calcPoint = newChildJointPosition.sub(jointPosition);
  const constrainedNewPosition = constrainJoint(
    calcPoint,
    dirFromJointToChildJoint,
    coordinateFrame,
    jointConstraints
  );

  return jointPosition.clone().add(constrainedNewPosition);
}

/**
 * @param {Vector3} calcPoint The point that 'points' from the p[i + 1]'s new
 * position p[i + 1]" to the p[i]'s position. calcPoint = p[i + 1]' - p[i]'
 * @param {Vector3} dir The cone vector, the point that 'points' from p[i + 1]'s
 * position to the p[i]'s position. dir = p[i + 1]' - p[i]'
 * @param {CoordinateFrame} coordinateFrame The position of p[i]' and the rotation
 * matrix (orientation) of p[i]'.
 * @param {Object} jointConstraints The rotation constraints of the joint p[i]' described
 * by angles θ1(left), θ2(right), θ3(up), θ4(down) in radians.
 * @returns {Vector3} The re-positioned position of p[i + 1]""
 */
function constrainJoint(calcPoint, dir, coordinateFrame, jointConstraints) {
  // Get the up, down, left, right vectors relative to coordinateFrame's
  // rotation matrix.
  const { xAxis, yAxis } = coordinateFrame.rotationMatrix;
  const upAndDown = [yAxis, yAxis.clone().multiplyScalar(-1)];
  const rightAndLeft = [xAxis, xAxis.clone().multiplyScalar(-1)];

  // In order to convert a 3D problem to a 2D problem, we need a vector
  // going up or down and a vector going right or left. We pick the two
  // vectors by comparing the up against the down and the left against
  // the right to see which is closer to the calculated point.
  const compareDistanceToCalculatedPoint = (a, b) => {
    const distanceA = a.distanceTo(calcPoint);
    const distanceB = b.distanceTo(calcPoint);
    return distanceA - distanceB;
  };
  upAndDown.sort(compareDistanceToCalculatedPoint);
  rightAndLeft.sort(compareDistanceToCalculatedPoint);
  const upVector3 = upAndDown[0];
  const rightVector3 = rightAndLeft[0];

  // Find the shared height by projecting the calculated point onto the
  // cone’s center axis.
  const calcPointAndDirSharedHeight = calcPoint.dot(dir) / dir.length();
  // Get the projection vector.
  const projectionVector3 = dir
    .clone()
    .normalize()
    .multiplyScalar(calcPointAndDirSharedHeight);

  // Get the point that 'points' from the projection to the calculated point.
  // point = calcPoint - projectionVector3
  const pointOnTheProjectionPlane = calcPoint.clone().sub(projectionVector3);

  // If the calculated point and the direction (cone 3D vector) have an angle
  // difference greater than 90 degrees, we need to flip the projection 3D vector.
  if (calcPointAndDirSharedHeight < 0) {
    // `Vector3.multiplyScalar(scalar: Float)` changes the original `Vector3`
    // object.
    projectionVector3.multiplyScalar(-1);
  }

  // Get the 2D components on the projection plane.
  const xAspect = pointOnTheProjectionPlane.dot(rightVector3);
  const yAspect = pointOnTheProjectionPlane.dot(upVector3);

  // Get the cross section of the cone.
  const { leftAngle, rightAngle, upAngle, downAngle } = jointConstraints;
  const leftCrossSectionLength = -(
    projectionVector3.length() * Math.tan(leftAngle)
  );
  const rightCrossSectionLength =
    projectionVector3.length() * Math.tan(rightAngle);
  const upCrossSectionLength = projectionVector3.length() * Math.tan(upAngle);
  const downCrossSectionLength = -(
    projectionVector3.length() * Math.tan(downAngle)
  );

  // Find in which quadrant the p[i + 1]"" belongs.
  const xBound =
    xAspect >= 0 ? rightCrossSectionLength : leftCrossSectionLength;
  const yBound = yAspect >= 0 ? upCrossSectionLength : downCrossSectionLength;

  let repositionedJointPosition = calcPoint.clone();
  const calcPointIsInEllipse =
    checkPointWithinEllipse(xAspect, yAspect, xBound, yBound) <= 1;
  const calcPointIsInBounds =
    calcPointIsInEllipse && calcPointAndDirSharedHeight >= 0;
  console.log(
    'ellipse: ',
    checkPointWithinEllipse(xAspect, yAspect, xBound, yBound)
  );
  console.log('inBounds', calcPointIsInEllipse);

  if (!calcPointIsInBounds) {
    // Get the angle of the point that is outside of the ellipse.
    const calcPointAngle = Math.atan2(yAspect, xAspect);
    // Find the nearest point on that cone cross section from the `calcPoint`.
    const nearestX = xBound * Math.cos(calcPointAngle);
    const nearestY = yBound * Math.sin(calcPointAngle);
    // Convert the nearest point back to 3D.
    repositionedJointPosition = convertNearestPointTo3D(
      projectionVector3,
      rightVector3,
      upVector3,
      nearestX,
      nearestY,
      calcPoint
    );
  }

  return repositionedJointPosition;
}

function checkPointWithinEllipse(xAspect, yAspect, xBound, yBound) {
  console.log('xAspect: ', xAspect);
  console.log('xBound: ', xBound);
  console.log('yAspect: ', yAspect);
  console.log('yBound: ', yBound);
  return xAspect ** 2 / xBound ** 2 + yAspect ** 2 / yBound ** 2;
}

function convertNearestPointTo3D(
  projectionVector3,
  rightVector3,
  upVector3,
  nearestX,
  nearestY,
  calcPoint
) {
  // Multiply the respective component to the respective 3D vector.
  const mappedRightVector3 = rightVector3.clone().multiplyScalar(nearestX);
  const mappedUpVector3 = upVector3.clone().multiplyScalar(nearestY);
  // Since the point we converted to 2D was relative to the projection,
  // we need to add the projection 3D vector back to it.
  const absolutePoint = projectionVector3
    .clone()
    .addVectors(mappedRightVector3, mappedUpVector3);
  const normalizedAbsolutePoint = absolutePoint.normalize();
  // Multiply the normalized absolute point by the fixed length between the
  // two joints to retain proper distance.
  return normalizedAbsolutePoint.multiplyScalar(calcPoint.length());
}

// const test = () => {
//   const jointPos = new Vector3(9.03430271, 14.716032, -10.5470963);
//   const lookAt = new Vector3(6.65153408, 18.3193016, -12.3130484);
//   const cf = new CoordinateFrame(jointPos, lookAt, upAxis);

//   const calcPoint = new Vector3(-2.55285263, 3.86047363, -1.89200592);

//   // const myConeVec = new Vector3(6.65153408, 18.3193016, -12.3130484).sub(
//   //   jointPos
//   // );
//   // console.log('myConeVec: ', myConeVec.toArray());
//   const coneVec = new Vector3(-2.51751995, 3.85656643, -1.9465313);

//   const newPos = constrainJoint(calcPoint, coneVec, cf, jointConstraints);
//   console.log('newPos', newPos); // -2.55285263, 3.86047363, -1.89200592
// };

// test();

export default applyLimitsToChildJoint;
