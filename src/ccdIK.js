import {
  Scene,
  PerspectiveCamera,
  WebGLRenderer,
  PointLight,
  Group,
  Object3D,
  Vector3,
  MathUtils,
  CylinderGeometry,
  Mesh,
  MeshBasicMaterial,
  Line,
  BufferGeometry,
  LineBasicMaterial,
  Quaternion,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls';

const TOLERANCE = 0.01;
const MAX_NUM_OF_ITERATIONS = 10;

const scene = new Scene();
const camera = new PerspectiveCamera(
  45,
  window.innerWidth / window.innerHeight,
  0.01,
  1000
);
camera.position.set(5, 5, 5);

const renderer = new WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const lights = [];
lights[0] = new PointLight(0xffffff, 1, 0);
lights[1] = new PointLight(0xffffff, 1, 0);
lights[2] = new PointLight(0xffffff, 1, 0);

lights[0].position.set(0, 200, 0);
lights[1].position.set(100, 200, 100);
lights[2].position.set(-100, -200, -100);

scene.add(lights[0]);
scene.add(lights[1]);
scene.add(lights[2]);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.minDistance = 0.1;
orbitControls.target.y = 1;
orbitControls.update();

const radiusTop = 0.01;
const radiusBottom = 0.01;
const height = 0.05;
const radialSegments = 8;
const cylinderGeometry = new CylinderGeometry(
  radiusTop,
  radiusBottom,
  height,
  radialSegments
);

const lineMaterial = new LineBasicMaterial({
  color: 0xffff00,
});

const rootJoint = {
  localPosition: [0, 0, 0],
  axis: [0, 1, 0],
  limits: [0, 0],
};

const firstJoint = {
  localPosition: [0, 0.11, 0],
  axis: [0, 1, 0],
  limits: [-170, 170],
};

const secondJoint = {
  localPosition: [0, 0.2, 0],
  axis: [1, 0, 0],
  limits: [-90, 90],
};

const thirdJoint = {
  localPosition: [0, 0.2, 0],
  axis: [0, 1, 0],
  limits: [-170, 170],
};

const fourthJoint = {
  localPosition: [0, 0.2, 0],
  axis: [1, 0, 0],
  limits: [-120, 120],
};

const fifthJoint = {
  localPosition: [0, 0.2, 0],
  axis: [0, 1, 0],
  limits: [-170, 170],
};

const sixthJoint = {
  localPosition: [0, 0.19, 0],
  axis: [1, 0, 0],
  limits: [-10, 10],
};

const seventhJoint = {
  localPosition: [0, 0.078, 0],
  axis: [0, 1, 0],
  limits: [-170, 170],
};

const joints = [
  rootJoint,
  firstJoint,
  secondJoint,
  thirdJoint,
  fourthJoint,
  fifthJoint,
  sixthJoint,
  seventhJoint,
];

const movingTarget = new Object3D();
movingTarget.position.y = joints.reduce(
  (sumOfY, joint) => sumOfY + joint.localPosition[1],
  0
);
const transformControls = new TransformControls(camera, renderer.domElement);
transformControls.addEventListener('dragging-changed', (evt) => {
  orbitControls.enabled = !evt.value;
});
transformControls.attach(movingTarget);
scene.add(movingTarget);
scene.add(transformControls);

const IKChain = [];
let parentJoint = scene;

for (let idx = 0; idx < joints.length; idx++) {
  const joint = addJoint(parentJoint, joints[idx], idx);
  parentJoint = joint;
}

function addJoint(parent, { localPosition, axis, limits }, idx) {
  const joint = new Group();
  parent.add(joint);
  joint.position.set(...localPosition);
  joint.axis = new Vector3(...axis);
  joint.limits = {
    min: MathUtils.degToRad(limits[0]),
    max: MathUtils.degToRad(limits[1]),
  };
  IKChain.push(joint);

  if (idx > 0) {
    for (let idx = 0; idx < axis.length; idx++) {
      if (axis[idx] === 1) {
        const cylinderMaterial = new MeshBasicMaterial({ color: 0xffff00 });
        cylinderMaterial.transparent = true;
        cylinderMaterial.opacity = 0.5;
        const cylinder = new Mesh(cylinderGeometry, cylinderMaterial);

        if (idx === 0) {
          cylinder.material.color.setHex(0xff4d4d);
          cylinder.rotateZ(MathUtils.degToRad(90));
        } else if (idx === 2) {
          cylinder.material.color.setHex(0x4dff4d);
          cylinder.rotateX(MathUtils.degToRad(90));
        }
        joint.add(cylinder);
      }
    }

    const lineGeometry = new BufferGeometry();
    const lineStartPoint = new Vector3(0, 0, 0);
    const lineEndPoint = joint.position;
    lineGeometry.setFromPoints([lineStartPoint, lineEndPoint]);
    const line = new Line(lineGeometry, lineMaterial);
    parent.add(line);
  }

  return joint;
}

console.log(IKChain);

function ccdIKSolver(targetPosition) {
  const endEffector = IKChain[IKChain.length - 1];
  const endEffectorWorldPosition = new Vector3();

  const fromToQuaternion = new Quaternion();

  let endEffectorTargetDistance = endEffector
    .worldToLocal(targetPosition.clone())
    .length();
  let numOfIterations = 0;

  while (
    endEffectorTargetDistance > TOLERANCE &&
    numOfIterations <= MAX_NUM_OF_ITERATIONS
  ) {
    for (let idx = IKChain.length - 2; idx >= 0; idx--) {
      endEffector.getWorldPosition(endEffectorWorldPosition);

      const joint = IKChain[idx];

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
      const directionToEndEffector = joint
        .worldToLocal(endEffectorWorldPosition.clone())
        .normalize();

      // Get the direction from current joint to target.
      const directionToTarget = joint
        .worldToLocal(targetPosition.clone())
        .normalize();

      fromToQuaternion.setFromUnitVectors(
        directionToEndEffector,
        directionToTarget
      );
      joint.quaternion.multiply(fromToQuaternion);

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
      const inverseRotation = joint.quaternion.clone().invert();
      const hingeAxisAfterRotation = joint.axis
        .clone()
        .applyQuaternion(inverseRotation);
      fromToQuaternion.setFromUnitVectors(joint.axis, hingeAxisAfterRotation);
      joint.quaternion.multiply(fromToQuaternion);

      // Apply hinge limits.

      const clampedRotation = joint.rotation.toVector3();
      const hingeAxis = joint.axis.toArray();
      for (let idx = 0; idx < hingeAxis.length; idx++) {
        if (hingeAxis[idx] === 1) {
          const rotationValue = clampedRotation.getComponent(idx);
          const clampedValue = MathUtils.clamp(
            rotationValue,
            joint.limits.min,
            joint.limits.max
          );
          clampedRotation.setComponent(idx, clampedValue);
        }
      }
      joint.rotation.setFromVector3(clampedRotation);

      joint.updateMatrixWorld();
    }

    endEffectorTargetDistance = endEffector
      .worldToLocal(targetPosition.clone())
      .length();
    numOfIterations++;
  }
}

transformControls.addEventListener('objectChange', () => {
  ccdIKSolver(movingTarget.position);
});

function render() {
  renderer.render(scene, camera);
  requestAnimationFrame(render);
}
render();
