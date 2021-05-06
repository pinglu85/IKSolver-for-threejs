import {
  Scene,
  PerspectiveCamera,
  WebGLRenderer,
  Bone,
  Skeleton,
  SkeletonHelper,
  Object3D,
  Vector3,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls';

import IKSolver from './fabrikIKSolver';

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

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.minDistance = 0.1;
orbitControls.target.y = 1;
orbitControls.update();

const BONE_COUNT = 4;
const BONE_LENGTH = 0.25;

const bones = [];
let parentBone;
for (let idx = 0; idx < BONE_COUNT; idx++) {
  const bone = new Bone();
  bone.position.y = idx === 0 ? 0 : 0.25;
  // bone.matrixWorldNeedsUpdate = true;
  if (parentBone) parentBone.add(bone);
  parentBone = bone;
  bones.push(bone);
}

// Debug
// const pos = new Vector3();
// for (const bone of bones) {
//   bone.getWorldPosition(pos);
//   console.log('bone pos: ', pos);
// }

const skeleton = new Skeleton(bones);
const skeletonHelper = new SkeletonHelper(skeleton.bones[0]);
skeletonHelper.material.linewidth = 2;
scene.add(skeletonHelper);
scene.add(skeleton.bones[0]);

const movingTarget = new Object3D();
movingTarget.position.y = (BONE_COUNT - 1) * BONE_LENGTH;
const transformControls = new TransformControls(camera, renderer.domElement);
transformControls.addEventListener('dragging-changed', (evt) => {
  orbitControls.enabled = !evt.value;
});

transformControls.addEventListener('objectChange', () => {
  const newBonePositions = IKSolver(bones, movingTarget);
  // console.log(newBonePositions);
  for (let idx = 0; idx < bones.length; idx++) {
    const bone = bones[idx];
    const boneGlobalPosition = newBonePositions[idx];
    const parentBone = bones[idx - 1];
    const parentPosition = new Vector3();
    if (parentBone) {
      parentBone.getWorldPosition(parentPosition);
    }
    const boneLocalPosition = boneGlobalPosition.sub(parentPosition);
    bone.position.copy(boneLocalPosition);
    // console.log('bone pos', bone.position);
  }
});

transformControls.attach(movingTarget);
scene.add(movingTarget);
scene.add(transformControls);

function render() {
  // bones[1].position.z = 0.25;
  renderer.render(scene, camera);
  requestAnimationFrame(render);
}
render();
