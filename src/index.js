import {
  Scene,
  PerspectiveCamera,
  WebGLRenderer,
  PointLight,
  Object3D,
  LoadingManager,
  Vector3,
  Group,
  Mesh,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls';
import URDFLoader from 'urdf-loader';

import { IKChain, IKHelper, IKSolver } from './ik';

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

const transformControls = new TransformControls(camera, renderer.domElement);
transformControls.addEventListener('dragging-changed', (evt) => {
  orbitControls.enabled = !evt.value;
});

const loadingManager = new LoadingManager();
const urdfLoader = new URDFLoader(loadingManager);

let robot;
const ikSolver = new IKSolver({ shouldUpdateUrdfRobot: true });

urdfLoader.load('./urdf/KUKA_LWR/urdf/kuka_lwr.URDF', (result) => {
  robot = result;
});

loadingManager.onLoad = () => {
  console.log(robot);

  robot.traverse((child) => {
    if (child instanceof Mesh) {
      child.material.transparent = true;
      child.material.opacity = 0.5;
    }
    child.castShadow = true;
  });

  const group = new Group();
  group.add(robot);

  const ikChain = new IKChain();
  ikChain.createFromUrdfRobot(robot, group);

  group.rotateX(-Math.PI / 2);
  scene.add(group);

  ikSolver.ikChain = ikChain;
  ikSolver.target = createMovingTarget(ikChain.endEffector);

  const ikHelper = new IKHelper(ikChain);
  ikHelper.visualizeIKChain();
};

function createMovingTarget(endEffector) {
  const movingTarget = new Object3D();
  const endEffectorWorldPosition = new Vector3();
  endEffector.getWorldPosition(endEffectorWorldPosition);
  movingTarget.position.copy(endEffectorWorldPosition);

  transformControls.addEventListener('objectChange', () => {
    ikSolver.solve();
  });

  transformControls.attach(movingTarget);
  scene.add(movingTarget);
  scene.add(transformControls);

  return movingTarget;
}

function render() {
  renderer.render(scene, camera);
  requestAnimationFrame(render);
}
render();
