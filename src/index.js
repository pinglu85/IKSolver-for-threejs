import {
  Scene,
  PerspectiveCamera,
  WebGLRenderer,
  DirectionalLight,
  AmbientLight,
  Object3D,
  LoadingManager,
  Vector3,
  Group,
  Mesh,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls';
import URDFLoader from 'urdf-loader';
import { XacroLoader } from 'xacro-parser';

import { IKChain, IKHelper, IKSolver } from './ik';
import { HALF_PI } from './constants/math';
import {
  KUKA_URL,
  STAUBLI_URL,
  STAUBLI_PACKAGES_URLS,
} from './constants/robotFilePaths';

const selectInput = document.querySelector('select');
selectInput.addEventListener('change', (evt) => {
  const { value } = evt.target;
  if (value === 'kuka') {
    loadURDFRobot(true);
  } else if (value === 'staubli') {
    loadURDFRobot(false);
  }
});

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

const directionalLight = new DirectionalLight(0xffffff, 1.0);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.setScalar(1024);
directionalLight.position.set(5, 30, 5);
scene.add(directionalLight);

const ambientLight = new AmbientLight(0xffffff, 0.2);
scene.add(ambientLight);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.minDistance = 0.1;
orbitControls.target.y = 1;
orbitControls.update();

const transformControls = new TransformControls(camera, renderer.domElement);
transformControls.addEventListener('dragging-changed', (evt) => {
  orbitControls.enabled = !evt.value;
});
const movingTarget = new Object3D();
transformControls.attach(movingTarget);

const loadingManager = new LoadingManager();
const urdfLoader = new URDFLoader(loadingManager);
const xacroLoader = new XacroLoader();

let robot;
let robotGroup;
const ikSolver = new IKSolver({ shouldUpdateURDFRobot: true });
ikSolver.target = movingTarget;
transformControls.addEventListener('objectChange', () => {
  ikSolver.solve();
});

let isLoading = false;

function loadURDFRobot(isKuka) {
  if (isLoading) return;

  isLoading = true;

  if (robot) {
    const robotGroupID = scene.getObjectById(robotGroup.id);
    scene.remove(robotGroupID);
  }

  if (isKuka) {
    urdfLoader.load(KUKA_URL, (result) => {
      robot = result;
      isLoading = false;
    });
  } else {
    // Handle `xacro:include filename='...'` in XACRO file.
    xacroLoader.rospackCommands = {
      find(pkg) {
        switch (pkg) {
          case 'staubli_resources':
            return STAUBLI_PACKAGES_URLS.staubli_resources;
          case 'staubli_tx2_90_support':
            return STAUBLI_PACKAGES_URLS.staubli_tx2_90_support;
          default:
            return pkg;
        }
      },
    };

    xacroLoader.load(STAUBLI_URL, (xml) => {
      urdfLoader.packages = {
        staubli_tx2_90_support: STAUBLI_PACKAGES_URLS.staubli_tx2_90_support,
      };
      robot = urdfLoader.parse(xml);
      isLoading = false;
    });
  }
}
loadURDFRobot(true);

loadingManager.onLoad = () => {
  console.log(robot);

  robot.traverse((child) => {
    if (child instanceof Mesh) {
      child.material.transparent = true;
      child.material.opacity = 0.5;
    }
    child.castShadow = true;
  });

  robotGroup = new Group();
  robotGroup.add(robot);

  const ikChain = new IKChain();
  ikChain.createFromURDFRobot(robot, robotGroup);

  robotGroup.rotateX(-HALF_PI);
  scene.add(robotGroup);

  ikSolver.ikChain = ikChain;
  setMovingTargetPosition(ikChain.endEffector);

  const ikHelper = new IKHelper(ikChain);
  ikHelper.visualizeIKChain();
};

const endEffectorWorldPosition = new Vector3();

function setMovingTargetPosition(endEffector) {
  const movingTargetInScene = scene.getObjectById(movingTarget.id);
  if (!movingTargetInScene) {
    scene.add(movingTarget);
    scene.add(transformControls);
  }

  endEffector.getWorldPosition(endEffectorWorldPosition);
  movingTarget.position.copy(endEffectorWorldPosition);
}

function render() {
  renderer.render(scene, camera);
  requestAnimationFrame(render);
}
render();
