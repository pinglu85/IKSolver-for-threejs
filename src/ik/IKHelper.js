import {
  TubeGeometry,
  CatmullRomCurve3,
  Vector3,
  MeshBasicMaterial,
  Mesh,
  CylinderGeometry,
} from 'three';

const COLOR = 0x0092ff;
const LINK_SIZE = {
  tubularSegments: 64,
  radius: 0.001,
  radialSegments: 8,
};
const JOINT_SIZE = {
  radius: 0.01,
  height: 0.05,
  radialSegments: 8,
};

class IKHelper {
  constructor(ikChain, config = {}) {
    this.ikJoints = ikChain.ikJoints;
    this.config = config;
    this.linkMaterial = this._createLinkMaterial();
    this.jointMaterial = this._createJointMaterial();
  }

  visualizeIKChain() {
    const jointGeometry = this._createJointGeometry();

    let parent = this.ikJoints[0].parent;

    for (let idx = 0; idx < this.ikJoints.length; idx++) {
      const ikJoint = this.ikJoints[idx];

      const linkGeometry = this._createLinkGeometry(ikJoint.position);
      const link = new Mesh(linkGeometry, this.linkMaterial);
      parent.add(link);

      parent = ikJoint;

      if (idx === 0) continue;

      const joint = new Mesh(jointGeometry, this.jointMaterial);

      const jointAxisIdx = ikJoint.axis
        .toArray()
        .findIndex((value) => value === 1 || value === -1);
      let jointAxis = jointAxisIdx === 0 ? 'x' : jointAxisIdx === 1 ? 'y' : 'z';
      if (jointAxis === 'z') {
        joint.rotateX(Math.PI / 2);
      }

      ikJoint.add(joint);
    }
  }

  _createLinkGeometry(endPoint) {
    const startPoint = new Vector3(0, 0, 0);
    const linkPathPoints = [startPoint, endPoint];
    const linkPath = new CatmullRomCurve3(linkPathPoints);
    const radius = this.config.linkWidth / 2 || LINK_SIZE.radius;
    const radialSegments =
      this.config.linkRoundness || LINK_SIZE.radialSegments;
    return new TubeGeometry(
      linkPath,
      LINK_SIZE.tubularSegments,
      radius,
      radialSegments
    );
  }

  _createLinkMaterial() {
    const material = new MeshBasicMaterial({
      color: this.config.linkColor || COLOR,
    });
    return material;
  }

  _createJointGeometry() {
    const radiusTop = this.config.jointRadius || JOINT_SIZE.radius;
    const radiusBottom = radiusTop;
    const height = this.config.jointHeight || JOINT_SIZE.height;
    const radialSegments =
      this.config.jointRoundness || JOINT_SIZE.radialSegments;
    return new CylinderGeometry(
      radiusTop,
      radiusBottom,
      height,
      radialSegments
    );
  }

  _createJointMaterial() {
    const material = new MeshBasicMaterial({
      color: this.config.JointColor || COLOR,
    });
    return material;
  }
}

export default IKHelper;
