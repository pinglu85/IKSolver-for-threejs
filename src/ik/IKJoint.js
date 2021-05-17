import { Group, Vector3 } from 'three';

class IKJoint extends Group {
  constructor(urdfJoint = null) {
    super();
    this.axis = new Vector3(0, 1, 0);
    this.limit = {};
    this.isHinge = false;
    this.isRootJoint = false;
    this.isIkJoint = true;

    if (urdfJoint) {
      this.position.copy(urdfJoint.position);
      this.rotation.copy(urdfJoint.rotation);
      this.isHinge = urdfJoint.jointType === 'revolute';
      this.axis.copy(urdfJoint.axis);
      this.limit = { ...urdfJoint.limit };
    } else {
      this.position.set(0, 0, 0);
      this.isRootJoint = true;
      this.limit = { lower: 0, upper: 0 };
    }
  }
}

export default IKJoint;
