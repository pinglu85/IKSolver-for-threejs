import { Group, Vector3 } from 'three';

import AXIS_NAMES from '../constants/axisNames';

class IKJoint extends Group {
  constructor(urdfJoint = null) {
    super();
    this.position.set(0, 0, 0);
    this.axis = new Vector3(0, 1, 0);
    this.isRootJoint = true;
    this.isHinge = false;
    this.isFixed = false;
    this.isIkJoint = true;
    this.limit = { lower: 0, upper: 0 };

    if (urdfJoint) {
      this.position.copy(urdfJoint.position);
      this.rotation.copy(urdfJoint.rotation);
      this.isRootJoint = false;
      this.isHinge = urdfJoint.jointType === 'revolute';
      this.isFixed = urdfJoint.jointType === 'fixed';
      this.axis.copy(urdfJoint.axis);
      this.limit = {
        ...urdfJoint.limit,
      };
    }
  }

  get axisArray() {
    return this.axis.toArray();
  }

  get axisIdx() {
    return this.axisArray.findIndex((value) => value !== 0);
  }

  get axisName() {
    return this.axisIdx === 0
      ? AXIS_NAMES.X
      : this.axisIdx === 1
      ? AXIS_NAMES.Y
      : AXIS_NAMES.Z;
  }

  get axisIsNegative() {
    return this.axisArray[this.axisIdx] < 0;
  }
}

export default IKJoint;
