import IKJoint from './IKJoint';

class IKChain {
  constructor() {
    this._ikJoints = [];
    this._urdfJoints = [];
    this._rootJoint = null;
    this._endEffector = null;
  }

  addJoint(parent, ikJoint) {
    parent.add(ikJoint);
    this._ikJoints.push(ikJoint);
  }

  get ikJoints() {
    return this._ikJoints;
  }

  get rootJoint() {
    return this._rootJoint;
  }

  get endEffector() {
    return this._endEffector;
  }

  get urdfJoints() {
    return this._urdfJoints;
  }

  createFromUrdfRobot(urdfRobot, rootJointParent) {
    this._rootJoint = new IKJoint();
    let parent = rootJointParent;
    this.addJoint(parent, this._rootJoint);
    parent = this._rootJoint;

    const robotJointsKeys = Object.keys(urdfRobot.joints);

    for (let idx = 0; idx < robotJointsKeys.length; idx++) {
      const key = robotJointsKeys[idx];
      const urdfJoint = urdfRobot.joints[key];
      this._urdfJoints.push(urdfJoint);

      const ikJoint = new IKJoint(urdfJoint);
      this.addJoint(parent, ikJoint);

      if (idx === robotJointsKeys.length - 1) {
        this._endEffector = ikJoint;
      }

      parent = ikJoint;
    }

    return this;
  }
}

export default IKChain;
