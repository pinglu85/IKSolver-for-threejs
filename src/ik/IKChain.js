import IKJoint from './IKJoint';
class IKChain {
  constructor() {
    this._ikJoints = [];
    this._urdfJoints = [];
    this._rootJoint = null;
    this._urdfBaseJointId = '';
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
    this.addJoint(rootJointParent, this._rootJoint);

    const urdfRobotBaseJoint = this._findUrdfBaseJoint(urdfRobot);
    this._urdfBaseJointId = urdfRobotBaseJoint.id;

    this._traverseUrdfJoints(this._rootJoint, urdfRobotBaseJoint);

    return this;
  }

  _findUrdfBaseJoint({ children }) {
    let baseJoint = null;
    for (const child of children) {
      if (!child.isURDFJoint) continue;

      const [urdfLink] = child.children;
      const hasNextUrdfJoint = urdfLink.children.some(
        (child) => child.isURDFJoint
      );
      if (hasNextUrdfJoint) {
        baseJoint = child;
        break;
      }
    }

    return baseJoint;
  }

  _traverseUrdfJoints(parentIkJoint, urdfJoint) {
    this._urdfJoints.push(urdfJoint);

    const ikJoint = new IKJoint(urdfJoint);
    this.addJoint(parentIkJoint, ikJoint);
    parentIkJoint = ikJoint;

    const [urdfLink] = urdfJoint.children;
    const { children } = urdfLink;
    const nextUrdfJoint = children.find((child) => child.isURDFJoint);
    const isEndEffector =
      ikJoint.isFixed && urdfJoint.id !== this._urdfBaseJointId;

    if (!nextUrdfJoint || isEndEffector) {
      this._endEffector = ikJoint;
      return;
    }

    this._traverseUrdfJoints(parentIkJoint, nextUrdfJoint);
  }
}

export default IKChain;
