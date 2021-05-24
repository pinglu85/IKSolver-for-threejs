import ccdIKSolver from './ccdIKSolver';

class IKSolver {
  constructor(config = {}) {
    this._ikChain = null;
    this._target = null;

    this.isHybrid = config.isHybrid || false;
    this.tolerance = config.tolerance || 0.01;
    this.maxNumOfIterations = config.maxNumOfIterations || 10;
    this.shouldUpdateURDFRobot = config.shouldUpdateURDFRobot || false;
  }

  get ikChain() {
    return this._ikChain;
  }

  set ikChain(newIkChain) {
    this._ikChain = newIkChain;
  }

  get target() {
    return this._target;
  }

  set target(newTarget) {
    this._target = newTarget;
  }

  setConfig(config) {
    for (const key in config) {
      this[key] = config[key];
    }
  }

  solve() {
    if (!this.ikChain || !this.target) return;

    ccdIKSolver(
      this.ikChain,
      this.target.position,
      this.tolerance,
      this.maxNumOfIterations
    );

    if (this.shouldUpdateURDFRobot) {
      this._updateURDFRobot();
    }
  }

  _updateURDFRobot() {
    const { ikJoints, urdfJoints } = this.ikChain;
    for (let idx = 0; idx < urdfJoints.length; idx++) {
      const ikJoint = ikJoints[idx + 1];
      const urdfJoint = urdfJoints[idx];

      urdfJoint.rotation.copy(ikJoint.rotation);
    }
  }
}

export default IKSolver;
