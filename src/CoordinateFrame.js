import { Vector3, Matrix4 } from 'three';

class CoordinateFrame {
  constructor(position, lookAt, upAxis) {
    this.position = position.clone();
    this.matrix4 = new Matrix4();
    this.matrix4.setPosition(this.position);
    this.matrix4.lookAt(this.position, lookAt, upAxis);
  }

  get positionVector() {
    return this.position;
  }

  get rotationMatrix() {
    const xAxis = new Vector3();
    const yAxis = new Vector3();
    const zAxis = new Vector3();
    this.matrix4.extractBasis(xAxis, yAxis, zAxis);
    return {
      xAxis,
      yAxis,
      zAxis,
    };
  }
}

export default CoordinateFrame;
