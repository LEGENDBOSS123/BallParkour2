import Vector3 from "./Vector3.mjs";
import Matrix3 from "./Matrix3.mjs";


var Quaternion = class {
    constructor(w = 1, x = 0, y = 0, z = 0) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    multiply(q) {
        var w = this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z;
        var x = this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y;
        var y = this.w * q.y + this.y * q.w + this.z * q.x - this.x * q.z;
        var z = this.w * q.z + this.z * q.w + this.x * q.y - this.y * q.x;
        return new this.constructor(w, x, y, z);
    }

    multiplyInPlace(q) {
        var oldW = this.w;
        var oldX = this.x;
        var oldY = this.y;
        var oldZ = this.z;
        this.w = oldW * q.w - oldX * q.x - oldY * q.y - oldZ * q.z;
        this.x = oldW * q.x + oldX * q.w + oldY * q.z - oldZ * q.y;
        this.y = oldW * q.y + oldY * q.w + oldZ * q.x - oldX * q.z;
        this.z = oldW * q.z + oldZ * q.w + oldX * q.y - oldY * q.x;
        return this;
    }

    multiplyVector3(v) {
        var q = new this.constructor(0, v.x, v.y, v.z);
        var finalQ = this.multiply(q).multiply(this.conjugate());
        return new Vector3(finalQ.x, finalQ.y, finalQ.z);
    }

    multiplyVector3InPlace(v) {
        var q = new this.constructor(0, v.x, v.y, v.z);
        var finalQ = this.multiply(q).multiply(this.conjugate());
        v.x = finalQ.x;
        v.y = finalQ.y;
        v.z = finalQ.z;
        return v;
    }

    slerp(q, t) {
        if (t == 0 || t == 1) {
            return (t == 0) ? this.copy() : q.copy();
        }
        let cosHalfTheta = this.w * q.w + this.x * q.x + this.y * q.y + this.z * q.z;

        if (cosHalfTheta < 0) {
            q = new this.constructor(-q.w, -q.x, -q.y, -q.z);
            cosHalfTheta = -cosHalfTheta;
        }

        if (cosHalfTheta > 0.9995) {
            let w = this.w + t * (q.w - this.w);
            let x = this.x + t * (q.x - this.x);
            let y = this.y + t * (q.y - this.y);
            let z = this.z + t * (q.z - this.z);
            let norm = Math.sqrt(w * w + x * x + y * y + z * z);
            return new this.constructor(w / norm, x / norm, y / norm, z / norm);
        }

        const halfTheta = Math.acos(cosHalfTheta);
        const sinHalfTheta = Math.sin(halfTheta);

        const ratioA = Math.sin((1 - t) * halfTheta) / sinHalfTheta;
        const ratioB = Math.sin(t * halfTheta) / sinHalfTheta;

        return new this.constructor(
            this.w * ratioA + q.w * ratioB,
            this.x * ratioA + q.x * ratioB,
            this.y * ratioA + q.y * ratioB,
            this.z * ratioA + q.z * ratioB
        );
    }


    conjugate() {
        return new this.constructor(this.w, -this.x, -this.y, -this.z);
    }

    conjugateInPlace() {
        this.x = -this.x;
        this.y = -this.y;
        this.z = -this.z;
        return this;
    }

    copy() {
        return new this.constructor(this);
    }

    normalize() {
        var length = this.magnitude();
        if (length == 0) {
            return this;
        }
        return this.scale(1 / length);
    }

    normalizeInPlace() {
        var length = this.magnitude();
        if (length == 0) {
            return this;
        }
        return this.scaleInPlace(1 / length);
    }

    magnitude() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
    }

    scale(s) {
        return new this.constructor(this.w * s, this.x * s, this.y * s, this.z * s);
    }

    scaleInPlace(s) {
        this.x *= s;
        this.y *= s;
        this.z *= s;
        this.w *= s;
        return this;
    }

    copy() {
        return new this.constructor(this.w, this.x, this.y, this.z);
    }

    set(q) {
        this.w = q.w;
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        return this;
    }

    reset() {
        this.w = 1;
        this.x = 0;
        this.y = 0;
        this.z = 0;
        return this;
    }

    toMatrix3() {
        var matrix = new Matrix3();
        var wx = this.w * this.x;
        var wy = this.w * this.y;
        var wz = this.w * this.z;
        var xx = this.x * this.x;
        var xy = this.x * this.y;
        var xz = this.x * this.z;
        var yy = this.y * this.y;
        var yz = this.y * this.z;
        var zz = this.z * this.z;

        matrix.elements[0] = 1 - 2 * (yy + zz);
        matrix.elements[1] = 2 * (xy - wz);
        matrix.elements[2] = 2 * (xz + wy);

        matrix.elements[3] = 2 * (xy + wz);
        matrix.elements[4] = 1 - 2 * (xx + zz);
        matrix.elements[5] = 2 * (yz - wx);

        matrix.elements[6] = 2 * (xz - wy);
        matrix.elements[7] = 2 * (yz + wx);
        matrix.elements[8] = 1 - 2 * (xx + yy);

        return matrix;
    }

    rotateByAngularVelocity(angularVelocity) {
        var angularVelocityQuaternion = new Quaternion(1, angularVelocity.x * 0.5, angularVelocity.y * 0.5, angularVelocity.z * 0.5);
        return angularVelocityQuaternion.multiply(this).normalizeInPlace();
    }


    static lookAt(direction, up) {

        var right = up.cross(direction).normalize();
        up = direction.cross(right);

        var trace = right.x + up.y + direction.z;

        var w;
        var x;
        var y;
        var z;
        var s;

        if (trace > 0) {
            s = 0.5 / Math.sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (up.z - direction.y) * s;
            y = (direction.x - right.z) * s;
            z = (right.y - up.x) * s;
        }
        else if (right.x > up.y && right.x > direction.z) {
            s = Math.sqrt(1.0 + right.x - up.y - direction.z) * 2;
            x = 0.25 * s;
            y = (right.y + up.x) / s;
            z = (direction.x + right.z) / s;
            w = (up.z - direction.y) / s;
        }
        else if (up.y > direction.z) {
            s = Math.sqrt(1.0 + up.y - right.x - direction.z) * 2;
            x = (right.y + up.x) / s;
            y = 0.25 * s;
            z = (up.z + direction.y) / s;
            w = (direction.x - right.z) / s;
        }
        else {
            s = Math.sqrt(1.0 + direction.z - right.x - up.y) * 2;
            x = (direction.x + right.z) / s;
            y = (up.z + direction.y) / s;
            z = 0.25 * s;
            w = (right.y - up.x) / s;
        }

        return new this(w, x, y, z).normalizeInPlace();
    }

    toJSON() {
        return {
            w: this.w,
            x: this.x,
            y: this.y,
            z: this.z
        };
    }
    static from(w = 1, x = 0, y = 0, z = 0) {
        return new this(w?.w ?? w[0] ?? w ?? 1,
            w?.x ?? w[1] ?? x ?? 0,
            w?.y ?? w[2] ?? y ?? 0,
            w?.z ?? w[3] ?? z ?? 0);
    }
    static fromJSON(json) {
        return this.from(json);
    }
    [Symbol.iterator]() {
        return [this.w, this.x, this.y, this.z][Symbol.iterator]();
    }
};

export default Quaternion;