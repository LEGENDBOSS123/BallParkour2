import Vector3 from "../Math3D/Vector3.mjs";


var CollisionContact = class {
    constructor(options) {
        this.type = "CollisionContact";
        this.impulse = options?.impulse;

        this.normal = options?.normal;
        this.penetration = options?.penetration;

        this.body1 = options?.body1;
        this.body2 = options?.body2;

        this.point = options?.point;
        this.velocity = options?.velocity;

        this.body1Map = options?.body1Map;
        this.body2Map = options?.body2Map;

        this.material = options?.combinedMaterial;

        this.denominator = 0;
        this.denominatorFric = 0;
        this.solved = false;
    }

    solve() {
        this.velocity = this.body1.getVelocityAtPosition(this.point).subtractInPlace(this.body2.getVelocityAtPosition(this.point));
        var impactSpeed = this.velocity.dot(this.normal);
        if(impactSpeed > 0){
            return false;
        }
        var tangential = this.velocity.projectOntoPlane(this.normal);
        if(!this.solved){
            var radius1 = this.point.subtract(this.body1.maxParent.global.body.position);
            var radius2 = this.point.subtract(this.body2.maxParent.global.body.position);

            var rotationalEffects1 = this.normal.dot(this.body1.maxParent.global.body.inverseMomentOfInertia.multiplyVector3(radius1.cross(this.normal)).cross(radius1));
            var rotationalEffects2 = this.normal.dot(this.body2.maxParent.global.body.inverseMomentOfInertia.multiplyVector3(radius2.cross(this.normal)).cross(radius2));
            rotationalEffects1 = isFinite(rotationalEffects1) ? rotationalEffects1 : 0;
            rotationalEffects2 = isFinite(rotationalEffects2) ? rotationalEffects2 : 0;

            
            var tangentialNorm = tangential.normalize();
            var rotationalEffects1Fric = tangentialNorm.dot(this.body1.maxParent.global.body.inverseMomentOfInertia.multiplyVector3(radius1.cross(tangentialNorm)).cross(radius1));
            var rotationalEffects2Fric = tangentialNorm.dot(this.body2.maxParent.global.body.inverseMomentOfInertia.multiplyVector3(radius2.cross(tangentialNorm)).cross(radius2));
            rotationalEffects1Fric = isFinite(rotationalEffects1Fric) ? rotationalEffects1Fric : 0;
            rotationalEffects2Fric = isFinite(rotationalEffects2Fric) ? rotationalEffects2Fric : 0;

            var invMass1 = this.body1.maxParent.global.body.inverseMass;
            var invMass2 = this.body2.maxParent.global.body.inverseMass;

            if (this.body1.maxParent.isImmovable()) {
                invMass1 = 0;
                rotationalEffects1 = 0;
                rotationalEffects1Fric = 0;
            }
            if (this.body2.maxParent.isImmovable()) {
                invMass2 = 0;
                rotationalEffects2 = 0;
                rotationalEffects2Fric = 0;
            }
            this.denominator = invMass1 * (1 - this.body1.maxParent.global.body.linearDamping.multiply(this.normal).magnitude()) + rotationalEffects1 * (1 - this.body1.maxParent.global.body.angularDamping);

            this.denominator += invMass2 * (1 - this.body2.maxParent.global.body.linearDamping.multiply(this.normal).magnitude()) + rotationalEffects2 * (1 - this.body2.maxParent.global.body.angularDamping);

            this.denominatorFric = invMass1 * (1 - this.body1.maxParent.global.body.linearDamping.multiply(tangentialNorm).magnitude()) + rotationalEffects1Fric * (1 - this.body1.maxParent.global.body.angularDamping);

            this.denominatorFric += invMass2 * (1 - this.body2.maxParent.global.body.linearDamping.multiply(tangentialNorm).magnitude()) + rotationalEffects2Fric * (1 - this.body2.maxParent.global.body.angularDamping);
            if (this.denominator == 0) {
                return false;
            }
        }
        var impulse = - (1 + this.material.restitution) * impactSpeed / this.denominator;

        if (impulse < 0) {
            impulse = 0;
        }


        var maxFriction = tangential.magnitude() / this.denominatorFric;
        tangential.normalizeInPlace();
        var friction = impulse * this.material.friction;
        this.impulse = tangential.scale(-1 * Math.max(0, Math.min(maxFriction, friction))).addInPlace(this.normal.scale(impulse));
        this.solved = true;
        return true;
    }

    applyForces(){
        this.body1.maxParent.applyForce(this.impulse, this.point);
        this.body2.maxParent.applyForce(this.impulse.scale(-1), this.point);
    }

    copy() {
        var c = new this.constructor();
        c.normal = this.normal.copy();
        c.penetration = this.penetration;

        c.body1 = this.body1;
        c.body2 = this.body2;
        c.point = this.point;
        c.velocity = this.velocity;

        c.solved = this.solved;
        c.impulse = this.impulse;

        c.combinedMaterial = this.combinedMaterial;
        return c;
    }

    toJSON() {
        return {
            normal: this.normal.toJSON(),
            penetration: this.penetration,
            body1: this.body1.id,
            body2: this.body2.id,
            point: this.point.toJSON(),
            velocity: this.velocity.toJSON(),
            solved: this.solved,
            impulse: this.impulse.toJSON(),
            combinedMaterial: this.combinedMaterial.toJSON()
        }
    }

    static fromJSON(json, world) {
        var c = new this();
        c.normal = new Vector3().fromJSON(json.normal);
        c.penetration = json.penetration;

        c.body1 = world.getByID(json.body1);
        c.body2 = world.getByID(json.body2);
        c.point = new Vector3().fromJSON(json.point);
        c.velocity = new Vector3().fromJSON(json.velocity);

        c.solved = json.solved;
        c.impulse = new Vector3().fromJSON(json.impulse);

        c.combinedMaterial = new Material().fromJSON(json.combinedMaterial);
        return c;
    }
};

export default CollisionContact;