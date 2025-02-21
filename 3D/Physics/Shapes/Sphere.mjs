import Composite from "./Composite.mjs";
import Vector3 from "../Math3D/Vector3.mjs";
import Matrix3 from "../Math3D/Matrix3.mjs";
import ClassRegistry from "../Core/ClassRegistry.mjs";
var Sphere = class extends Composite {

    static name = "SPHERE";

    constructor(options) {
        super(options);
        this.radius = options?.radius ?? 1;
        this.setLocalFlag(this.constructor.FLAGS.OCCUPIES_SPACE, true);
        this.dimensionsChanged();
    }

    calculateLocalHitbox() {
        this.local.hitbox.min = new Vector3(-this.radius, -this.radius, -this.radius);
        this.local.hitbox.max = new Vector3(this.radius, this.radius, this.radius);
        return this.hitbox;
    }

    calculateGlobalHitbox(forced = false) {
        if(!this.global.body.changed && !forced && this.global.body.position.equals(this.global.body.actualPreviousPosition) && this.global.body.previousRotation.equals(this.global.body.rotation)){
            return;
        }
        this.global.hitbox.min = this.local.hitbox.min.add(this.global.body.position);
        this.global.hitbox.max = this.local.hitbox.max.add(this.global.body.position);
        return this.global.hitbox;
    }

    rotateLocalMomentOfInertia(quaternion) {
        return this.local.body.momentOfInertia;
    }

    calculateLocalMomentOfInertia() {
        this.local.body.momentOfInertia = Matrix3.zero();
        var I = (2 / 5) * this.local.body.mass * this.radius * this.radius;
        this.local.body.momentOfInertia.set(0, 0, I);
        this.local.body.momentOfInertia.set(1, 1, I);
        this.local.body.momentOfInertia.set(2, 2, I);
        return this.local.body.momentOfInertia;
    }

    setMesh(options, graphicsEngine) {
        var geometry = options?.geometry ?? new graphicsEngine.THREE.SphereGeometry(this.radius, 32, 32);
        this.mesh = graphicsEngine.meshLinker.createMeshData(new graphicsEngine.THREE.Mesh(geometry, options?.material ?? new graphicsEngine.THREE.MeshPhongMaterial({ color: 0x00ff00, wireframe: true })));
    }

    setMeshAndAddToScene(options, graphicsEngine) {
        this.setMesh(options, graphicsEngine);
        this.addToScene(graphicsEngine.scene);
    }

    toJSON() {
        var composite = super.toJSON();
        composite.radius = this.radius;
        return composite;
    }

    static fromJSON(json, world) {
        var sphere = super.fromJSON(json, world);
        sphere.radius = json.radius;
        return sphere;
    }
};

ClassRegistry.register(Sphere);

export default Sphere;