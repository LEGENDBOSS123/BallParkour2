import Composite from "./Composite.mjs";
import Matrix3 from "../Math3D/Matrix3.mjs";

var Point = class extends Composite {
    static name = "POINT";
    constructor(options) {
        super(options);
        this.shape = this.constructor.SHAPES.POINT;
        this.setLocalFlag(this.constructor.FLAGS.OCCUPIES_SPACE, true);
        this.calculateLocalHitbox();
        this.calculateGlobalHitbox();
    }

    calculateLocalMomentOfInertia() {
        this.local.body.momentOfInertia = Matrix3.zero();
        return this.local.body.momentOfInertia;
    }

    rotateLocalMomentOfInertia(quaternion) {
        return this.local.body.momentOfInertia;
    }

    setMesh(options, graphicsEngine) {
        var geometry = options?.geometry ?? new graphicsEngine.THREE.SphereGeometry(options?.radius ?? 1, 16, 16);
        this.mesh = graphicsEngine.meshLinker.createMeshData(new graphicsEngine.THREE.Mesh(geometry, options?.material ?? new graphicsEngine.THREE.MeshPhongMaterial({ color: 0x00ff00, wireframe: true })));
    }

    toJSON() {
        var composite = super.toJSON();
        return composite;
    }

    static fromJSON(json, world) {
        var point = new this(new Composite(json));
        return box;
    }
};

Composite.REGISTER_SHAPE(Point);

export default Point;