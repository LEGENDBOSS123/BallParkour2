import Sphere from "./3D/Physics/Shapes/Sphere.mjs";
import Composite from "./3D/Physics/Shapes/Composite.mjs";
import Vector3 from "./3D/Physics/Math3D/Vector3.mjs";
import Entity from "./Entity.mjs";
var SlimeSpawner = class extends Entity {
    constructor(options) {
        super(options);
        this.sphere = new Sphere(options?.sphere);
        this.sphere.radius = 15;
        this.sphere.local.body.mass = Infinity;
        this.sphere.calculateLocalHitbox();

        this.sphere.setLocalFlag(Composite.FLAGS.STATIC, true);
        this.sphere.setRestitution(0);
        this.sphere.setFriction(0);
        this.updateShapeID(this.sphere);
    }

    addToScene(scene) {
        this.sphere.addToScene(scene);
    }

    addToWorld(world) {
        world.addComposite(this.sphere);
        this.updateShapeID(this.sphere);
    }

    setMeshAndAddToScene(options, graphicsEngine) {
        graphicsEngine.load("slimeSpawner.glb", function (gltf) {
            gltf.scene.traverse(function (child) {
                if (child.isMesh) {
                    child.castShadow = true;
                    child.receiveShadow = true;
                }
            })
            this.sphere.mesh = graphicsEngine.meshLinker.createMeshData(gltf.scene);
            this.addToScene(graphicsEngine.scene);
        }.bind(this));
    }

    toJSON() {
        var json = {};
        json.sphere = this.sphere.id;
        return json;
    }

    static fromJSON(json, world) {
        var slimeSpawner = new this();
        this.sphere = json.sphere;
        return slimeSpawner;
    }

    updateReferences(world) {
        this.sphere = world.getByID(this.sphere);
    }

    spawnSlime(slimeClass, world, graphicsEngine) {
        var slime = new slimeClass({
            sphere: {
                global: {
                    body: {
                        position: this.sphere.global.body.position.add(new Vector3(0, 10, 0)),
                        acceleration: new Vector3(0, -0.2, 0),
                    }
                }
            }
        });
        slime.addToWorld(world);
        this.entitySystem.register(slime);
        slime.setMeshAndAddToScene({}, graphicsEngine);
        return slime;
    }
    getMainShape() {
        return this.sphere;
    }
}

export default SlimeSpawner;