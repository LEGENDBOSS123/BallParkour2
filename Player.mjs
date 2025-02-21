import Composite from "./3D/Physics/Shapes/Composite.mjs";
import Sphere from "./3D/Physics/Shapes/Sphere.mjs";
import Vector3 from "./3D/Physics/Math3D/Vector3.mjs";
import Entity from "./Entity.mjs";
import Quaternion from "./3D/Physics/Math3D/Quaternion.mjs";
var Player = class extends Entity {
    constructor(options) {
        super(options);
        var mass = options?.local?.body?.mass ?? 1;
        if (options?.local?.body?.mass) {
            options.local.body.mass = 0;
        }
        this.moveStrength = options?.moveStrength ?? new Vector3(0.1, 0.1, 0.1);
        this.jumpStrength = options?.jumpStrength ?? 1;
        this.composite = new Composite(options);
        this.spheres = [];
        this.spheres.push(new Sphere({
            radius: (options?.radius ?? 1),
            local: {
                body: {
                    mass: 1,
                    position: new Vector3(0, 0, 0),
                }
            }
        }));

       
        for (var sphere of this.spheres) {
            this.composite.add(sphere);
            sphere.collisionMask = 0;
            sphere.collisionMask = sphere.setBitMask(sphere.collisionMask, "P", true);
        }
        this.composite.setLocalFlag(Composite.FLAGS.CENTER_OF_MASS, true);
        this.composite.syncAll();
        this.composite.setRestitution(0);
        this.composite.setFriction(0);
        for (var sphere of this.spheres) {
            sphere.setRestitution(1);
            sphere.setFriction(100);
        }
        this.spawnPoint = this.spheres[0].global.body.position.copy();
        this.canJump = false;
        this.jumpPostCollision = function (contact) {
            if (contact.body1.maxParent == this.composite) {
                if (contact.normal.dot(new Vector3(0, 1, 0)) > 0.75) {
                    this.canJump = true;
                }
            }
            else {
                if (contact.normal.dot(new Vector3(0, -1, 0)) > 0.75) {
                    this.canJump = true;
                }
            }
        }.bind(this);

        this.postStepCallback = function () {
            var vel = this.composite.global.body.getVelocity();
            var velXZ = new Vector3(vel.x, 0, vel.z);
            if (this.composite.mesh) {
                this.composite.mesh.animations.actions[0].timeScale = 1;
                this.composite.mesh.animations.actions[0].timeScale = velXZ.magnitude() * 5;
            }
            if (velXZ.magnitudeSquared() < 0.001) {
                if (this.composite.mesh) {
                    //this.composite.mesh.animations.actions[0].timeScale = 0;
                }
                return;
            }

            //this.composite.global.body.rotation = Quaternion.lookAt(velXZ.normalize(), new Vector3(0, 1, 0));
        }.bind(this);


        this.preStepCallback = function(){
            //this.canJump = false;
        }.bind(this);

        this.spheres[0].addEventListener("postCollision", this.jumpPostCollision);
        this.composite.addEventListener("postStep", this.postStepCallback);
        this.composite.addEventListener("preStep", this.preStepCallback);
        this.updateShapeID(this.composite);
    }

    addToScene(scene) {
        this.composite.addToScene(scene);
        for (var sphere of this.spheres) {
            sphere.addToScene(scene);
        }
    }

    addToWorld(world) {
        world.addComposite(this.composite);
        for (var sphere of this.spheres) {
            world.addComposite(sphere);
        }
        this.updateShapeID();
    }

    setMeshAndAddToScene(options, graphicsEngine) {

        // graphicsEngine.load("player2.glb", function (gltf) {
        //     gltf.scene.scale.set(...(new Vector3(0.02, 0.02, 0.02).scale(this.spheres[0].radius)));
        //     top.gltf = gltf;
        //     for (var e of gltf.scene.children) {
        //         e.position.y -= 100;
        //     }
        //     gltf.scene.traverse(function (child) {
        //         if (child.isMesh) {
        //             child.castShadow = true;
        //             child.receiveShadow = true;
        //         }
        //     })
        //     var meshData = graphicsEngine.meshLinker.createMeshData(gltf.scene, graphicsEngine.createAnimations(gltf.scene, gltf.animations));
        //     this.composite.mesh = meshData;
        //     meshData.animations.actions[0].play();
        //     this.addToScene(graphicsEngine.scene);
        // }.bind(this));
        this.spheres.forEach(sphere => {
            sphere.setMeshAndAddToScene({}, graphicsEngine);
        });
        
    }

    update() {
    }

    respawn() {
        this.composite.global.body.setPosition(this.spawnPoint.copy());
        this.composite.global.body.setVelocity(new Vector3(0, 0, 0));
        this.composite.global.body.angularVelocity.reset();
        this.composite.global.body.rotation.reset();
        this.composite.global.body.netForce.reset();
        this.composite.global.body.netTorque.reset();
        this.canJump = true;
        this.composite.syncAll();
    }

    toJSON() {
        var json = super.toJSON();
        json.spheres = [];
        for (var sphere of this.spheres) {
            json.spheres.push(sphere.id);
        }
        json.composite = this.composite.id;
        json.moveStrength = this.moveStrength;
        json.jumpStrength = this.jumpStrength;
        json.spawnPoint = this.spawnPoint.toJSON();
        json.canJump = this.canJump;
        return json;
    }

    static fromJSON(json, world) {
        var player = super.fromJSON(json, world);
        player.moveStrength = Vector3.fromJSON(json.moveStrength);
        player.jumpStrength = json.jumpStrength;
        player.spawnPoint = Vector3.fromJSON(json.spawnPoint);
        player.composite = json.composite;
        player.spheres = json.spheres;
        player.canJump = json.canJump;
        return player;
    }

    updateReferences(world) {
        this.composite = world.getByID(this.composite);
        for (var i = 0; i < this.spheres.length; i++) {
            this.spheres[i] = world.getByID(this.spheres[i]);
        }
        this.spheres[0].addEventListener("postCollision", this.jumpPostCollision);
        this.composite.addEventListener("postStep", this.postStepCallback);
        this.composite.addEventListener("preStep", this.preStepCallback);
    }

    getMainShape() {
        return this.spheres[0];
    }
}

export default Player;