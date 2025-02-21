
import Composite from "./Composite.mjs";
import Matrix3 from "../Math3D/Matrix3.mjs";
import Vector3 from "../Math3D/Vector3.mjs";
import Quaternion from "../Math3D/Quaternion.mjs";
import ClassRegistry from "../Core/ClassRegistry.mjs";


var Polyhedron = class extends Composite {
    static name = "POLYHEDRON";
    constructor(options) {
        super(options);
        this.localVertices = options?.localVertices ?? [
            new Vector3(1, -1, 1),
            new Vector3(-1, -1, 1),
            new Vector3(-1, 1, 1),
            new Vector3(1, 1, 1),
            new Vector3(1, -1, -1),
            new Vector3(-1, -1, -1),
            new Vector3(-1, 1, -1),
            new Vector3(1, 1, -1)
        ];
        this.globalVertices = options?.globalVertices ?? [];
        this.faces = options?.faces ?? [
            [0, 2, 1], [0, 3, 2], [4, 5, 6],
            [4, 6, 7], [0, 1, 5], [0, 5, 4],
            [2, 3, 7], [2, 7, 6], [0, 4, 7],
            [0, 7, 3], [1, 2, 6], [1, 6, 5],
        ];
        this.calculateGlobalVertices();
        this.setLocalFlag(this.constructor.FLAGS.OCCUPIES_SPACE, true);
        this.calculateLocalHitbox();
        this.calculateGlobalHitbox();
    }

    calculateGlobalVertices() {
        this.globalVertices.length = this.localVertices.length;
        for (var i = 0; i < this.localVertices.length; i++) {
            this.globalVertices[i] = this.translateLocalToWorld(this.localVertices[i]);
        }
    }


    calculateLocalMomentOfInertia() {
        this.local.body.momentOfInertia = Matrix3.zero();
        var I = (1 / 12) * this.local.body.mass * 8;
        this.local.body.momentOfInertia.set(0, 0, I);
        this.local.body.momentOfInertia.set(1, 1, I);
        this.local.body.momentOfInertia.set(2, 2, I);
        return this.local.body.momentOfInertia;
    }

    calculateLocalHitbox() {
        this.local.hitbox.min = new Vector3(Infinity, Infinity, Infinity);
        this.local.hitbox.max = new Vector3(-Infinity, -Infinity, -Infinity);
        for (var v of this.localVertices) {
            this.local.hitbox.expandToFitPoint(v);
        }
        return this.local.hitbox;
    }

    calculateGlobalHitbox() {
        this.calculateGlobalVertices();
        this.global.hitbox.min = new Vector3(Infinity, Infinity, Infinity);
        this.global.hitbox.max = new Vector3(-Infinity, -Infinity, -Infinity);
        for (var v of this.globalVertices) {
            this.global.hitbox.expandToFitPoint(v);
        }
        return this.global.hitbox;
    }

    setMesh(options, graphicsEngine) {
        var geometry = new graphicsEngine.THREE.BufferGeometry();
        var positions = new Float32Array(this.faces.length * 9);
        var normals = new Float32Array(this.faces.length * 9);
        var indices = [];

        var vertexIndex = 0;
        for (var face of this.faces) {
            var v1 = this.localVertices[face[0]];
            var v2 = this.localVertices[face[1]];
            var v3 = this.localVertices[face[2]];
            var edge1 = v2.subtract(v1);
            var edge2 = v3.subtract(v1);
            var normal = edge1.cross(edge2).normalize();
            positions[vertexIndex * 9 + 0] = v1.x;
            positions[vertexIndex * 9 + 1] = v1.y;
            positions[vertexIndex * 9 + 2] = v1.z;
            normals[vertexIndex * 9 + 0] = normal.x;
            normals[vertexIndex * 9 + 1] = normal.y;
            normals[vertexIndex * 9 + 2] = normal.z;

            positions[vertexIndex * 9 + 3] = v2.x;
            positions[vertexIndex * 9 + 4] = v2.y;
            positions[vertexIndex * 9 + 5] = v2.z;
            normals[vertexIndex * 9 + 3] = normal.x;
            normals[vertexIndex * 9 + 4] = normal.y;
            normals[vertexIndex * 9 + 5] = normal.z;

            positions[vertexIndex * 9 + 6] = v3.x;
            positions[vertexIndex * 9 + 7] = v3.y;
            positions[vertexIndex * 9 + 8] = v3.z;
            normals[vertexIndex * 9 + 6] = normal.x;
            normals[vertexIndex * 9 + 7] = normal.y;
            normals[vertexIndex * 9 + 8] = normal.z;

            indices.push(vertexIndex * 3, vertexIndex * 3 + 1, vertexIndex * 3 + 2);
            vertexIndex++;
        }
        geometry.setAttribute('position', new graphicsEngine.THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('normal', new graphicsEngine.THREE.BufferAttribute(normals, 3));
        geometry.setIndex(indices);
        geometry.computeVertexNormals();

        var material = options?.material ?? new graphicsEngine.THREE.MeshPhongMaterial({ color: options?.color ?? 0x00ff00, wireframe: false });
        this.mesh = graphicsEngine.meshLinker.createMeshData(new graphicsEngine.THREE.Mesh(geometry, material));
    }

    setMeshAndAddToScene(options, graphicsEngine) {
        this.setMesh(options, graphicsEngine);
        this.addToScene(graphicsEngine.scene);
    }

    fromMesh(mesh, graphicsEngine) {
        const geometry = mesh.geometry;
        var vertices = [];
        var faces = [];

        var positions = geometry.attributes.position.array;
        var numVertices = positions.length / 3;

        for (let i = 0; i < numVertices; i++) {
            var x = positions[i * 3];
            var y = positions[i * 3 + 1];
            var z = positions[i * 3 + 2];
            vertices.push(new Vector3(x, y, z).multiply(new Vector3(mesh.scale.x, mesh.scale.y, mesh.scale.z).map(Math.abs)));
        }


        var indices = geometry.index ? geometry.index.array : null;
        if (indices) {
            var numFaces = indices.length / 3;
            for (let i = 0; i < numFaces; i++) {
                var a = indices[i * 3]
                var b = indices[i * 3 + 1]
                var c = indices[i * 3 + 2]
                faces.push([a, b, c]);
            }
        } else {
            var numFaces = numVertices / 3;
            for (let i = 0; i < numFaces; i++) {
                var a = i * 3;
                var b = i * 3 + 1;
                var c = i * 3 + 2;
                faces.push([a, b, c]);
            }
        }
        this.localVertices = vertices;
        this.faces = faces;
        
        this.global.body.rotation = new Quaternion(mesh.quaternion.w, mesh.quaternion.x, mesh.quaternion.y, mesh.quaternion.z);
        this.global.body.previousRotation = this.global.body.rotation.copy();
        this.global.body.setPosition(new Vector3(mesh.position.x, mesh.position.y, mesh.position.z));
        this.calculateLocalHitbox();
        this.calculateGlobalHitbox();
        return this;
    }

    toJSON() {
        var poly = super.toJSON();
        poly.localVertices = this.localVertices.map(function (v) { return v.toJSON() });
        poly.globalVertices = this.globalVertices.map(function (v) { return v.toJSON() });
        poly.faces = this.faces.map(function (f) { return [...f] });
        return poly;
    }

    static fromJSON(json, world) {
        var poly = super.fromJSON(json, world);
        poly.localVertices = json.localVertices.map(function (v) { return Vector3.fromJSON(v) });
        poly.globalVertices = json.globalVertices.map(function (v) { return Vector3.fromJSON(v) });
        poly.faces = json.faces.map(function (f) { return [...f] });
        return poly;
    }
};

ClassRegistry.register(Polyhedron);

export default Polyhedron;