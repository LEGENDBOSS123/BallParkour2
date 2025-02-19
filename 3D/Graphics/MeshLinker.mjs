
var MeshLinker = class {
    constructor() {
        this.meshes = {};
    }

    createMeshData(mesh, animations = []) {
        return {
            mesh: mesh,
            animations: animations,
            id: null,
            isMeshLink: true
        }
    }
    addMesh(id, mesh) {
        this.meshes[id] = mesh;
        mesh.id = id;
    }
    removeMesh(id) {
        if (!this.meshes[id]) {
            return;
        }
        delete this.meshes[id];
    }
    getByID(id) {
        return this.meshes[id];
    }
    update(previousWorld, world, lerpAmount) {
        for (var meshID in this.meshes) {
            var mesh = this.meshes[meshID];
            if (!world.getByID(meshID) || !previousWorld.getByID(meshID)) {
                mesh.mesh.visible = false;
                continue;
            }
            mesh.mesh.visible = true;
            var composite = world.getByID(meshID);
            var previousComposite = previousWorld.getByID(meshID);
            composite.lerpMesh(previousComposite, lerpAmount);
        }
    }
};

export default MeshLinker;