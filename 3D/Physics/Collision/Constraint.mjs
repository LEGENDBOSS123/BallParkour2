var Constraint = class{

    static CONSTRAINTS = {};

    static CONSTRAINTS_CLASSES = {};

    static CONSTRAINT_MAX_ID = 0;
    static REGISTER_CONSTRAINT(obj) {
        this.CONSTRAINTS[obj.name] = this.CONSTRAINT_MAX_ID++;
        this.CONSTRAINTS_CLASSES[this.CONSTRAINTS[obj.name]] = obj;
    }

    static name = "CONSTRAINT";


    constructor(options) {
        this.id = options?.id ?? -1;
        this.constraint = options?.constraint ?? this.constructor.CONSTRAINTS.CONSTRAINT;
        this.world = options?.world ?? null;

        this.events = {};
        this.toBeRemoved = options?.toBeRemoved ?? false;

        this.graphicsEngine = options?.graphicsEngine ?? null;
        this._mesh = options?.mesh ?? null;
    }

    addEventListener(event, callback) {
        if (!this.events[event]) {
            this.events[event] = [];
        }
        this.events[event].push(callback);
    }

    removeEventListener(event, callback) {
        if (!this.events[event]) {
            return;
        }
        var index = this.events[event].indexOf(callback);
        if (index == -1) {
            return;
        }
        this.events[event].splice(index, 1);
    }

    solve(){

    }

    setWorld(world) {
        this.world = world;
        return this;
    }

    setMesh(options, graphicsEngine) {
        return null;
    }

    setMeshAndAddToScene(options, graphicsEngine) {
        return null;
    }

    addToScene(scene) {
        if (!this.mesh) {
            return null;
        }
        if (this.mesh.isMeshLink) {
            scene.add(this.mesh.mesh);
            return;
        }
        scene.add(this.mesh);
    }

    set mesh(value) {
        if (this.id == -1 || !value) {
            this._mesh = value;
            return;
        }
        this.graphicsEngine.meshLinker.addMesh(this.id, value);
    }

    get mesh() {
        if (this.id == -1) {
            return this._mesh;
        }
        return this.graphicsEngine.meshLinker.getByID(this.id);
    }

    toJSON(){
        var json = {};
        json.id = this.id
        json.constraint = this.shape;
        json.world = this.world?.id ?? null;
        json.toBeRemoved = this.toBeRemoved ?? false;
        return json;
    }

    static fromJSON(json, world){
        var constraint = new this();
        constraint.id = json.id;
        constraint.constraint = json.constraint;
        constraint.world = world;
        constraint.toBeRemoved = json.toBeRemoved;
        return constraint;
    }

}


Constraint.REGISTER_CONSTRAINT(Constraint);


export default Constraint;