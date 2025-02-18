var EntitySystem = class {
    constructor(options) {
        this.maxID = options?.maxID ?? 0;
        this.all = options?.all ?? {};
        this.shapeLookup = options?.shapeLookup ?? {};
    }

    getByID(id) {
        return this.all[id];
    }

    register(entity) {
        const id = this.maxID++;
        this.all[id] = entity;
        entity.id = id;
        entity.entitySystem = this;
        entity.updateShapeID();
        return id;
    }

    getEntityFromShape(shape) {
        var id = shape.maxParent.id;
        return this.shapeLookup[id];
    }
};

export default EntitySystem;
