
var Entity = class {
    constructor(options) {
        this.id = options?.id ?? -1;
        this.entitySystem = options?.entitySystem ?? null;
        this.oldShape = options?.oldShape ?? null;
        this.isEntity = true;
    }

    updateShapeID(shape = this.oldShape) {
        if (!shape) {
            return;
        }
        if (!this.oldShape) {
            this.oldShape = shape;
        }
        if (!this.entitySystem || shape.maxParent.id == -1) {
            return;
        }
        if (this.entitySystem.shapeLookup[this.oldShape.maxParent.id] && this.oldShape.maxParent.id == shape.maxParent.id) {
            return;
        }
        if(this.entitySystem.shapeLookup[this.oldShape.maxParent.id]){
            delete this.entitySystem.shapeLookup[this.oldShape.maxParent.id];
        }
        this.entitySystem.shapeLookup[shape.maxParent.id] = this;
        this.oldShape = shape;
    }

    getMainShape() {

    }
}

export default Entity;