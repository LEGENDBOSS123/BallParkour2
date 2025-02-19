import WorldObject from "../Core/WorldObject.mjs";
import ClassRegistry from "../Core/ClassRegistry.mjs";

var Constraint = class extends WorldObject {

    
    static name = "CONSTRAINT";

    constructor(options) {
        super(options);
    }

    solve(){
        return null;
    }

    toJSON(){
        var json = super.toJSON();
        return json;
    }

    static fromJSON(json, world){
        var constraint = super.fromJSON(json, world, graphicsEngine);
        return constraint;
    }
}


ClassRegistry.register(Constraint);


export default Constraint;