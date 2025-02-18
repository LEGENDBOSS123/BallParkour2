import Vector3 from "../Math3D/Vector3.mjs";
import DistanceConstraint from "./DistanceConstraint.mjs";
import CollisionContact from "./CollisionContact.mjs";
import Triangle from "../Shapes/Triangle.mjs";
import Composite from "../Shapes/Composite.mjs";
var CollisionDetector = class {

    static seperatorCharacter = ":";

    constructor(options) {
        this.pairs = options?.pairs ?? new Map();
        this.world = options?.world ?? null;
        this.contacts = options?.contacts ?? [];
        this.handlers = {};
        this.binarySearchDepth = options?.binarySearchDepth ?? 8;
        this.iterations = options?.iterations ?? 4;
        this.initHandlers();
    }

    inPairs(shape1, shape2) {
        if (shape1.id > shape2.id) {
            return this.pairs.has(shape2.id + this.constructor.seperatorCharacter + shape1.id);
        }

        return this.pairs.has(shape1.id + this.constructor.seperatorCharacter + shape2.id);
    }

    addContact(contact) {
        this.contacts.push(contact);
    }

    addPair(shape1, shape2) {

        if (!shape1.canCollideWith(shape2)) {
            return;
        }
        if (!shape1.global.hitbox.intersects(shape2.global.hitbox)) {
            return;
        }
        if (shape1.id > shape2.id) {
            var temp = shape1;
            shape1 = shape2;
            shape2 = temp;
        }
        if (!(this.handlers[shape1.shape]?.[shape2.shape] || this.handlers[shape2.shape]?.[shape1.shape])) {
            return;
        }
        return this.pairs.set(shape1.id + this.constructor.seperatorCharacter + shape2.id, [shape1, shape2]);
    }

    detectCollision(shape1, shape2) {
        if (shape1.maxParent == shape2.maxParent) {
            return false;
        }
        if (shape1.getLocalFlag(Composite.FLAGS.STATIC) && shape2.getLocalFlag(Composite.FLAGS.STATIC)) {
            return false;
        }
        if (shape1.shape > shape2.shape) {
            var temp = shape1;
            shape1 = shape2;
            shape2 = temp;
        }
        return this.handlers[shape1.shape]?.[shape2.shape]?.bind(this)(shape1, shape2);
    }

    initHandlers() {
        this.handlers[Composite.SHAPES.SPHERE] = {};
        this.handlers[Composite.SHAPES.SPHERE][Composite.SHAPES.SPHERE] = this.handleSphereSphere;
        this.handlers[Composite.SHAPES.SPHERE][Composite.SHAPES.TERRAIN3] = this.handleSphereTerrain;
        this.handlers[Composite.SHAPES.SPHERE][Composite.SHAPES.BOX] = this.handleSphereBox;
        this.handlers[Composite.SHAPES.TERRAIN3] = {};
        this.handlers[Composite.SHAPES.TERRAIN3][Composite.SHAPES.POINT] = this.handleTerrainPoint;
        this.handlers[Composite.SHAPES.BOX] = {};
    }

    handle(shape) {
        var query = this.world.spatialHash.query(shape.id);
        for (var i of query) {
            this.addPair(shape, this.world.getByID(i));
        }
    }

    handleAll(shapes) {
        for (var i in shapes) {
            this.handle(shapes[i]);
        }
    }


    resolveAll() {
        for (var value of this.pairs.values()) {
            this.detectCollision(value[0], value[1]);
        }
        this.resolveAllContacts();
        this.pairs.clear();
    }

    broadphase(shape1, shape2) {
        return shape1.global.hitbox.intersects(shape2.global.hitbox);
    }

    resolveAllContacts() {
        this.contacts = this.contacts.concat(this.world.constraints);
        var maxParentMap = new Object(null);

        for (var i = 0; i < this.contacts.length; i++) {
            var contact = this.contacts[i];
            contact.solved = false;
            contact.material = contact.body1.material.getCombined(contact.body2.material);
            if (!maxParentMap[contact.body1.maxParent.id]) {
                maxParentMap[contact.body1.maxParent.id] = { penetrationSum: 0 };
            }

            if (!maxParentMap[contact.body2.maxParent.id]) {
                maxParentMap[contact.body2.maxParent.id] = { penetrationSum: 0 };
            }

            if (contact.body1.isSensor || contact.body2.isSensor) {
                contact.penetration = new Vector3();
                contact.impulse = new Vector3();
                contact.solved = true;
            }
            var body1Map = maxParentMap[contact.body1.maxParent.id];
            var body2Map = maxParentMap[contact.body2.maxParent.id];
            contact.body1Map = body1Map;
            contact.body2Map = body2Map;
        }

        for (var iter = 0; iter < this.iterations; iter++) {
            for (var contact of this.contacts) {
                if (!contact.solve()) {
                    continue;
                }
                var a = contact.body1.maxParent;
                var b = contact.body2.maxParent;
                var a_body = a.global.body;
                var b_body = b.global.body;
                // a_body.netForce.reset();
                // b_body.netForce.reset();
                // a_body.netTorque.reset();
                // b_body.netTorque.reset();
                contact.applyForces();
                a_body.setVelocity(a_body.getVelocity().add(a_body.netForce.scale(a_body.inverseMass).multiply(new Vector3(1 - a_body.linearDamping.x, 1 - a_body.linearDamping.y, 1 - a_body.linearDamping.z))));
                b_body.setVelocity(b_body.getVelocity().add(b_body.netForce.scale(b_body.inverseMass).multiply(new Vector3(1 - b_body.linearDamping.x, 1 - b_body.linearDamping.y, 1 - b_body.linearDamping.z))));
                a_body.angularVelocity.addInPlace(a_body.inverseMomentOfInertia.multiplyVector3(a_body.netTorque).scale(1 - a_body.angularDamping));
                b_body.angularVelocity.addInPlace(b_body.inverseMomentOfInertia.multiplyVector3(b_body.netTorque).scale(1 - b_body.angularDamping));
                a.syncAll();
                b.syncAll();
                a_body.netForce.reset();
                b_body.netForce.reset();
                a_body.netTorque.reset();
                b_body.netTorque.reset();
            }
        }

        for (var i = 0; i < this.contacts.length; i++) {
            var contact = this.contacts[i];
            contact.body1Map.penetrationSum += contact.penetration.magnitude();
            contact.body2Map.penetrationSum += contact.penetration.magnitude();
        }

        for (var contact of this.contacts) {
            var translation = contact.penetration;
            var totalMass = contact.body1.maxParent.getEffectiveTotalMass(contact.normal) + contact.body2.maxParent.getEffectiveTotalMass(contact.normal);
            if (contact.type == "CollisionContact") {
                contact.body1.dispatchEvent("preCollision", [contact]);
            }
            var massRatio2 = contact.body2.maxParent.getEffectiveTotalMass() / totalMass;
            massRatio2 = isNaN(massRatio2) ? 1 : massRatio2;
            if (contact.body1Map.penetrationSum != 0) {
                contact.body1.translate(translation.scale(contact.penetration.magnitude() / contact.body1Map.penetrationSum * massRatio2));
            }
            var massRatio1 = contact.body1.maxParent.getEffectiveTotalMass() / totalMass;
            massRatio1 = isNaN(massRatio1) ? 1 : massRatio1;
            if (contact.type == "CollisionContact") {
                contact.body2.dispatchEvent("preCollision", [contact]);
            }
            if (contact.body2Map.penetrationSum != 0) {
                contact.body2.translate(translation.scale(-contact.penetration.magnitude() / contact.body2Map.penetrationSum * massRatio1));
            }
        }

        for (var contact of this.contacts) {
            if (contact.type == "CollisionContact") {
                contact.body1.dispatchEvent("postCollision", [contact]);
                contact.body2.dispatchEvent("postCollision", [contact]);
            }
        }
        this.contacts = [];
    }

    getClosestPointToAABB(v, aabb, dimensions) {
        var dimensions = dimensions ?? new Vector3(aabb.width, aabb.height, aabb.depth).scale(0.5);
        if (v.x < -dimensions.x) {
            v.x = -dimensions.x;
        }
        else if (v.x > dimensions.x) {
            v.x = dimensions.x;
        }
        if (v.y < -dimensions.y) {
            v.y = -dimensions.y;
        }
        else if (v.y > dimensions.y) {
            v.y = dimensions.y;
        }
        if (v.z < -dimensions.z) {
            v.z = -dimensions.z;
        }
        else if (v.z > dimensions.z) {
            v.z = dimensions.z;
        }
        return v;
    }

    handleSphereBox(sphere1, box1) {
        var spherePos = sphere1.global.body.position;
        var dimensions = new Vector3(box1.width, box1.height, box1.depth).scale(0.5);
        var relativePos = box1.translateWorldToLocal(spherePos);
        var dimensions2 = new Vector3(sphere1.radius, sphere1.radius, sphere1.radius).scale(0).addInPlace(dimensions);



        var prevPos = box1.global.body.rotation.conjugate().multiplyVector3(sphere1.global.body.previousPosition.subtract(box1.global.body.previousPosition));

        var delta = relativePos.subtract(prevPos);


        var minT = 0;
        var maxT = 1;
        var dimensions = new Vector3(box1.width, box1.height, box1.depth).scale(0.5);
        var binarySearch = function (t, getData = false) {
            var spherePos = sphere1.global.body.previousPosition.lerp(sphere1.global.body.position, t);
            var boxPos = box1.global.body.previousPosition.lerp(box1.global.body.position, t);
            var relativePos = box1.global.body.rotation.conjugate().multiplyVector3(spherePos.subtract(boxPos));

            var closest = this.getClosestPointToAABB(relativePos.copy(), box1);
            var distanceSquared = closest.subtract(relativePos).magnitudeSquared();
            if (getData) {
                if (distanceSquared >= sphere1.radius * sphere1.radius) {
                    return false;
                }
                if (!(relativePos.x >= dimensions.x || relativePos.y >= dimensions.y || relativePos.z >= dimensions.z || relativePos.x <= -dimensions.x || relativePos.y <= -dimensions.y || relativePos.z <= -dimensions.z)) {
                    var penetrationValues = new Vector3(relativePos.x - dimensions.x, relativePos.y - dimensions.y, relativePos.z - dimensions.z);
                    if (relativePos.x < 0) {
                        penetrationValues.x = relativePos.x + dimensions.x;
                    }
                    if (relativePos.y < 0) {
                        penetrationValues.y = relativePos.y + dimensions.y;
                    }
                    if (relativePos.z < 0) {
                        penetrationValues.z = relativePos.z + dimensions.z;
                    }
                    var absPenetrationValues = new Vector3(Math.abs(penetrationValues.x), Math.abs(penetrationValues.y), Math.abs(penetrationValues.z));
                    var contactPoint = new Vector3();
                    if (absPenetrationValues.x < absPenetrationValues.y && absPenetrationValues.x < absPenetrationValues.z) {
                        contactPoint = new Vector3(penetrationValues.x, 0, 0);
                    }
                    else if (absPenetrationValues.y < absPenetrationValues.z) {
                        contactPoint = new Vector3(0, penetrationValues.y, 0);
                    }
                    else {
                        contactPoint = new Vector3(0, 0, penetrationValues.z);
                    }
                    var contact = new CollisionContact();
                    contactPoint = boxPos.add(box1.global.body.rotation.multiplyVector3(contactPoint.addInPlace(relativePos)));
                    contact.normal = spherePos.subtract(contactPoint).normalizeInPlace();
                    contact.point = contactPoint;
                    if (contact.normal.magnitudeSquared() == 0) {
                        contact.normal = new Vector3(1, 0, 0);
                    }
                    contact.penetration = contact.normal.scale(sphere1.radius + contactPoint.distance(spherePos));
                    contact.body1 = sphere1;
                    contact.body2 = box1;
                    contact.point = sphere1.global.body.position.subtract(contact.normal.scale(sphere1.radius));
                    return contact;
                }
                closest = boxPos.add(box1.global.body.rotation.multiplyVector3(closest));
                var contact = new CollisionContact();
                contact.normal = spherePos.subtract(closest).normalizeInPlace();
                if (contact.normal.magnitudeSquared() == 0) {
                    contact.normal = new Vector3(1, 0, 0);
                }
                contact.point = closest;
                contact.penetration = contact.normal.scale(sphere1.radius).add(contact.point.subtract(sphere1.global.body.position).projectOnto(contact.normal));
                contact.body1 = sphere1;
                contact.body2 = box1;
                contact.point = sphere1.global.body.position.subtract(contact.normal.scale(sphere1.radius));

                return contact;
            }
            return distanceSquared - sphere1.radius * sphere1.radius;
        }.bind(this);
        var t = 1;
        for (var i = 0; i < this.binarySearchDepth; i++) {
            t = (minT + maxT) / 2;
            var result = binarySearch(t);
            if (result > 0) {
                minT = t;
            } else {
                maxT = t;
            }
        }
        t = maxT;
        if (t !== null) {

            var pos = prevPos.addInPlace(delta.scale(t - 0.00001));
            var closest = this.getClosestPointToAABB(pos.copy(), box1);
            if (pos.distanceSquared(closest) > 0 && pos.distanceSquared(closest) < sphere1.radius * sphere1.radius) {
                var contact = new CollisionContact();

                var normal = pos.subtract(closest).normalize();

                contact.point = box1.translateLocalToWorld(closest);
                contact.normal = box1.global.body.rotation.multiplyVector3(normal);
                contact.penetration = contact.normal.scale(sphere1.radius).add(contact.point.subtract(spherePos).projectOnto(contact.normal));
                contact.body1 = sphere1;
                contact.body2 = box1;
                contact.point = sphere1.global.body.position.subtract(contact.normal.scale(sphere1.radius));
                this.addContact(contact);
                return true;

            }
            // var contact = binarySearch(maxT, true);
            // if(contact instanceof Contact){
            //     this.addContact(contact);
            //     return true;
            // }

        }
        if (!(relativePos.x >= dimensions.x || relativePos.y >= dimensions.y || relativePos.z >= dimensions.z || relativePos.x <= -dimensions.x || relativePos.y <= -dimensions.y || relativePos.z <= -dimensions.z)) {
            var penetrationValues = new Vector3(relativePos.x - dimensions.x, relativePos.y - dimensions.y, relativePos.z - dimensions.z);
            if (relativePos.x < 0) {
                penetrationValues.x = relativePos.x + dimensions.x;
            }
            if (relativePos.y < 0) {
                penetrationValues.y = relativePos.y + dimensions.y;
            }
            if (relativePos.z < 0) {
                penetrationValues.z = relativePos.z + dimensions.z;
            }
            var absPenetrationValues = new Vector3(Math.abs(penetrationValues.x), Math.abs(penetrationValues.y), Math.abs(penetrationValues.z));
            var contactPoint = new Vector3();
            if (absPenetrationValues.x < absPenetrationValues.y && absPenetrationValues.x < absPenetrationValues.z) {
                contactPoint = new Vector3(penetrationValues.x, 0, 0);
            }
            else if (absPenetrationValues.y < absPenetrationValues.z) {
                contactPoint = new Vector3(0, penetrationValues.y, 0);
            }
            else {
                contactPoint = new Vector3(0, 0, penetrationValues.z);
            }
            var contact = new CollisionContact();
            contactPoint = box1.translateLocalToWorld(contactPoint.addInPlace(relativePos));

            contact.point = contactPoint;

            contact.normal = spherePos.subtract(contactPoint).normalizeInPlace();
            contact.penetration = contact.normal.scale(sphere1.radius + contactPoint.distance(spherePos));
            contact.body1 = sphere1;
            contact.body2 = box1;
            this.addContact(contact);
            return true;
        }


        var closestClampedPoint = this.getClosestPointToAABB(relativePos.copy(), box1);
        var distanceSquared = closestClampedPoint.subtract(relativePos).magnitudeSquared();
        if (distanceSquared >= sphere1.radius * sphere1.radius) {
            return false;
        }
        var contact = new CollisionContact();
        var closestClampedPointToWorld = box1.translateLocalToWorld(closestClampedPoint);



        contact.normal = spherePos.subtract(closestClampedPointToWorld).normalizeInPlace();
        contact.point = sphere1.global.body.position.subtract(contact.normal.scale(sphere1.radius));
        if (contact.normal.magnitudeSquared() == 0) {
            contact.normal = new Vector3(1, 0, 0);
        }
        contact.penetration = contact.normal.scale(sphere1.radius - Math.sqrt(distanceSquared));
        contact.body1 = sphere1;
        contact.body2 = box1;
        this.addContact(contact);
        return true;
    }

    handleSphereSphere(sphere1, sphere2) {

        var minT = 0;
        var maxT = 1;
        var sphere1Pos = null;
        var sphere2Pos = null;
        var distanceSquared = null;
        var binarySearch = function (t) {
            sphere1Pos = sphere1.global.body.actualPreviousPosition.lerp(sphere1.global.body.position, t);
            sphere2Pos = sphere2.global.body.actualPreviousPosition.lerp(sphere2.global.body.position, t);
            distanceSquared = sphere1Pos.subtract(sphere2Pos).magnitudeSquared();
            return Math.sqrt(distanceSquared) - (sphere1.radius + sphere2.radius);
        }.bind(this);
        var t = 1;
        for (var i = 0; i < this.binarySearchDepth; i++) {
            t = (minT + maxT) / 2;
            var result = binarySearch(t);
            if (result > 0) {
                minT = t;
            } else {
                maxT = t;
            }
        }

        t = maxT


        var isColliding = binarySearch(t) < 0;

        if (!isColliding) {
            return false;
        }
        var distanceTo = sphere1.global.body.position.distance(sphere2.global.body.position);

        var contact = new CollisionContact();
        contact.normal = sphere1Pos.subtract(sphere2Pos).normalizeInPlace();
        if (contact.normal.magnitudeSquared() == 0) {
            contact.normal = new Vector3(1, 0, 0);
        }
        contact.point = sphere1.global.body.position.add(sphere2.global.body.position).scale(0.5);

        contact.body1 = sphere1;
        contact.body2 = sphere2;
        var penetration = sphere1.radius + sphere2.radius - distanceTo

        contact.penetration = contact.normal.scale(penetration);

        this.addContact(contact);
        return;
    }

    handleSphereTerrain(sphere1, terrain1) {
        var heightmapSphereWidth = sphere1.radius * terrain1.inverseTerrainScale;
        var spherePos = null;
        var terrainPos = null;
        var relativePos = null;
        var heightmapPos = null;
        var min = null;
        var max = null;
        var binarySearch = function (t, getData = false) {
            spherePos = sphere1.global.body.previousPosition.lerp(sphere1.global.body.position, t);
            terrainPos = terrain1.global.body.previousPosition.lerp(terrain1.global.body.position, t);
            relativePos = terrain1.global.body.rotation.conjugate().multiplyVector3(spherePos.subtract(terrainPos));
            heightmapPos = terrain1.translateLocalToHeightmap(relativePos);
            if (heightmapPos.x <= -heightmapSphereWidth || heightmapPos.x >= terrain1.heightmaps.widthSegments + heightmapSphereWidth || heightmapPos.z <= -heightmapSphereWidth || heightmapPos.z >= terrain1.heightmaps.depthSegments + heightmapSphereWidth) {
                return 1;
            }
            var currentHeight = 0;
            var currentTriangle = terrain1.getTriangle(terrain1.heightmaps.top, heightmapPos);
            if (currentTriangle) {
                var currentHeight = relativePos.y - currentTriangle.getHeight(heightmapPos).y;
                if (currentHeight < sphere1.radius) {
                    return currentHeight - sphere1.radius;
                }
            }

            return 1;
        }
        var minT = 0;
        var maxT = 1;
        var t = 1;
        for (var i = 0; i < this.binarySearchDepth; i++) {
            t = (minT + maxT) / 2;
            var result = binarySearch(t);
            if (result > 0) {
                minT = t;
            } else {
                maxT = t;
            }
        }
        t = maxT;
        binarySearch(t);

        var currentHeight = 0;
        var currentTriangle = terrain1.getTriangle(terrain1.heightmaps.top, heightmapPos);
        if (currentTriangle) {
            var currentHeight = relativePos.y - currentTriangle.getHeight(heightmapPos).y;
            if (currentHeight < 0) {
                currentTriangle.a = terrain1.translateHeightmapToWorld(currentTriangle.a);
                currentTriangle.b = terrain1.translateHeightmapToWorld(currentTriangle.b);
                currentTriangle.c = terrain1.translateHeightmapToWorld(currentTriangle.c);
                var normal = currentTriangle.getNormal();
                var spherePos2 = sphere1.global.body.position;
                var intersection = currentTriangle.intersectsSphere(spherePos2);
                if (intersection) {
                    var contact = new CollisionContact();
                    contact.point = intersection;
                    contact.normal = normal;
                    contact.penetration = intersection.subtract(spherePos2);
                    contact.body1 = sphere1;
                    contact.body2 = terrain1;


                    this.addContact(contact);
                }
            }
        }

        var min = new Vector3(heightmapPos.x - heightmapSphereWidth - 1, 0, heightmapPos.z - heightmapSphereWidth - 1);
        var max = new Vector3(heightmapPos.x + heightmapSphereWidth + 1, 0, heightmapPos.z + heightmapSphereWidth + 1);

        for (var x = min.x; x <= max.x; x++) {
            for (var z = min.z; z <= max.z; z++) {
                var triangles = terrain1.getTrianglePair(terrain1.heightmaps.top, new Vector3(x, 0, z));
                if (!triangles) {
                    continue;
                }
                for (var t of triangles) {
                    t.a = terrain1.translateHeightmapToWorld(t.a);
                    t.b = terrain1.translateHeightmapToWorld(t.b);
                    t.c = terrain1.translateHeightmapToWorld(t.c);
                    spherePos2 = sphere1.global.body.position;
                    var intersection = t.intersectsSphere(spherePos2);
                    if (!intersection) {
                        continue;
                    }
                    var contact = new CollisionContact();
                    contact.point = intersection;
                    contact.penetration = sphere1.radius - contact.point.distance(spherePos2);
                    contact.normal = t.getNormal();//contact.point.subtract(spherePos2).normalizeInPlace();
                    if (contact.penetration <= 0) {
                        continue;
                    }

                    if (contact.normal.magnitudeSquared() == 0) {
                        contact.normal = new Vector3(1, 0, 0);
                    }
                    contact.body1 = sphere1;
                    contact.body2 = terrain1;

                    contact.penetration = contact.normal.scale(contact.penetration);
                    this.addContact(contact);
                }
            }
        }
    }

    handleTerrainPoint(terrain1, point1, manual = false) {
        var pointPos = point1.global.body.position;

        var pointPosPrev = point1.global.body.actualPreviousPosition;
        var translatedPointPos = terrain1.translateWorldToLocal(pointPos);
        var heightmapPos = terrain1.translateLocalToHeightmap(translatedPointPos);
        var translatedPointPosPrev = terrain1.translateWorldToLocal(pointPosPrev);
        var heightmapPosPrev = terrain1.clampToHeightmap(terrain1.translateLocalToHeightmap(translatedPointPosPrev));

        if (heightmapPos.x <= 0 || heightmapPos.x >= terrain1.heightmaps.widthSegments || heightmapPos.z <= 0 || heightmapPos.z >= terrain1.heightmaps.depthSegments) {
            return false;
        }

        var triangleTop = terrain1.getTriangle(terrain1.heightmaps.top, heightmapPos);
        var triangleBottom = terrain1.getTriangle(terrain1.heightmaps.bottom, heightmapPos);

        var triangle = new Triangle(triangleTop.a.add(triangleBottom.a).scaleInPlace(0.5), triangleTop.b.add(triangleBottom.b).scaleInPlace(0.5), triangleTop.c.add(triangleBottom.c).scaleInPlace(0.5));


        var height = 0;
        var top = true;
        var normal = new Vector3(1, 0, 0);
        var height1 = triangle.getHeight(heightmapPosPrev);
        var height2 = triangle.getHeight(heightmapPosPrev);
        // if(1==0 && heightmapPos.y > height1.y && heightmapPosPrev.y > height2.y){

        //     top = true;
        // }
        // else if(1==0 && heightmapPos.y < height1.y && heightmapPosPrev.y < height2.y){
        //     top = false;
        // }
        // else{
        //     var triangle2 = triangle.copy();
        //     triangle2.a = terrain1.translateHeightmapToWorld(triangle2.a);
        //     triangle2.b = terrain1.translateHeightmapToWorld(triangle2.b);
        //     triangle2.c = terrain1.translateHeightmapToWorld(triangle2.c);

        //     var velocity = point1.global.body.getVelocity();//pointPos.subtract(p);
        //     normal = triangle2.getNormal();
        //     var pointVelocity = velocity.dot(normal);
        //     if(pointVelocity > 0){
        //         //top = false;
        //     }
        // }

        if (top) {
            var height = terrain1.translateHeightmapToWorld(triangleTop.getHeight(heightmapPos));
            var triangle2 = triangleTop.copy();
            triangle2.a = terrain1.translateHeightmapToWorld(triangle2.a);
            triangle2.b = terrain1.translateHeightmapToWorld(triangle2.b);
            triangle2.c = terrain1.translateHeightmapToWorld(triangle2.c);
            var normal = triangle2.getNormal();
            var contact = new CollisionContact();
            contact.normal = normal;
            contact.penetration = triangle2.a.subtract(pointPos).dot(contact.normal);
            if (contact.penetration <= 0 && !manual) {
                return false;
            }
            console.log(triangle2.a.subtract(pointPos));
            contact.body1 = point1;
            contact.body2 = terrain1;
            contact.point = point1.global.body.position;
            contact.penetration = contact.normal.scale(contact.penetration);
            if (!manual) {
                this.addContact(contact);
            }
            return contact;
        }
        else {
            var height = terrain1.translateHeightmapToWorld(triangleBottom.getHeight(heightmapPos));
            if (pointPos.y > height.y) {
                //point1.translate(new Vector3(0, height.y - pointPos.y, 0));
            }
        }

        //return true;
        /*
        var height = terrain1.getHeightFromHeightmap(terrain1.heightmaps.top, point1.global.body.position.copy());
        if(height != null){
            if(point1.global.body.position.y < height.y){
                point1.global.body.position = height.copy();
            }
        }
        return true;*/
        return false;
    }

    toJSON() {
        return {
            binarySearchDepth: this.binarySearchDepth
        };
    }

    static fromJSON(json, world) {
        var collisionDetector = new CollisionDetector({
            world: world
        });
        collisionDetector.binarySearchDepth = json.binarySearchDepth;
        return collisionDetector;
    }
};


export default CollisionDetector;