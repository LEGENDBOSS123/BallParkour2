import Vector3 from "../Math3D/Vector3.mjs";
import DistanceConstraint from "./DistanceConstraint.mjs";
import CollisionContact from "./CollisionContact.mjs";
import Triangle from "../Shapes/Triangle.mjs";
import Composite from "../Shapes/Composite.mjs";
import ClassRegistry from "../Core/ClassRegistry.mjs";
var CollisionDetector = class {

    static seperatorCharacter = ":";

    constructor(options) {
        this.pairs = options?.pairs ?? new Map();
        this.world = options?.world ?? null;
        this.contacts = options?.contacts ?? [];
        this.handlers = {};
        this.binarySearchDepth = options?.binarySearchDepth ?? 4;
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
        if (!(this.handlers[shape1.type]?.[shape2.type] || this.handlers[shape2.type]?.[shape1.type])) {
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
        if (shape1.type > shape2.type) {
            var temp = shape1;
            shape1 = shape2;
            shape2 = temp;
        }
        return this.handlers[shape1.type]?.[shape2.type]?.bind(this)(shape1, shape2);
    }

    initHandlers() {
        this.handlers[ClassRegistry.getTypeFromName("SPHERE")] = {};
        this.handlers[ClassRegistry.getTypeFromName("SPHERE")][ClassRegistry.getTypeFromName("SPHERE")] = this.handleSphereSphere;
        this.handlers[ClassRegistry.getTypeFromName("SPHERE")][ClassRegistry.getTypeFromName("TERRAIN3")] = this.handleSphereTerrain;
        this.handlers[ClassRegistry.getTypeFromName("SPHERE")][ClassRegistry.getTypeFromName("BOX")] = this.handleSphereBox;
        this.handlers[ClassRegistry.getTypeFromName("SPHERE")][ClassRegistry.getTypeFromName("POLYHEDRON")] = this.handleSpherePolyhedron;
        this.handlers[ClassRegistry.getTypeFromName("TERRAIN3")] = {};
        this.handlers[ClassRegistry.getTypeFromName("TERRAIN3")][ClassRegistry.getTypeFromName("POINT")] = this.handleTerrainPoint;
        this.handlers[ClassRegistry.getTypeFromName("BOX")] = {};
        top.ClassRegistry = ClassRegistry;
    }

    handle(shape) {
        var func = function (x) {
            this.addPair(shape, this.world.getByID(x));
            return false;
        }.bind(this);
        this.world.spatialHash.query(shape.id, func);
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
            var massRatio1 = contact.body1.maxParent.getEffectiveTotalMass() / totalMass;
            massRatio1 = isNaN(massRatio1) ? 1 : massRatio1;


            if (contact.body1Map.penetrationSum != 0) {
                contact.body1.translate(translation.scale(contact.penetration.magnitude() / contact.body1Map.penetrationSum * massRatio2));
            }
            if (contact.body2Map.penetrationSum != 0) {
                contact.body2.translate(translation.scale(-contact.penetration.magnitude() / contact.body2Map.penetrationSum * massRatio1));
            }

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
    closestPointOnTriangle(p, a, b, c) {
        let ab = b.subtract(a);
        let ac = c.subtract(a);
        let ap = p.subtract(a);

        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);

        if (d1 <= 0 && d2 <= 0) return a;

        let bp = p.subtract(b);
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if (d3 >= 0 && d4 <= d3) return b

        let cp = p.subtract(c);
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);
        if (d6 >= 0 && d5 <= d6) return c;

        let vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) {
            let v = d1 / (d1 - d3);
            return a.add(ab.scale(v));
        }

        let vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) {
            let w = d2 / (d2 - d6);
            return a.add(ac.scale(w));
        }

        let va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b.add(c.subtract(b).scale(w));
        }

        let denom = 1 / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        return a.add(ab.scale(v)).add(ac.scale(w));
    }
    handleSpherePolyhedron(sphere, poly) {
        var spherePos = null;
        var localSpherePos = null;
        var closestPoint = null;
        var minDistanceSquared = Infinity;
        var closestFace = null;
        var polyPos = null;
        var relativePos = null;
        var inside = 0;
        var inside2 = 0;
        var minT = 0;
        var maxT = 1;
        var binarySearch = function (t) {
            spherePos = sphere.global.body.previousPosition.lerp(sphere.global.body.position, t);
            polyPos = poly.global.body.previousPosition.lerp(poly.global.body.position, t);
            relativePos = poly.global.body.rotation.conjugate().multiplyVector3(spherePos.subtract(polyPos));
            closestPoint = null;
            closestFace = null;
            minDistanceSquared = Infinity;
            inside = 0;
            inside2 = 0;
            var min = new Vector3();
            var max = new Vector3();
            var minS = relativePos.subtract(new Vector3(1, 1, 1).scale(sphere.radius));
            var maxS = relativePos.add(new Vector3(1, 1, 1).scale(sphere.radius));
            for (var face of poly.faces) {
                var a = poly.localVertices[face[0]];
                var b = poly.localVertices[face[1]];
                var c = poly.localVertices[face[2]];
                min.x = Math.min(a.x, b.x, c.x);
                max.x = Math.max(a.x, b.x, c.x);

                min.y = Math.min(a.y, b.y, c.y);
                max.y = Math.max(a.y, b.y, c.y);

                min.z = Math.min(a.z, b.z, c.z);
                max.z = Math.max(a.z, b.z, c.z);


                if (!(min.x <= maxS.x && max.x >= minS.x && min.y <= maxS.y && max.y >= minS.y && min.z <= maxS.z && max.z >= minS.z)) {
                    continue;
                }
                
                var normal = b.subtract(a).cross(c.subtract(a));
                var closest = this.closestPointOnTriangle(relativePos, a, b, c);
                inside2++;
                if (relativePos.subtract(a).dot(normal) < 0) {
                    inside++;
                }
                var distSq = closest.subtract(relativePos).magnitudeSquared();
                if (distSq < minDistanceSquared) {
                    minDistanceSquared = distSq;
                    closestPoint = closest;
                    closestFace = face;
                }
            }
            return minDistanceSquared - sphere.radius * sphere.radius;
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
        if (binarySearch(t) > 0 && inside != poly.faces.length) {
            return false;
        }


        var closestPoint2 = poly.global.body.rotation.multiplyVector3(closestPoint).addInPlace(polyPos);
        var contact = new CollisionContact();
        contact.point = poly.translateLocalToWorld(closestPoint);
        contact.normal = spherePos.subtract(closestPoint2).normalizeInPlace();
        if (inside == inside2) {
            contact.normal.scaleInPlace(-1);
        }
        contact.penetration = contact.normal.scale(sphere.radius).add(contact.point.subtract(sphere.global.body.position).projectOnto(contact.normal));
        contact.body1 = sphere;
        contact.body2 = poly;
        contact.point = sphere.global.body.position.subtract(contact.normal.scale(sphere.radius));
        this.addContact(contact);
        return true;

    }

    handleSphereBox(sphere1, box1) {
        var spherePos = sphere1.global.body.position;
        var dimensions = new Vector3(box1.width, box1.height, box1.depth).scale(0.5);
        var relativePos = box1.translateWorldToLocal(spherePos);

        var prevPos = box1.global.body.rotation.conjugate().multiplyVector3(sphere1.global.body.previousPosition.subtract(box1.global.body.previousPosition));

        var delta = relativePos.subtract(prevPos);


        var minT = 0;
        var maxT = 1;
        var dimensions = new Vector3(box1.width, box1.height, box1.depth).scale(0.5);
        var binarySearch = function (t, getData = false) {
            spherePos = sphere1.global.body.previousPosition.lerp(sphere1.global.body.position, t);
            var boxPos = box1.global.body.previousPosition.lerp(box1.global.body.position, t);
            relativePos = box1.global.body.rotation.conjugate().multiplyVector3(spherePos.subtract(boxPos));

            var closest = this.getClosestPointToAABB(relativePos.copy(), box1);
            var distanceSquared = closest.subtract(relativePos).magnitudeSquared();

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
        if (binarySearch(t) > 0) {
            return false;
        }
        if (t !== null) {

            var pos = prevPos.addInPlace(delta.scale(t - 0.00001));
            var closest = this.getClosestPointToAABB(pos.copy(), box1);
            if (pos.distanceSquared(closest) > 0 && pos.distanceSquared(closest) < sphere1.radius * sphere1.radius) {
                var contact = new CollisionContact();

                var normal = pos.subtract(closest).normalize();

                contact.point = box1.translateLocalToWorld(closest);
                contact.normal = box1.global.body.rotation.multiplyVector3(normal);
                contact.penetration = contact.normal.scale(sphere1.radius).add(contact.point.subtract(sphere1.global.body.position).projectOnto(contact.normal));
                contact.body1 = sphere1;
                contact.body2 = box1;
                contact.point = sphere1.global.body.position.subtract(contact.normal.scale(sphere1.radius));
                this.addContact(contact);
                return true;

            }

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
            sphere1Pos = sphere1.global.body.previousPosition.lerp(sphere1.global.body.position, t);
            sphere2Pos = sphere2.global.body.previousPosition.lerp(sphere2.global.body.position, t);
            distanceSquared = sphere1Pos.subtract(sphere2Pos).magnitudeSquared();
            return distanceSquared - (sphere1.radius + sphere2.radius) * (sphere1.radius + sphere2.radius);
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
        var penetration = sphere1.radius + sphere2.radius - distanceTo;

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

        var pointPosPrev = point1.global.body.previousPosition;
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