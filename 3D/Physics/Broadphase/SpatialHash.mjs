import Vector3 from "../Math3D/Vector3.mjs";
import Hitbox3 from "../Broadphase/Hitbox3.mjs";

var SpatialHash = class {
    static hashConstants = [4917, 2933, 9893, 512];
    constructor(options) {
        this.world = options?.world ?? null;
        this.spatialHashes = [];
        for (var i = 0; i < (options?.gridSizes?.length ?? 16); i++) {
            var spatialHash = {};
            spatialHash.hashmap = new Map();
            spatialHash.gridSize = options?.gridSizes?.[i] ?? Math.pow(2, i) * 0.25;
            spatialHash.inverseGridSize = 1 / spatialHash.gridSize;
            spatialHash.threshold = options?.thresholds?.[i] ?? 2;
            spatialHash.translation = new Vector3();
            spatialHash.index = i;
            if(spatialHash.index % 2 == 0){
                spatialHash.translation = new Vector3(spatialHash.gridSize * 0.5, spatialHash.gridSize * 0.5, spatialHash.gridSize * 0.5);
            }
            
            this.spatialHashes.push(spatialHash);
        }
        for (var i = 0; i < this.spatialHashes.length - 1; i++) {
            this.spatialHashes[i].next = this.spatialHashes[i + 1];
            this.spatialHashes[i].final = false;
        }
        this.global = new Set();
        this.spatialHashes.push({ final: true, hashmap: this.global, next: null, index: this.spatialHashes.length });
        this.spatialHashes[this.spatialHashes.length - 2].next = this.spatialHashes[this.spatialHashes.length - 1];
        this.ids = {};
    }

    hash(cellPos) {
        return ((cellPos.x * this.constructor.hashConstants[0]) ^ (cellPos.y * this.constructor.hashConstants[1]) ^ (cellPos.z * this.constructor.hashConstants[2])) % this.constructor.hashConstants[3];
    }

    remove(id) {
        this.removeHitbox(id);
        delete this.ids[id];
    }

    getCellPosition(v, hash) {
        var v1 = v.add(hash.translation);
        return new Vector3(Math.floor(v1.x * hash.inverseGridSize), Math.floor(v1.y * hash.inverseGridSize), Math.floor(v1.z * hash.inverseGridSize));
    }

    getSizeHeuristic(min, max) {
        return (max.x - min.x + 1) * (max.y - min.y + 1) * (max.z - min.z + 1);
    }

    _addHitbox(hitbox, id, hash = this.spatialHashes[0]) {

        if (hash.final) {
            hash.hashmap.add(id);
            this.ids[id].hash = hash;
            return true;
        }

        var min = this.getCellPosition(hitbox.min, hash);
        var max = this.getCellPosition(hitbox.max, hash);
        if (this.getSizeHeuristic(min, max) > hash.threshold) {
            return this._addHitbox(hitbox, id, hash.next);
        }
        this.ids[id].hash = hash;
        var v = min.copy();
        for (v.x = min.x; v.x <= max.x; v.x++) {
            for (v.y = min.y; v.y <= max.y; v.y++) {
                for (v.z = min.z; v.z <= max.z; v.z++) {
                    var cell = this.hash(v);
                    this.addToCell(cell, id, hash);
                }
            }
        }
    }

    addHitbox(hitbox, id) {
        if (this.ids[id]) {
            if (this.ids[id].hitbox.equals(hitbox)) {
                return;
            }
            this.removeHitbox(id);
        }
        this.ids[id] = {};
        this.ids[id].hitbox = hitbox.copy();
        this._addHitbox(hitbox, id, this.spatialHashes[0]);
    }

    _removeHitbox(hitbox, id, hash = this.spatialHashes[0]) {
        if (hash.final) {
            hash.hashmap.delete(id);
            return true;
        }
        var min = this.getCellPosition(hitbox.min, hash);
        var max = this.getCellPosition(hitbox.max, hash);
        if (this.getSizeHeuristic(min, max) > hash.threshold) {
            return this._removeHitbox(hitbox, id, hash.next);
        }
        var v = min.copy();
        for (v.x = min.x; v.x <= max.x; v.x++) {
            for (v.y = min.y; v.y <= max.y; v.y++) {
                for (v.z = min.z; v.z <= max.z; v.z++) {
                    var cell = this.hash(v);
                    this.removeFromCell(cell, id, hash);
                }
            }
        }
    }

    removeHitbox(id) {
        if (!this.ids[id]) {
            return;
        }
        this._removeHitbox(this.ids[id].hitbox, id, this.ids[id].hash);
    }

    removeFromCell(cell, id, hash) {
        var map = hash.hashmap.get(cell);
        if (!map) {
            return false;
        }
        var index = map.indexOf(id);
        if (index == -1) {
            return false;
        }
        map.splice(index, 1);
        if (map.length == 0) {
            hash.hashmap.delete(cell);
        }
        return true;
    }

    addToCell(cell, id, hash) {
        var map = hash.hashmap.get(cell);
        if (!map) {
            map = [];
            hash.hashmap.set(cell, map);
        }
        map.push(id);
        return true;
    }

    _query(hitbox, result = new Set(), hash = this.spatialHashes[0]) {
        if (hash.final) {
            if (hash.hashmap.size == 0) {
                return result;
            }
            for (var i of hash.hashmap) {
                result.add(i);
            }
            return [...(new Set(result))];
        }
        var min = this.getCellPosition(hitbox.min, hash);
        var max = this.getCellPosition(hitbox.max, hash);
        var v = min.copy();
        var map = null;
        var cell = null;
        if (hash.hashmap.size > 0) {
            for (v.x = min.x; v.x <= max.x; v.x++) {
                for (v.y = min.y; v.y <= max.y; v.y++) {
                    for (v.z = min.z; v.z <= max.z; v.z++) {
                        cell = this.hash(v);
                        map = hash.hashmap.get(cell);
                        if (map) {
                            for (var i = 0; i < map.length; i++) {
                                result.add(map[i]);
                            }
                        }
                    }
                }
            }
        }
        return this._query(hitbox, result, hash.next);
    }

    query(id) {
        if (!this.ids[id]) {
            return [];
        }
        return this._query(this.ids[id].hitbox, new Set(), this.ids[id].hash);
    }

    toJSON() {
        var spatialHash = {};

        spatialHash.world = this.world.id;
        spatialHash.spatialHashes = [];
        spatialHash.global = new Set(this.global);
        for (var i = 0; i < this.spatialHashes.length; i++) {
            var hash = {};
            hash.gridSize = this.spatialHashes[i].gridSize;
            hash.inverseGridSize = this.spatialHashes[i].inverseGridSize;
            hash.threshold = this.spatialHashes[i].threshold;
            hash.translation = this.spatialHashes[i].translation.toJSON();
            hash.final = this.spatialHashes[i].final;
            hash.index = this.spatialHashes[i].index;
            if (!this.spatialHashes[i].final) {
                hash.next = i + 1;
                hash.hashmap = new Map(this.spatialHashes[i].hashmap);
            }
            else {
                hash.next = null;
                hash.hashmap = spatialHash.global;
            }
            spatialHash.spatialHashes.push(hash);
        }
        spatialHash.ids = {};
        for (var i in this.ids) {
            spatialHash.ids[i] = {};
            spatialHash.ids[i].hitbox = this.ids[i].hitbox.toJSON();
            spatialHash.ids[i].hash = this.ids[i].hash.index;
        }
        return spatialHash;
    }

    static fromJSON(json, world) {
        var spatialHash = new SpatialHash();
        spatialHash.world = world;
        spatialHash.global = json.global;
        spatialHash.spatialHashes = json.spatialHashes;
        for (var i = 0; i < spatialHash.spatialHashes.length; i++) {
            var hash = spatialHash.spatialHashes[i];
            hash.translation = Vector3.fromJSON(hash.translation);
            if (!hash.final) {
                hash.next = spatialHash.spatialHashes[hash.next];
            }
        }
        spatialHash.ids = json.ids;
        for (var i in spatialHash.ids) {
            spatialHash.ids[i].hitbox = Hitbox3.fromJSON(spatialHash.ids[i].hitbox);
            spatialHash.ids[i].hash = spatialHash.spatialHashes[spatialHash.ids[i].hash];
        }
        return spatialHash;
    }
};


export default SpatialHash;