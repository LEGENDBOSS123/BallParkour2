var Keysheld = class {

    constructor(element) {
        this.keysheld = {};

        element.addEventListener('keydown', function (e) {
            this.keysheld[e.code] = true;
        }.bind(this));
        element.addEventListener('keyup', function (e) {
            this.keysheld[e.code] = false;
        }.bind(this));
    }

    isHeld(key) {
        return this.keysheld[key] || false;
    }
}


export default Keysheld;