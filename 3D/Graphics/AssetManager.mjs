

var AssetManager = class {

    constructor(options) {
        this.assets = new Map();
        this.loader = options?.loader ?? null;
        this.extraLoaders = options?.extraLoaders ?? {};
        this.texturesDirectory = options?.texturesDirectory ?? new URL('.', import.meta.url).href + "Textures/";
        this.modelsDirectory = options?.modelsDirectory ?? new URL('.', import.meta.url).href + "Models/";
    }

    resolvePath(path) {
        if (path.startsWith('http://') || path.startsWith('https://') || path.startsWith('file://') || path.startsWith('./')) {
            return path;
        }
        return new URL(path, this.texturesDirectory).href;
    }

    load(name, file, specialFormat = null) {
        var path = this.resolvePath(file);
        if (specialFormat) {
            for (var extension in this.extraLoaders) {
                if (specialFormat == extension) {
                    this.textures.set(name, this.extraLoaders[extension].load(path));
                    return this.textures.get(name);
                }
            }
        }
        this.assets.set(name, {
            name: name,
            path: file,
            data: this.loader.load(path),
            type: specialFormat
        });
        return this.assets.get(name);
    }

    loadAll(textures) {
        for (var txt of textures) {
            this.load(txt.name, txt.file, txt.specialFormat ?? null);
        }
    }

    get(name) {
        return this.assets.get(name);
    }
}


export default AssetManager;