const fs = require("fs/promises");
const path = require("path");

const BASE_PATH = "/project-root";
const folderGroups = ["resource", "upload", "saved"];
const folderTypes = ["world", "model", "launch"];

/**
 * List available files under resource and upload folders.
 */
async function readAvailableResources() {
    try {
        const result = {};
        for (const group of folderGroups) {
            result[group] = {};
            for (const type of folderTypes) {
                const folderPath = path.join(BASE_PATH, group, type);
                try {
                    const files = await fs.readdir(folderPath);
                    result[group][type] = files;
                } catch (err) {
                    result[group][type] = [];
                }
            }
        }
        return result;
    } catch (err) {
        console.error('[availableResources read]', err);
        return `Failed to read available resources: ${err.message || String(err)}`;
    }
}

async function saveFile(filePath, data) {
    try {
        await fs.mkdir(path.dirname(filePath), { recursive: true });
        const buffer = Buffer.from(data, 'utf8');
        await fs.writeFile(filePath, buffer);
        // console.log("✅ File written:", filePath);
        return filePath;
    } catch (err) {
        console.error("❌ Failed to write file:", err.message);
        throw new Error("File save failed");
    }
}

async function handleUploadFile(input) {
    const data = await input.value();
    const { name, content, target } = data;

    if (!name || !content || !target || !folderTypes.includes(target)) {
        throw new Error("Invalid upload parameters");
    }

    const filePath = path.join(BASE_PATH, 'upload', target, name);
    await saveFile(filePath, content);
    return `File '${name}' uploaded to '${filePath}'`;
}



async function resolveFilePath(fileName) {
    const searchPaths = [
        ["resource", "world"],
        ["resource", "model"],
        ["resource", "model", "ur_description"],
        ["resource", "launch"],
        ["upload", "world"],
        ["upload", "model"],
        ["upload", "launch"],
        ["saved", "world"]
    ];

    for (const pathParts of searchPaths) {
        const fullPath = path.join(BASE_PATH, ...pathParts, fileName);
        try {
            const stat = await fs.stat(fullPath);
            if (stat.isFile()) {
                return fullPath;
            }
        } catch {
            // Silently skip on error
        }
    }
    console.warn("File NOT FOUND:", fileName);
    return null;
}

module.exports = {
    readAvailableResources,
    saveFile,
    handleUploadFile,
    resolveFilePath
};
