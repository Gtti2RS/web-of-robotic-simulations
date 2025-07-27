const fs = require("fs/promises");
const path = require("path");
const os = require("os");

const BASE_PATH = path.join(os.homedir(), "wos");
const folderGroups = ["resource", "upload"];
const folderTypes = ["world", "model", "launch"];

/**
 * List available files under resource and upload folders.
 */
async function readAvailableResources() {
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
}

/**
 * Handle uploadFile WoT action.
 */
async function handleUploadFile(input) {
    const data = await input.value();
    const { name, content, target } = data;

    if (!name || !content || !target || !folderTypes.includes(target)) {
        throw new Error("Invalid upload parameters");
    }

    const filePath = path.join(BASE_PATH, "upload", target, name);
    const buffer = Buffer.from(content, "utf8");

    try {
        await fs.mkdir(path.dirname(filePath), { recursive: true });
        await fs.writeFile(filePath, buffer);
        console.log("✅ File written:", filePath);
        return `File '${name}' uploaded to '${target}'`;
    } catch (err) {
        console.error("❌ Failed to write file:", err.message);
        throw new Error("File upload failed");
    }
}

async function resolveFilePath(fileName) {
    const searchPaths = [
        ["resource", "world"],
        ["resource", "model"],
        ["resource", "launch"],
        ["upload", "world"],
        ["upload", "model"],
        ["upload", "launch"]
    ];

    for (const [group, subdir] of searchPaths) {
        const fullPath = path.join(BASE_PATH, group, subdir, fileName);
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
    handleUploadFile,
    resolveFilePath
};
