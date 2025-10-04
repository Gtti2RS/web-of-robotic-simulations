const express = require('express');
const formidable = require('formidable').default;
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = 8082;

// Generic upload directories (fallback for non-simulator uploads)
const uploadDirs = {
  world: path.join("/project-root", "upload", "world"),
  model: path.join("/project-root", "upload", "model"),
  launch: path.join("/project-root", "upload", "launch")
};

// Simulator-specific upload handlers
const simulatorHandlers = {
  gazebo: require('./library/common/fileUtils').handleGazeboUpload,
  // Add other simulators here in the future
  // coppeliasim: require('./library/coppeliasim/coppeliaUpload').handleCoppeliaUpload,
  // unity: require('./library/unity/unityUpload').handleUnityUpload,
};

// Ensure upload folders exist
Object.values(uploadDirs).forEach(dir => {
  if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
});

/**
 * Handle simulator-specific uploads
 * @param {string} simulator - The simulator name (e.g., 'gazebo')
 * @param {Object} file - The uploaded file object from formidable
 * @param {string} target - The target type (e.g., 'model', 'world', 'launch')
 * @returns {Promise<string>} - Success message
 */
async function handleSimulatorUpload(simulator, file, target) {
  const handler = simulatorHandlers[simulator];
  if (!handler) {
    throw new Error(`No handler found for simulator: ${simulator}`);
  }

  // Read file content
  const fileContent = fs.readFileSync(file.filepath, 'utf8');
  
  // Create mock input object for the handler
  const mockInput = {
    value: () => Promise.resolve({
      name: file.originalFilename,
      content: fileContent,
      target: target
    })
  };

  // Call the simulator-specific handler
  return await handler(mockInput);
}

app.post('/upload', (req, res) => {
  const form = formidable({
    multiples: false,
    keepExtensions: true,
  });

  form.parse(req, async (err, fields, files) => {
    console.log('Received fields:', fields);
    console.log('Received files:', files);

    if (err) {
      console.error('Form parse error:', err);
      return res.status(400).json({ error: 'Error parsing the file upload' });
    }

    let target = fields.target;
    let simulator = fields.simulator;
    let file = files.file;

    if (Array.isArray(target)) target = target[0];
    if (Array.isArray(simulator)) simulator = simulator[0];
    if (Array.isArray(file)) file = file[0];

    if (!file || typeof target !== 'string' || !(target in uploadDirs)) {
      console.warn('Rejected upload:', { target, file });
      return res.status(400).json({ error: 'Invalid target or missing file' });
    }

    // Handle simulator-specific uploads
    if (simulator && simulatorHandlers[simulator]) {
      try {
        const result = await handleSimulatorUpload(simulator, file, target);
        
        // Clean up temporary file
        fs.unlink(file.filepath, (unlinkErr) => {
          if (unlinkErr) {
            console.warn('Failed to delete temporary file:', unlinkErr);
          }
        });

        console.log(`${simulator} file uploaded: ${result}`);
        res.json({ message: result, simulator: simulator });
        return;
      } catch (error) {
        console.error(`${simulator} upload error:`, error);
        return res.status(500).json({ error: `${simulator} upload failed: ${error.message}` });
      }
    }

    // Default upload behavior (generic file upload)
    const oldPath = file.filepath;
    const newPath = path.join(uploadDirs[target], file.originalFilename);

    // Use copyFile + unlink instead of rename to handle cross-device moves
    fs.copyFile(oldPath, newPath, (err) => {
      if (err) {
        console.error('File copy error:', err);
        return res.status(500).json({ error: 'File copy failed' });
      }

      // Delete the temporary file after successful copy
      fs.unlink(oldPath, (unlinkErr) => {
        if (unlinkErr) {
          console.warn('Failed to delete temporary file:', unlinkErr);
        }
      });

      console.log(`File uploaded to: ${newPath}`);
      res.json({ message: 'File uploaded successfully', path: newPath });
    });
  });
});

app.listen(PORT, () => {
  console.log(`File upload server running on http://localhost:${PORT}`);
});
