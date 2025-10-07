const express = require('express');
const formidable = require('formidable').default;
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = 8082;

// Simulator-specific upload handlers
const simulatorHandlers = {
  gazebo: require('../library/common/fileUtils').handleGazeboUpload,
  coppeliasim: require('../library/common/fileUtils').handleCoppeliaUpload,
  // Add other simulators here in the future
  // unity: require('./library/unity/unityUpload').handleUnityUpload,
};

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
      return res.status(400).json({ 
        success: false,
        error: 'Error parsing the file upload',
        details: { timestamp: new Date().toISOString() }
      });
    }

    let target = fields.target;
    let simulator = fields.simulator;
    let file = files.file;

    if (Array.isArray(target)) target = target[0];
    if (Array.isArray(simulator)) simulator = simulator[0];
    if (Array.isArray(file)) file = file[0];

    if (!file) {
      console.warn('Rejected upload: missing file');
      return res.status(400).json({ 
        success: false,
        error: 'Missing file',
        details: { 
          requiredFields: ['file', 'simulator'],
          timestamp: new Date().toISOString()
        }
      });
    }

    // Simulator is mandatory
    if (!simulator) {
      console.warn('Rejected upload: missing simulator');
      return res.status(400).json({ 
        success: false,
        error: 'Simulator parameter is required',
        details: {
          supportedSimulators: Object.keys(simulatorHandlers),
          timestamp: new Date().toISOString()
        }
      });
    }

    // Validate simulator handler exists
    if (!simulatorHandlers[simulator]) {
      console.warn(`Rejected upload: unknown simulator '${simulator}'`);
      return res.status(400).json({ 
        success: false,
        error: `Unknown simulator '${simulator}'`,
        details: {
          providedSimulator: simulator,
          supportedSimulators: Object.keys(simulatorHandlers),
          timestamp: new Date().toISOString()
        }
      });
    }

    // Handle simulator-specific upload
    try {
      const result = await handleSimulatorUpload(simulator, file, target);
      
      // Clean up temporary file
      fs.unlink(file.filepath, (unlinkErr) => {
        if (unlinkErr) {
          console.warn('Failed to delete temporary file:', unlinkErr);
        }
      });

      const fileExtension = path.extname(file.originalFilename);
      const fileSize = file.size;
      const fileSizeKB = (fileSize / 1024).toFixed(2);

      console.log(`${simulator} file uploaded: ${result}`);
      res.json({ 
        success: true,
        message: result,
        details: {
          simulator: simulator,
          fileName: file.originalFilename,
          fileExtension: fileExtension,
          fileSize: `${fileSizeKB} KB`,
          uploadedAt: new Date().toISOString()
        }
      });
    } catch (error) {
      console.error(`${simulator} upload error:`, error);
      return res.status(500).json({ 
        success: false,
        error: error.message,
        details: {
          simulator: simulator,
          fileName: file?.originalFilename || 'unknown',
          timestamp: new Date().toISOString()
        }
      });
    }
  });
});

app.listen(PORT, () => {
  console.log(`File upload server running on http://localhost:${PORT}`);
});
