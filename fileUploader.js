const express = require('express');
const formidable = require('formidable').default;
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = 8082;

const uploadDirs = {
  world: path.join("/project-root", "upload", "world"),
  model: path.join("/project-root", "upload", "model"),
  launch: path.join("/project-root", "upload", "launch")
};

// Ensure upload folders exist
Object.values(uploadDirs).forEach(dir => {
  if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
});

app.post('/upload', (req, res) => {
  const form = formidable({
    multiples: false,
    keepExtensions: true,
  });

  form.parse(req, (err, fields, files) => {
    console.log('Received fields:', fields);
    console.log('Received files:', files);

    if (err) {
      console.error('Form parse error:', err);
      return res.status(400).json({ error: 'Error parsing the file upload' });
    }

    let target = fields.target;
    let file = files.file;

    if (Array.isArray(target)) target = target[0];
    if (Array.isArray(file)) file = file[0];

    if (!file || typeof target !== 'string' || !(target in uploadDirs)) {
      console.warn('Rejected upload:', { target, file });
      return res.status(400).json({ error: 'Invalid target or missing file' });
    }


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
