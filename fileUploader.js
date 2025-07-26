const express = require('express');
const formidable = require('formidable').default;
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = 8081;

const uploadDirs = {
  world: path.join(process.env.HOME, "wos/upload", "world"),
  model: path.join(process.env.HOME, "wos/upload", "model"),
  launch: path.join(process.env.HOME, "wos/upload", "launch")
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

    fs.rename(oldPath, newPath, (err) => {
      if (err) {
        console.error('File move error:', err);
        return res.status(500).json({ error: 'File move failed' });
      }

      console.log(`File uploaded to: ${newPath}`);
      res.json({ message: 'File uploaded successfully', path: newPath });
    });
  });
});

app.listen(PORT, () => {
  console.log(`File upload server running on http://localhost:${PORT}`);
});
