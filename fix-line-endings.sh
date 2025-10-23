#!/bin/bash
# Emergency line ending fix for Windows Docker
# Only run this if Docker automatic fixes don't work

echo "Emergency line ending fix..."
find . -name "*.sh" -type f -exec sed -i 's/\r$//' {} \;
echo "Done! All shell scripts now have LF line endings."
