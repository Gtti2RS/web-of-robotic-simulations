@echo off
REM Emergency line ending fix for Windows Docker
REM Only run this if Docker automatic fixes don't work

echo Emergency line ending fix...

REM Use PowerShell to fix line endings
powershell -Command "Get-ChildItem -Recurse -Filter '*.sh' | ForEach-Object { (Get-Content $_.FullName -Raw) -replace \"`r`n\", \"`n\" | Set-Content $_.FullName -NoNewline }"

echo Done! All shell scripts now have LF line endings.
