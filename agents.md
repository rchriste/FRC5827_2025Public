# Agents / Build Instructions

To build this project from a clean PowerShell session on Windows, run:

```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2025\jdk"
$env:PATH="$env:JAVA_HOME\bin;$env:PATH"
./gradlew.bat build
```

This sets the WPILib 2025 JDK as the active Java runtime for Gradle, then runs the standard build.
