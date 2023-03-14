@echo off
SET java_path=%CD%\jre\bin
SET java_lib=%CD%\jre\lib
path %java_path%;%java_lib%;%PATH%

jre\bin\java -cp "lib/RXTXcomm.jar;lib/LeapJava.jar;dolittle.jar" -Djava.library.path=lib o3.UI &
if %ERRORLEVEL% == 0 (
    goto end
)

if not exist "%SystemRoot%\System32\java.exe" goto wow64
if "%ProgramFiles(x86)%XXX"=="XXX" goto w32
goto w64


:wow64
jre\bin\java  -cp "lib/RXTXcomm.jar;lib/LeapJava.jar;dolittle.jar" -Djava.library.path=lib o3.UI &
goto end
:w32
jre\bin\java  -cp "lib/RXTXcomm.jar;lib/LeapJava.jar;dolittle.jar" -Djava.library.path=lib o3.UI &
goto end
:w64
jre\bin\java   -cp "lib64/RXTXcomm.jar;lib/LeapJava.jar;dolittle.jar" -Djava.library.path=lib64 o3.UI &
goto end


:end