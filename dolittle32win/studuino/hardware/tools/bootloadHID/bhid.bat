@echo off
setlocal enabledelayedexpansion
for /l %%n in (1, 1, 10) do (
   rem Waiting for 1s.
   ping 1.1.1.1 -n 1 -w 500 > nul

   set RES=
   for /f "delims=" %%i in ('%~dp0\bhid.exe -r %1 2^>^&1') do (
     echo %%i
      set RES=%%i
   )

   if "!RES:~0,5!" neq "Error" (
      echo Successfully finished.
      rem goto break
      exit /b 0
   )

)
echo !RES! 1>&2
exit /b 1
rem :break
endlocal
