@echo off
del .\Objects
set PROJECT_NAME=adt3102_hare_people_counting
echo %PROJECT_NAME%
set UV=C:\Keil_v5\UV4\UV4.exe
set UV_PRO_PATH=%PROJECT_NAME%.uvprojx
echo Init building...
echo %UV_PRO_PATH%
%UV% -b -X -j0 -r %UV_PRO_PATH% -o build_log
copy .\Objects\%PROJECT_NAME%.lib ..\src\