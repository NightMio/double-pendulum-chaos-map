@echo off
cd /d %~dp0
gcc -O2 -march=native -mno-avx -mno-avx2 -fopenmp -municode ^
    chaosmap.c pendwnd.c cl_gpu.c -o ..\ChaosMap.exe ^
    -mwindows -lgdi32 -lcomctl32 -luser32 -lwinmm -lgdiplus -lole32 -luuid -lm
if errorlevel 1 (echo BUILD FAILED) else (echo BUILD OK)
pause
