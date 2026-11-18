@echo off
setlocal

:: 1. Entra na pasta do script
cd /d "%~dp0"

echo ====================================
echo  CONFIGURANDO E COMPILANDO TESTES
echo ====================================

:: Cria a pasta build e gera os arquivos do compilador (usando GCC/MinGW)
echo [CONFIGURE] CMake
cmake -B build -G "MinGW Makefiles"

:: Se a configuração falhar, para o script
if %ERRORLEVEL% NEQ 0 (
    echo [ERRO] Falha ao configurar o CMake.
    pause
    exit /b %ERRORLEVEL%
)
echo [CONFIGURE OK] CMake

:: Compile each executable separately so build failures identify the test.
echo.
echo [BUILD] test_baro
cmake --build build --target test_baro
if errorlevel 1 goto build_baro_failed
echo [BUILD OK] test_baro

echo.
echo [BUILD] test_accel
cmake --build build --target test_accel
if errorlevel 1 goto build_accel_failed
echo [BUILD OK] test_accel

echo.
echo [BUILD] test_gyro
cmake --build build --target test_gyro
if errorlevel 1 goto build_gyro_failed
echo [BUILD OK] test_gyro

echo ====================================
echo  RODANDO OS TESTES
echo ====================================

echo.
echo [RUN] test_baro
build\test_baro.exe
if errorlevel 1 goto run_baro_failed
echo [RUN OK] test_baro

echo.
echo [RUN] test_accel
build\test_accel.exe
if errorlevel 1 goto run_accel_failed
echo [RUN OK] test_accel

echo.
echo [RUN] test_gyro
build\test_gyro.exe
if errorlevel 1 goto run_gyro_failed
echo [RUN OK] test_gyro

echo.
echo ====================================
echo  FIM DA EXECUCAO
echo ====================================
pause
exit /b 0

:build_baro_failed
echo [BUILD FAILED] test_baro
exit /b 1

:build_accel_failed
echo [BUILD FAILED] test_accel
exit /b 1

:build_gyro_failed
echo [BUILD FAILED] test_gyro
exit /b 1

:run_baro_failed
echo [RUN FAILED] test_baro with exit code %ERRORLEVEL%
exit /b %ERRORLEVEL%

:run_accel_failed
echo [RUN FAILED] test_accel with exit code %ERRORLEVEL%
exit /b %ERRORLEVEL%

:run_gyro_failed
echo [RUN FAILED] test_gyro with exit code %ERRORLEVEL%
exit /b %ERRORLEVEL%
