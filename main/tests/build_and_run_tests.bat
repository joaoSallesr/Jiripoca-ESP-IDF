@echo off
setlocal

:: 1. Entra na pasta do script
cd /d "%~dp0"

echo ====================================
echo  CONFIGURANDO E COMPILANDO TESTES
echo ====================================

:: Cria a pasta build e gera os arquivos do compilador (usando GCC/MinGW)
cmake -B build -G "MinGW Makefiles"

:: Se a configuração falhar, para o script
if %ERRORLEVEL% NEQ 0 (
    echo [ERRO] Falha ao configurar o CMake.
    pause
    exit /b %ERRORLEVEL%
)

:: Compila todos os executaveis dentro da pasta build
cmake --build build

:: Se a compilação falhar, para o script
if %ERRORLEVEL% NEQ 0 (
    echo [ERRO] Falha na compilacao!
    pause
    exit /b %ERRORLEVEL%
)

echo ====================================
echo  RODANDO OS TESTES
echo ====================================

:: Executa os testes gerados
echo.
echo --- Executando Test Baro ---
build\test_baro.exe
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

echo.
echo --- Executando Test Accel ---
build\test_accel.exe
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

echo.
echo --- Executando Test Boost ---
build\test_boost.exe
if %ERRORLEVEL% NEQ 0 exit /b %ERRORLEVEL%

echo.
echo ====================================
echo  FIM DA EXECUCAO
echo ====================================
pause