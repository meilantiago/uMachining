@echo off
rem Instalador de uMachining para Windows: doble clic en este archivo.
chcp 65001 >nul
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 instalar.py %*
) else (
    where python >nul 2>nul
    if errorlevel 1 (
        echo No se encontro Python. Instalarlo desde https://www.python.org/downloads/
        echo y marcar "Add python.exe to PATH" en el instalador.
    ) else (
        python instalar.py %*
    )
)
echo.
pause
