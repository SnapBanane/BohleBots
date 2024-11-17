@echo off
for /r %%i in (._*) do (
    echo Deleting: %%i
    del "%%i"
)
echo Done!
pause
