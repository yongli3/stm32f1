::@echo off
rem ��������...
rem ɾ���ļ�
for /f "delims=" %%i in ('dir /b /a-d /s "*.sfr","*.dep","*.tmp"') do del /q "%%i"
for %%b in ("%cd%") do cd /d %%b&for /r %%c in ("Debug","settings", "hello-world", "process-post-test", "timer-test") do if exist %%c rmdir /s/q  "%%c"
rem ɾ�����
pause
