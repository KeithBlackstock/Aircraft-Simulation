@echo off
ECHO Starting backend and frontend servers in separate windows...

:: Start Uvicorn backend server with --reload for development
START "Backend Server (Uvicorn)" uvicorn server:app --reload

:: Start Python's simple HTTP server for frontend files on port 8001
START "Frontend Server (Python)" python -m http.server 8001

ECHO Both servers have been launched.