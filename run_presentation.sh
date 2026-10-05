#!/usr/bin/env bash

# ==============================================================================
# GPS-Denied Autonomous Drone Exploration - Interactive Presentation Launcher
# ==============================================================================

PORT=8080
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="${SCRIPT_DIR}/presentation_app"

echo "========================================================================"
echo "🚀 Starting GPS-Denied Autonomous Drone Exploration Presentation System"
echo "========================================================================"
echo "📁 Presentation App Directory: ${APP_DIR}"
echo "🌐 Local Server: http://localhost:${PORT}"
echo "========================================================================"

# Change into presentation app directory
cd "${APP_DIR}"

# Check for python3
if command -v python3 &>/dev/null; then
    # Open browser automatically on macOS or Linux
    if [[ "$OSTYPE" == "darwin"* ]]; then
        (sleep 1 && open "http://localhost:${PORT}") &
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        (sleep 1 && xdg-open "http://localhost:${PORT}" 2>/dev/null || sensible-browser "http://localhost:${PORT}") &
    fi
    echo "⚡ Serving presentation at http://localhost:${PORT} ... (Press Ctrl+C to stop)"
    python3 -m http.server ${PORT}
elif command -v node &>/dev/null; then
    if [[ "$OSTYPE" == "darwin"* ]]; then
        (sleep 1 && open "http://localhost:${PORT}") &
    fi
    echo "⚡ Serving presentation with Node.js at http://localhost:${PORT} ..."
    npx -y serve -l ${PORT} "${APP_DIR}"
else
    echo "❌ Error: Neither Python 3 nor Node.js was found in PATH."
    exit 1
fi
