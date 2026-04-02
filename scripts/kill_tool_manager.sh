#!/bin/bash
# Restart helper for environments where dnb_tool_manager needs a forced refresh.
# Kept for compatibility with external launch flows.

sleep 3
echo "[kill_tool_manager] Checking for dnb_tool_manager..."
if rosnode list 2>/dev/null | grep -q "dnb_tool_manager"; then
    echo "[kill_tool_manager] Found dnb_tool_manager - killing it"
    rosnode kill /dnb_tool_manager 2>/dev/null || true
    sleep 1
    echo "[kill_tool_manager] dnb_tool_manager killed successfully"
else
    echo "[kill_tool_manager] dnb_tool_manager not found - nothing to kill"
fi
