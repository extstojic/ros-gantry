#!/bin/bash
# Kill dnb_tool_manager (started by dnb_bundle) because our gantry driver
# fully replaces it. dnb_tool_manager publishes zeros to /dnb_tool_frame
# when TF lookups fail, causing UI position flickering.
# Our driver publishes to all the same topics with correct values.

sleep 3
echo "[kill_tool_manager] Checking for dnb_tool_manager..."
if rosnode list 2>/dev/null | grep -q "dnb_tool_manager"; then
    echo "[kill_tool_manager] Found dnb_tool_manager - killing it (gantry driver replaces it)"
    rosnode kill /dnb_tool_manager 2>/dev/null || true
    sleep 1
    echo "[kill_tool_manager] dnb_tool_manager killed successfully"
else
    echo "[kill_tool_manager] dnb_tool_manager not found - nothing to kill"
fi
