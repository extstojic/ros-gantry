#!/bin/bash
# Restart dnb_tool_manager so it picks up the correct params for gantry
# 
# Problem: dnb_tool_manager is started by dnb_bundle BEFORE start.launch sets
# the tool_frame_source=topic params. So it uses default TF-based config,
# which fails TF lookups and publishes zeros to /dnb_tool_frame.
#
# Fix: Wait for gantry driver to be publishing, then restart dnb_tool_manager
# so it reloads with tool_frame_source=topic pointing to our topic.

echo "[restart_tool_manager] Waiting for gantry driver to publish tool frame..."
sleep 5

# Verify our driver is publishing
if rostopic echo /dnb_gantry_simulator/tool_frame -n 1 --timeout=5 > /dev/null 2>&1; then
    echo "[restart_tool_manager] Gantry driver is publishing. Restarting dnb_tool_manager..."
    
    # Kill the old instance (which has stale config)
    rosnode kill /dnb_tool_manager 2>/dev/null || true
    sleep 2
    
    # Restart it - it will pick up the params set by start.launch
    rosrun dnb_tool_manager dnb_tool_manager_node __name:=dnb_tool_manager &
    
    echo "[restart_tool_manager] dnb_tool_manager restarted with correct config"
else
    echo "[restart_tool_manager] WARNING: Gantry driver not publishing - not restarting dnb_tool_manager"
fi
