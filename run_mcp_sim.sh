#!/usr/bin/env bash
# Launch the Go2 sim mission-control MCP server for the Claude CLI (stdio transport).
#
# The Claude CLI spawns this script. It sources ROS 2 + the sim workspace (so rclpy and the ZoneTask
# interface are importable) and matches the sim's DDS environment (so it discovers the running
# mission_control node), then execs the MCP server. Register it once with:
#
#     claude mcp add go2-sim -- "/abs/path/to/go2-sim/go2_ws/src/run_mcp_sim.sh"
#
# The env exports below MUST match the terminals the sim runs in, or DDS discovery will not find the
# services. They default to the sim runbook (domain 0) but respect any value already exported.

# This script lives at <go2_ws>/src/run_mcp_sim.sh, so the workspace is one directory up from src/.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVER="$WS/src/go2_inspection/go2_inspection/mcp_mission_server.py"

# Source ROS + the workspace overlay. Redirect sourcing chatter to stderr so it never corrupts the
# MCP stdout channel (the JSON-RPC stream).
source /opt/ros/jazzy/setup.bash 1>&2
source "$WS/install/setup.bash" 1>&2

# DDS discovery: the sim runs on the default domain (0), and the default transport already discovers
# mission_control. We only pin the domain, since the Claude CLI may spawn us with a minimal env. We do
# NOT force FASTDDS_BUILTIN_TRANSPORTS / ROS_LOCALHOST_ONLY here -- forcing localhost-only can mismatch
# a sim that was started without it and break discovery. If your sim sets them and the MCP tools report
# "not available", uncomment the two lines below to match your sim terminals.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# export ROS_LOCALHOST_ONLY=1

# Run the server with the python3 on PATH (the environment the Claude CLI launches us with, which must
# have fastmcp installed alongside rclpy).
exec python3 "$SERVER"
