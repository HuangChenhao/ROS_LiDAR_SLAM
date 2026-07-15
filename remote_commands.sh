#!/bin/bash
echo "=== Export driver patch (base64) ==="
echo "__B64_START__"
base64 -w0 /home/pi/rosmaster_tools/Ackman_driver_R2_patched.py
echo ""
echo "__B64_END__"
