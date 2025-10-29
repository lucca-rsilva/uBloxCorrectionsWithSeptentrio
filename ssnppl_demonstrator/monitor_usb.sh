#!/bin/bash
# monitor_usb.sh

while true; do
    if ! grep -q "ttyUSB0" /proc/tty/driver/usbserial 2>/dev/null; then
        echo "USB device disappeared, waiting..."
        sleep 2
        continue
    fi
    
    # Check for stall errors
    if dmesg | tail -5 | grep -q "urb stopped"; then
        echo "PL2303 STALLED! Resetting..."
        sudo usbreset /dev/ttyUSB0
        sleep 2
    fi
    
    sleep 5
done