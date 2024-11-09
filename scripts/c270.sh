#!/bin/bash

# Define the vendor and product IDs for C270 Device
U2D2_VENDOR_ID="046d"
U2D2_PRODUCT_ID="0825"

# Define the udev rule content for C270
RULE_CONTENT="SUBSYSTEM==\"video4linux\", ATTRS{idVendor}==\"$U2D2_VENDOR_ID\", ATTRS{idProduct}==\"$U2D2_PRODUCT_ID\", SYMLINK+=\"c270\""

# Write the rule content to the rule file (overwrite mode)
echo "$RULE_CONTENT" | sudo tee /etc/udev/rules.d/99-c270.rules > /dev/null

# Reload the udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Udev rule for Logitech C270 created successfully!"