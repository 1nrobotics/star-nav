#!/bin/bash

# Star Nav - Onboard Basic Setup Script
# Quick setup for running Star Nav on onboard computer

set -e

echo "=========================================="
echo "  Star Nav - Onboard Setup"
echo "=========================================="
echo ""

# Set system time
echo "[*] Setting system time..."
sudo date -s "2025-12-17 22:50:00"

# Add user to docker group (no password needed)
echo "[*] Adding user to docker group..."
sudo usermod -aG docker $USER

# Add user to dialout group (serial port access)
echo "[*] Adding user to dialout group..."
sudo usermod -aG dialout $USER

# Install sshpass for VK180 password authentication
echo "[*] Installing sshpass for VK180 SSH access..."
sudo apt-get update -qq && sudo apt-get install -y -qq sshpass

# Configure VK180 network
echo "[*] Adding VK180 host entry..."
echo "10.42.0.64	compulab-imx8mp-01" | sudo tee -a /etc/hosts

# Setup multicast route for eCAL
echo "[*] Configuring multicast route for VK180..."
WIRED_CONN=$(nmcli -t -f NAME,TYPE connection show | grep ethernet | head -n1 | cut -d: -f1)
echo "    Using connection: $WIRED_CONN"
sudo nmcli connection modify "$WIRED_CONN" ipv4.routes "239.0.0.0/24" ipv4.route-metric 1
sudo nmcli connection down "$WIRED_CONN" && sudo nmcli connection up "$WIRED_CONN"

echo ""
echo "=========================================="
echo "  Setup Complete!"
echo "=========================================="
echo ""
echo "⚠️  LOGOUT and LOGIN again for group changes to take effect"
echo ""
echo "Then run:"
echo "  cd $WORKSPACE_ROOT"
echo "  ./docker/build.sh              # Build Docker image"
echo "  ./scripts/start_onboard.sh     # Start system"
echo ""
