#!/bin/sh
# Configure and enable a standalone wireless access point on a Raspberry PI with
# this script, adapted from a guide available at:
#   www.raspberrypi.org/documentation/configuration/wireless/access-point.md
# Written by Tiger Sachse.

# Ensure the user is root!
if [ ! $(id -u) -eq 0 ]; then
    printf "This script must be run as root!\n"
    exit 1
fi

# Default network settings for this script.
ssid="EZRC"
interface="wlan1"
password="ezrassor"

# If the user has passed in arguments to this script, change the default network
# settings appropriately. This loop is a little wonky because I'm trying to avoid
# using Bash/KSH and remain completely POSIX compliant.
set_ssid=false
set_password=false
set_interface=false
for argument in "$@"; do
    if [ "$set_ssid" = "true" ]; then
        ssid="$argument"
        set_ssid=false
    elif [ "$set_password" = "true" ]; then
        password="$argument"
        set_password=false
    elif [ "$set_interface" = "true" ]; then
        interface="$argument"
        set_interface=false
    else
        case $argument in
            "--interface")
                set_interface=true
                ;;
            "--ssid")
                set_ssid=true
                ;;
            "--password")
                set_password=true
                ;;
        esac
    fi
done

# IPTABLES configuration.
IPTABLES_FILE="/etc/iptables.ipv4.nat"

# DHCPCD configuration.
DHCPCD_FILE="/etc/dhcpcd.conf"
DHCPCD_CONTENTS="\
interface $interface
    static ip_address=192.168.4.1/24
    nohook wpa_supplicant
"

# DNSMASQ configuration.
DNSMASQ_FILE="/etc/dnsmasq.conf"
DNSMASQ_CONTENTS="\
interface=$interface
    dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
"

# HOSTAPD configuration.
DEFAULT_HOSTAPD_FILE="/etc/default/hostapd"
DEFAULT_HOSTAPD_CONTENTS='DAEMON_CONF="/etc/hostapd/hostapd.conf"'
HOSTAPD_FILE="/etc/hostapd/hostapd.conf"
HOSTAPD_CONTENTS="\
ssid=$ssid
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=$password
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
"

# SYSCTL configuration.
SYSCTL_FILE="/etc/sysctl.conf"
SYSCTL_CONTENTS="net.ipv4.ip_forward=1"

# RC.LOCAL configuration.
RCLOCAL_FILE="/etc/rc.local"
RCLOCAL_TEMP_FILE="/tmp/rclocal.temp"
RCLOCAL_CONTENTS="\
iptables-restore < /etc/iptables.ipv4.nat

exit 0
"

# Install access point software and a DHCP server.
apt install -y dnsmasq hostapd
systemctl stop dnsmasq
systemctl stop hostapd

# Append all file configurations to their respective files.
printf "$DHCPCD_CONTENTS" >> "$DHCPCD_FILE"
printf "$DNSMASQ_CONTENTS" >> "$DNSMASQ_FILE"
printf "$HOSTAPD_CONTENTS" >> "$HOSTAPD_FILE"
printf "$DEFAULT_HOSTAPD_CONTENTS" >> "$DEFAULT_HOSTAPD_FILE"
printf "$SYSCTL_CONTENTS" >> "$SYSCTL_FILE"

# Add RCLOCAL_CONTENTS to RCLOCAL_FILE. The contents must be inserted
# before RCLOCAL_FILE's final line. That final line (a call to "exit") is
# re-added by RCLOCAL_CONTENTS.
sed '$d' "$RCLOCAL_FILE" > "$RCLOCAL_TEMP_FILE"
printf "$RCLOCAL_CONTENTS" >> "$RCLOCAL_TEMP_FILE"
cp "$RCLOCAL_TEMP_FILE" "$RCLOCAL_FILE"

# Add a masquerade via iptables.
iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
iptables-save > "$IPTABLES_FILE"

#systemctl start hostapd
#systemctl start dnsmasq
