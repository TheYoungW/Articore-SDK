#!/bin/sh
set -eu

if [ "$#" -ne 4 ]; then
    echo "usage: sudo $0 LEFT_SERIAL LEFT_PORT RIGHT_SERIAL RIGHT_PORT" >&2
    exit 2
fi
if [ "$(id -u)" -ne 0 ]; then
    echo "this installer must run as root" >&2
    exit 2
fi

left_serial=$1
left_port=$2
right_serial=$3
right_port=$4
case "$left_serial" in
    ''|*[!A-Za-z0-9]*)
        echo "adapter serials must be non-empty alphanumeric strings" >&2
        exit 2
        ;;
esac
case "$right_serial" in
    ''|*[!A-Za-z0-9]*)
        echo "adapter serials must be non-empty alphanumeric strings" >&2
        exit 2
        ;;
esac
case "$left_port:$right_port" in
    0:0|0:1|1:0|1:1) ;;
    *)
        echo "adapter ports must be 0 or 1" >&2
        exit 2
        ;;
esac

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
rules_tmp=$(mktemp)
trap 'rm -f "$rules_tmp"' EXIT HUP INT TERM

sed \
    -e "s/@LEFT_SERIAL@/$left_serial/g" \
    -e "s/@LEFT_PORT@/$left_port/g" \
    -e "s/@RIGHT_SERIAL@/$right_serial/g" \
    -e "s/@RIGHT_PORT@/$right_port/g" \
    "$script_dir/90-arx-canfd.rules.in" >"$rules_tmp"

install -m 0755 "$script_dir/arx-configure-canfd" \
    /usr/local/sbin/arx-configure-canfd
install -m 0644 "$script_dir/arx-canfd@.service" \
    /etc/systemd/system/arx-canfd@.service
install -m 0644 "$rules_tmp" /etc/udev/rules.d/90-arx-canfd.rules

systemctl daemon-reload
udevadm control --reload-rules

echo "ARX CAN-FD hotplug configuration installed."
echo "Reconnect both adapters once, or rename/configure the current interfaces manually."
