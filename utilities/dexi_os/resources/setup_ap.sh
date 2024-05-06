#== Main part ==#
#===============#
{
  scriptstart
  #== start your program here ==#
  infotitle "Masking existing network services"

  # disable debian networking and dhcpcd
  exec_cmd "systemctl mask networking.service"
  exec_cmd "systemctl mask dhcpcd.service"
  exec_cmd "mv /etc/network/interfaces /etc/network/interfaces~"
  exec_cmd "sed -i '1i resolvconf=NO' /etc/resolvconf.conf"

  infotitle "Enabling systemd-networkd and systemd-resolved"

  # enable systemd-networkd
  exec_cmd "systemctl enable systemd-networkd.service"
  exec_cmd "systemctl enable systemd-resolved.service"
  exec_cmd "ln -sf /run/systemd/resolve/resolv.conf /etc/resolv.conf"

  infotitle "Creating wlan0 wpa_supplicant file"

  cat >/etc/wpa_supplicant/wpa_supplicant-wlan0.conf <<EOF
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="${staSsid}"
    psk="${staPsk}"
}
EOF

  exec_cmd "chmod 600 /etc/wpa_supplicant/wpa_supplicant-wlan0.conf"
  exec_cmd "systemctl disable wpa_supplicant.service"
  exec_cmd "systemctl enable wpa_supplicant@wlan0.service"

  infotitle "Creating ap0 wpa_supplicant file"

  cat >/etc/wpa_supplicant/wpa_supplicant-ap0.conf <<EOF
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="${apSsid}"
    mode=2
    key_mgmt=WPA-PSK
    proto=RSN WPA
    psk="${apPsk}"
    frequency=2412
}
EOF

  exec_cmd "chmod 600 /etc/wpa_supplicant/wpa_supplicant-ap0.conf"

  infotitle "Creating both wlan0 and ap0 systemd network files"

  cat >/etc/systemd/network/08-wlan0.network <<EOF
[Match]
Name=wlan0
[Network]
DHCP=yes
EOF

  cat >/etc/systemd/network/12-ap0.network <<EOF
[Match]
Name=ap0
[Network]
Address=192.168.4.1/24
DHCPServer=yes
[DHCPServer]
DNS=84.200.69.80 84.200.70.40
EOF

  infotitle "Now for some slick systemd unit editing!"

  exec_cmd "systemctl disable wpa_supplicant@ap0.service"
  exec_cmd "cp /lib/systemd/system/wpa_supplicant@.service /etc/systemd/system/wpa_supplicant@ap0.service"
  exec_cmd "sed -i 's/Requires=sys-subsystem-net-devices-%i.device/Requires=sys-subsystem-net-devices-wlan0.device/' /etc/systemd/system/wpa_supplicant@ap0.service"
  exec_cmd "sed -i 's/After=sys-subsystem-net-devices-%i.device/After=sys-subsystem-net-devices-wlan0.device/' /etc/systemd/system/wpa_supplicant@ap0.service"
  exec_cmd "sed -i '/After=sys-subsystem-net-devices-wlan0.device/a Conflicts=wpa_supplicant@wlan0.service' /etc/systemd/system/wpa_supplicant@ap0.service"
  exec_cmd "sed -i '/Type=simple/a ExecStartPre=/sbin/iw dev wlan0 interface add ap0 type __ap' /etc/systemd/system/wpa_supplicant@ap0.service"
  exec_cmd "sed -i '/ExecStart=/a ExecStopPost=/sbin/iw dev ap0 del' /etc/systemd/system/wpa_supplicant@ap0.service"
  exec_cmd "systemctl daemon-reload"

  infotitle "Finally, setup the default wifi option"

  if [[ $flagOptD == 1 ]]; then
    exec_cmd "systemctl disable wpa_supplicant@wlan0.service"
    exec_cmd "systemctl enable wpa_supplicant@ap0.service"
  else
    exec_cmd "systemctl enable wpa_supplicant@wlan0.service"
    exec_cmd "systemctl disable wpa_supplicant@ap0.service"
  fi

  infotitle "YOU SHOULD NOW REBOOT YOUR PI" && echo "Run 'sudo reboot now'"