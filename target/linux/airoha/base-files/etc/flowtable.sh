#!/bin/sh

N_DSA_PORTS=4
BR=br0
WAN=eth1

LAN_IP=192.168.83.115
GW_LAN_IP=192.168.83.1
DST_LAN=192.168.83.120

WAN_IP=192.168.1.2
DST_WAN=192.168.1.1

# configure netowrk
{
	# LAN
	brctl addbr $BR
	sleep 1
	for i in $(seq $N_DSA_PORTS); do
		ip link set dev lan$i up
		brctl addif $BR lan$i
	done
	ip a a $LAN_IP/24 dev $BR
	ip link set dev $BR up
	ip r a default via $GW_LAN_IP

	# WAN
	ip a a $WAN_IP/24 dev $WAN
} >/dev/null 2>&1

ping -c 5 $DST_LAN
ping -c 5 $DST_WAN

# FLOWTABLE
nft flush ruleset
nft -f /dev/stdin <<EOF
table inet nat {
	chain postrouting {
		type nat hook postrouting priority filter; policy accept;
		oifname ${WAN} masquerade
	}
}
table inet filter {
	flowtable ft {
		hook ingress priority filter
		devices = { lan1, lan2, lan3, lan4, ${WAN} }
		flags offload;
	}
	chain forward {
		type filter hook forward priority filter; policy accept;
		meta l4proto { tcp, udp } flow add @ft
	}
}
EOF

nft list ruleset
