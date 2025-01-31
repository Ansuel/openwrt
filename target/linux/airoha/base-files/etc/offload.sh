#!/bin/sh

N_DSA_PORTS=4
BR_DEV=br0
LAN_DEV=eth0
LAN_SRC_IP=192.168.83.115
LAN_SRC_IP6="1001::a"
LAN_DST_IP=192.168.83.120
LAN_DST_IP6="1001::b"
# QoS channel ID for LAN destionation
LAN_CHANNEL_ID=1

WAN_DEV=eth1
WAN_SRC_IP=192.168.1.2
WAN_SRC_IP6="2001::a"
WAN_DST_IP=192.168.1.1
WAN_DST_IP6="2001::b"
# QoS channel ID for WAN destionation
WAN_CHANNEL_ID=4

RATE=100
NSTRICT=2
QUANTA="quanta 1514 1514 1514 1514 1514 3528"
PRIOMAP="priomap 1 2 3 4 5 6 7 0"

PORT0=6001
PORT1=6002
PRIO0=0
PRIO1=5

enable_hw_offload() {
	# FLOWTABLE
	nft -f /dev/stdin <<EOF
table inet filter {
	flowtable ft {
		hook ingress priority filter
		devices = { lan1, lan2, lan3, lan4, ${WAN_DEV} }
		flags offload;
	}
	chain forward {
		type filter hook forward priority filter; policy accept;
		meta l4proto { tcp, udp } flow add @ft
	}
}
EOF
}

enable_qos_offload() {
# TC
{
	# LAN -> WAN
	tc qdisc replace dev $LAN_DEV root handle 10: htb offload
	for i in $(seq $N_DSA_PORTS); do
		tc filter del dev lan$i egress
		tc filter del dev lan$i ingress

		# HTB class qdisc [10:x] (associated to hw QoS channels)
		tc class add dev $LAN_DEV parent 10: classid 10:$i		\
			htb rate "$((RATE*i))mbit" ceil "$((RATE*i))mbit"
		# ETS qdisc [1:x] (ETS bands associated to hw QoS per-channel queues)
		tc qdisc replace dev $LAN_DEV parent 10:$i handle $i: 		\
			ets bands 8 strict $NSTRICT $QUANTA $PRIOMAP

		# add CLSACT qdisc on DSA ports
		tc qdisc add dev lan$i clsact
		# TC filters - skb priority is associated to ETS bands
		tc filter add dev lan$i protocol ip egress		\
			flower ip_proto tcp dst_port $PORT0		\
			action skbedit priority 0x${i}000$((PRIO0+1))
		tc filter add dev lan$i protocol ip egress		\
			flower ip_proto tcp dst_port $PORT1		\
			action skbedit priority 0x${i}000$((PRIO1+1))
		tc filter add dev lan$i protocol ip ingress		\
			flower ip_proto tcp dst_port $PORT0		\
			action skbedit mark $((8*WAN_CHANNEL_ID+PRIO0))
		tc filter add dev lan$i protocol ip ingress		\
			flower ip_proto tcp dst_port $PORT1		\
			action skbedit mark $((8*WAN_CHANNEL_ID+PRIO1))
	done

	# WAN -> LAN
	tc filter del dev $WAN_DEV egress
	tc filter del dev $WAN_DEV ingress

	tc qdisc replace dev $WAN_DEV root handle 10: htb offload
	# HTB class qdisc [10:1] (associated to hw QoS channels)
	tc class add dev $WAN_DEV parent 10: classid 10:$WAN_CHANNEL_ID			\
		htb rate "${RATE}mbit" ceil "${RATE}mbit"
	# ETS qdisc [1:1] (ETS bands associated to hw QoS per-channel queues)
	tc qdisc replace dev $WAN_DEV parent 10:$WAN_CHANNEL_ID handle $WAN_CHANNEL_ID: \
		ets bands 8 strict $NSTRICT $QUANTA $PRIOMAP

	tc qdisc add dev $WAN_DEV clsact
	tc filter add dev $WAN_DEV protocol ip ingress		\
		flower ip_proto tcp dst_port $PORT0		\
		action skbedit mark $((8*LAN_CHANNEL_ID+PRIO0))
	tc filter add dev $WAN_DEV protocol ip ingress		\
		flower ip_proto tcp dst_port $PORT1		\
		action skbedit mark $((8*LAN_CHANNEL_ID+PRIO1))

	} >/dev/null 2>&1
}

# NETWORKING
{
	# LAN
	ip link add name $BR_DEV type bridge
	sleep 1
	for i in $(seq $N_DSA_PORTS); do
		ip link set dev lan$i up
		ip link set dev lan$i master $BR_DEV
	done
	ip addr add $LAN_SRC_IP/24 dev $BR_DEV
	ip -6 addr add $LAN_SRC_IP6/64 dev $BR_DEV nodad
	ip link set dev $BR_DEV up

	# WAN
	ip addr add $WAN_SRC_IP/24 dev $WAN_DEV
	ip -6 addr add $WAN_SRC_IP6/64 dev $WAN_DEV nodad
	ip link set dev $WAN_DEV up

	nft flush ruleset
} >/dev/null 2>&1

ping -c 5 $LAN_DST_IP
ping -6 -c 5 $LAN_DST_IP6
ping -c 5 $WAN_DST_IP
ping -6 -c 5 $WAN_DST_IP6

echo ""
echo -n "Enable HW offloading? [Y/n]"..
read ENABLE_OFFLOAD
echo $ENABLE_OFFLOAD
[ "$ENABLE_OFFLOAD" = "n" -o "$ENABLE_OFFLOAD" = "N" ] || enable_hw_offload

echo -n "Enable QoS offloading? [Y/n]"..
read ENABLE_QOS
echo $ENABLE_QOS
[ "$ENABLE_QOS" = "n" -o "$ENABLE_QOS" = "N" ] || enable_qos_offload
