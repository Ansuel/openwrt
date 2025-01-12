#!/bin/sh

BR=br0
DEV=eth0
N_DSA_PORTS=4

IP=192.168.83.115
DST=192.168.83.120
GW=192.168.83.1

RATE=100
NSTRICT=2
QUANTA="quanta 1514 1514 1514 1514 1514 3528"
PRIOMAP="priomap 1 2 3 4 5 6 7 0"

PORT0=6001
PORT1=6002
PRIO0=0
PRIO1=5

TIME=30

# configure netowrk
{
	brctl addbr $BR
	sleep 1
	for i in $(seq $N_DSA_PORTS); do
		ip link set dev lan$i up
		brctl addif $BR lan$i
	done
	ip a a $IP/24 dev $BR
	ip link set dev $BR up
	ip r a default via $GW
} >/dev/null 2>&1
sleep 2
ping -c 10 $DST

# configure tc policies
for q in $(seq $N_DSA_PORTS); do
	tc filter del dev lan$q egress
done >/dev/null 2>&1

# HTB root qdisc [10:]
tc qdisc replace dev $DEV root handle 10: htb offload
for q in $(seq $N_DSA_PORTS); do
	# HTB class qdisc [10:x] (associated to hw QoS channels)
	tc class add dev $DEV parent 10: classid 10:$q		\
		htb rate "$((RATE*q))mbit" ceil "$((RATE*q))mbit"
	# ETS qdisc [1:x] (ETS bands associated to hw QoS per-channel queues)
	tc qdisc replace dev $DEV parent 10:$q handle $q: 	\
		ets bands 8 strict $NSTRICT $QUANTA $PRIOMAP

	# add CLSACT qdisc on DSA ports
	tc qdisc add dev lan$q clsact
	# TC filters - skb priority is associated to ETS bands
	tc filter add dev lan$q protocol ip egress		\
		flower ip_proto tcp dst_port $PORT0		\
		action skbedit priority 0x${q}000$((PRIO0+1))
	tc filter add dev lan$q protocol ip egress		\
		flower ip_proto tcp dst_port $PORT1		\
		action skbedit priority 0x${q}000$((PRIO1+1))
done
sleep 2

echo -e "\n\n********* QDISC $DEV *********"
tc qdisc show dev $DEV

echo -e "\n\n"
cat /sys/kernel/debug/airoha-eth/qdma:0/qos-tx-meters
sleep 10
iperf3 -c $DST -p $PORT0 -t $((TIME*30)) > /dev/null &
for i in $(seq 15); do
	sleep 30
	iperf3 -c $DST -p $PORT1 -t $TIME > /dev/null
done &

while sleep 1; do
	clear
	cat /sys/kernel/debug/airoha-eth/qdma:0/xmit-rings
done

