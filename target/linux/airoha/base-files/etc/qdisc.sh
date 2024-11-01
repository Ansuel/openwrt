#!/bin/sh

IP=192.168.83.115
DST0=192.168.83.120
DST1=192.168.83.121

DEV0=lan1
DEV1=lan2

RATE0="100mbit"
RATE1="200mbit"
NSTRICT0=2
NSTRICT1=8
QUANTA0="quanta 1514 1514 1514 1514 1514 3528"
QUANTA1=""

PORT0=6001
PRIO0=0

PORT1=6002
PRIO1=5

TIME=30

brctl addbr br0
sleep 1
for i in $(seq 4); do
	ip link set dev lan$i up
	brctl addif br0 lan$i
done
ip a a $IP/24 dev br0
ip link set dev br0 up
sleep 2
ping -c 10 $DST0
ping -c 10 $DST1
sleep 2

tc filter del dev $DEV0 egress > /dev/null 2>&1
tc filter del dev $DEV1 egress > /dev/null 2>&1

tc qdisc add dev $DEV0 root handle 1: tbf rate $RATE0 burst 10kb latency 70ms
tc qdisc add dev $DEV1 root handle 1: tbf rate $RATE1 burst 10kb latency 70ms

tc qdisc replace dev $DEV0 parent 1: handle 2: ets bands 8 strict $NSTRICT0 $QUANTA0
tc qdisc replace dev $DEV1 parent 1: handle 2: ets bands 8 strict $NSTRICT1 $QUANTA1

tc qdisc add dev $DEV0 clsact
tc qdisc add dev $DEV1 clsact
sleep 2

tc filter add dev $DEV0 protocol ip egress flower ip_proto tcp dst_port $PORT0 action skbedit priority $PRIO0 flowid 2:$((PRIO0+1))
tc filter add dev $DEV0 protocol ip egress flower ip_proto tcp dst_port $PORT1 action skbedit priority $PRIO1 flowid 2:$((PRIO1+1))

tc filter add dev $DEV1 protocol ip egress flower ip_proto tcp dst_port $PORT0 action skbedit priority $PRIO0 flowid 2:$((PRIO0+1))
tc filter add dev $DEV1 protocol ip egress flower ip_proto tcp dst_port $PORT1 action skbedit priority $PRIO1 flowid 2:$((PRIO1+1))

echo -e "\n\n********* $DEV0 *********"
tc qdisc show dev $DEV0
tc filter show dev $DEV0 egress

echo -e "\n\n********* $DEV1 *********"
tc qdisc show dev $DEV1
tc filter show dev $DEV1 egress

echo -e "\n\n"
cat /sys/kernel/debug/airoha-eth:1/qos-tx-meters
sleep 10

iperf3 -c $DST0 -p $PORT0 -t $((TIME*30)) > /dev/null &
for i in $(seq 15); do
	sleep 30
	iperf3 -c $DST0 -p $PORT1 -t $TIME > /dev/null
done &

iperf3 -c $DST1 -p $PORT0 -t $((TIME*30)) > /dev/null &
for i in $(seq 15); do
	sleep 30
	iperf3 -c $DST1 -p $PORT1 -t $TIME > /dev/null
done &

while sleep 1; do
	clear
	cat /sys/kernel/debug/airoha-eth:1/qos-tx-counters
	cat /sys/kernel/debug/airoha-eth:1/xmit-rings
done

