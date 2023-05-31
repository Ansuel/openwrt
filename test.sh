#! /bin/bash

ALL_TARGETS="$(perl ./scripts/dump-target-info.pl kernels 2>/dev/null)"

TARGETS_SUBTARGETS="$(echo "$ALL_TARGETS" | sort -u -t '/' -k1)"
TARGETS="$(echo "$ALL_TARGETS" | sort -u -t '/' -k1,1)"

target="all"
subtarget="all"
testing="testing"

[ $subtarget = 'one' ] && TARGETS_SUBTARGETS=$TARGETS

JSON_TARGETS_SUBTARGETS='['
FIRST=1
while IFS= read -r line; do
        TARGET_SUBTARGET=$(echo $line | cut -d " " -f 1)
        TARGET=$(echo $TARGET_SUBTARGET | cut -d "/" -f 1)
        SUBTARGET=$(echo $TARGET_SUBTARGET | cut -d "/" -f 2)

        [ $target != 'all' ] && [ $target != $TARGET ] && continue
        [ $subtarget != 'all' ] && [ $subtarget != 'one' ] && 
                [ $subtarget != $SUBTARGET ] && continue
        if [ "$testing" = "testing" ]; then
                TESTING_KERNEL_VER=$(echo $line | cut -d " " -f 3)
                [ -z $TESTING_KERNEL_VER ] && continue
        fi

        TUPLE='{"target":"'"$TARGET"'","subtarget":"'"$SUBTARGET"'","testing":"'"$TESTING_KERNEL_VER"'"}'
        [[ $FIRST -ne 1 ]] && JSON_TARGETS_SUBTARGETS="$JSON_TARGETS_SUBTARGETS"','
        JSON_TARGETS_SUBTARGETS="$JSON_TARGETS_SUBTARGETS""$TUPLE"
        FIRST=0
done <<< "$TARGETS_SUBTARGETS"
JSON_TARGETS_SUBTARGETS="$JSON_TARGETS_SUBTARGETS"']'

JSON_TARGETS='['
FIRST=1
while IFS= read -r line; do
        TARGET_SUBTARGET=$(echo $line | cut -d " " -f 1)
        TARGET=$(echo $TARGET_SUBTARGET | cut -d "/" -f 1)
        SUBTARGET=$(echo $TARGET_SUBTARGET | cut -d "/" -f 2)

        [ $target != 'all' ] && [ $target != $TARGET ] && continue
        if [ "$testing" = "testing" ]; then
                TESTING_KERNEL_VER=$(echo $line | cut -d " " -f 3)
                [ -z $TESTING_KERNEL_VER ] && continue
        fi

        TUPLE='{"target":"'"$TARGET"'","subtarget":"'"$SUBTARGET"'","testing":"'"$TESTING_KERNEL_VER"'"}'
        [[ $FIRST -ne 1 ]] && JSON_TARGETS="$JSON_TARGETS"','
        JSON_TARGETS="$JSON_TARGETS""$TUPLE"
        FIRST=0
done <<< "$TARGETS"
JSON_TARGETS="$JSON_TARGETS"']'

echo $JSON_TARGETS_SUBTARGETS
echo $JSON_TARGETS