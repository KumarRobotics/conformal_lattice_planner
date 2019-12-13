#!/bin/sh
trap "trap - TERM && kill -- -$$" INT TERM EXIT
./carla_server.sh
wait -1
