#! /bin/bash

# run the unkei server

./unkei_server/utils.sh check_ros
./unkei_server/utils.sh source_ros
python unkei_server/server.py & disown
