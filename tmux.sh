#!/bin/bash


NAME="pyplc"

cd ~/private/pyPLC

if ! tmux has-session -t $NAME 2>/dev/null; then
    tmux new-session -d -s $NAME -n "main"
    tmux send-keys -t $NAME:main "~/startup.sh" C-m
fi
