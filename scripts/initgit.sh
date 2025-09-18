#!/bin/zsh

echo "Fetching lastest commits"
git pull
echo "Initializing Submodules"
git submodule update --init --recursive
echo "Complete"
