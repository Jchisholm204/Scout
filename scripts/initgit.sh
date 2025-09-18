#!/bin/zsh

# Script for Initializing the git submodules
# Need to run this after cloning the repository

echo "Fetching lastest commits"
git pull
echo "Initializing Submodules"
git submodule update --init --recursive
echo "Complete"
