#!/bin/sh
# Install system configuration

sudo apt-get install supervisor
sudo cp -r etc /
sudo supervisorctl reload
