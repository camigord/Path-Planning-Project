#!/bin/bash

# Remove the dedicated output directories
cd `dirname $0`

rm -rf build

# We're done!
echo Cleaned up the project!
