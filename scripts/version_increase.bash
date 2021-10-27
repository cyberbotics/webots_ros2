#!/usr/bin/env bash

# Please run the script from the repository root.


NEW_VERSION=$(echo $1 | grep -E -o "[0-9]{1,}\.[0-9]{1,}\.[0-9]{1,}")
if [ -z "${NEW_VERSION}" ]; then
    echo "Please run the script with a version number, e.g. './scripts/version_increase.bash 1.1.1'"
    exit 1
fi

# Upgrade the version in the Python setup.py files
PYTHON_SETUP_FILES=$(find -maxdepth 2 -name setup.py)
for file in ${PYTHON_SETUP_FILES}; do
    echo ${file}
    sed -i -E "s/version='[0-9]{1,}\.[0-9]{1,}\.[0-9]{1,}'/version='${NEW_VERSION}'/g" ${file}
done

# Upgrade the version in the XML package files
XML_PACKAGE_FILES=$(find -maxdepth 2 -name package.xml)
for file in ${XML_PACKAGE_FILES}; do
    echo ${file}
    sed -i -E "s/<version>[0-9]{1,}\.[0-9]{1,}\.[0-9]{1,}<\/version>/<version>${NEW_VERSION}<\/version>/g" ${file}
done
