#!/bin/bash


unzip x.zip

cd x

make linux

sudo mv wermit /usr/bin/kermit

cd -

echo "done!"




