#!/bin/sh

for i in `find . -name 'scripts' -type d`;do chmod a+x $i/*;done