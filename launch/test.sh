#!/bin/sh

res=`echo "abc def ghi" | grep "xyz"`
echo "res=$res len=${#res}"

if [ ${#res} -gt 0 ]; then
  echo "res is not empty"
else
  echo "res is empty"
fi
