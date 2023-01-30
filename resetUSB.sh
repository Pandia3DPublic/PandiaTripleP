#!/bin/bash

busses=$(lsusb | cut -d' ' -f2 | sort -u)
bind_usb() {
  echo "$1" >/sys/bus/usb/drivers/usb/bind
}
unbind_usb() {
  echo "$1" >/sys/bus/usb/drivers/usb/unbind
}
for bus in ${busses[@]}; do 
    echo "Resetting Bus $bus"
    b=$(echo $bus | sed 's/^0*//') # remove leading zeroes
    unbind_usb "usb$b"
    sleep 1
    bind_usb "usb$b"
done

#ports=$(lsusb | cut -d' ' -f6 | sort -u)
#sleep 1
#for port in ${ports[@]}; do
#    echo -n "${port} "
#    sudo usbreset ${port}
#done
