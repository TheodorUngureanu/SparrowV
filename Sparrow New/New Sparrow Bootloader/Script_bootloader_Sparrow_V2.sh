#!/bin/bash

avrdude -c avrispmkii -P usb -p m128rfa1 -U efuse:w:0xfe:m
avrdude -c avrispmkii -P usb -p m128rfa1 -U hfuse:w:0xd0:m
avrdude -c avrispmkii -P usb -p m128rfa1 -U lfuse:w:0xf7:m
avrdude -c avrispmkii -p m128rfa1 -e -u -U ATmegaBOOT_168_atmega128rfa1.hex
