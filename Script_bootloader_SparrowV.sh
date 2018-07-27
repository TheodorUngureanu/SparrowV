#!/bin/bash

avrdude -c avrispmkii -p m644rfr2 -e -u -U flash:w:ATmegaBOOT_168_atmega644rfr2.hex
avrdude -c avrispmkii -P usb -p m644rfr2 -U lfuse:w:0xf7:m
avrdude -c avrispmkii -P usb -p m644rfr2 -U hfuse:w:0xd8:m
avrdude -c avrispmkii -P usb -p m644rfr2 -U efuse:w:0xfd:m
