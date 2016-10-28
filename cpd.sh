#!/bin/bash
sudo dd if=output/mx6sdl/freertos/sabre_ai_rev_a/freertos_epit.bin of=/dev/sde bs=1K seek=1 skip=1 && sync
umount /dev/sde1
umount /dev/sde2

