#!/bin/bash
docker run -it --privileged --rm l2rpi:latest /usr/bin/qemu-system-x86_64 -m 128 -hda testpxe.qcow2 \
                              -boot n -nographic \
                              -option-rom /usr/lib/ipxe/qemu/pxe-rtl8139.rom \
                              -netdev user,id=n1 -device virtio-net-pci,netdev=n1
