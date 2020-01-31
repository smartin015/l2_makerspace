FROM bitnami/minideb:jessie

RUN apt-get update && apt-get install curl qemu -y && apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN qemu-img create -f qcow2 testpxe.qcow2 10

RUN /usr/bin/curl http://svn.savannah.gnu.org/viewvc/*checkout*/qemu/trunk/pc-bios/pxe-rtl8139.bin -o pxe-rtl8139.bin

CMD ["/usr/bin/qemu-system-x86_64", "-nographic", "-m", "1024", "-hda", "testpxe.qcow2", "-boot", "n", "-option-rom", "pxe-rtl8139.rom", "-net", "nic", "-net", "tap,ifname=virttap,script=no,downscript=no"]
