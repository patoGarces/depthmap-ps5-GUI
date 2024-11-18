####!/usr/bin/env python3       // TODO: revisar en linux

import usb.core
import usb.util
import sys

VID_PS5 = "VID_05A9"


devices = usb.core.find(find_all=True)

print('Devices encontrados: ')
# Itera sobre cada dispositivo y muestra información
for device in devices:
    print(f"Dispositivo encontrado: ID {hex(device.idVendor)}:{hex(device.idProduct)}")
    print(f"  - Clase: {device.bDeviceClass}")
    print(f"  - Subclase: {device.bDeviceSubClass}")
    print(f"  - Protocolo: {device.bDeviceProtocol}")
    print(f"  - Descripción: {usb.util.get_string(device, 256, device.iProduct)}")
    print(f"  - Fabricante: {usb.util.get_string(device, 256, device.iManufacturer)}")
    print(f"  - Serial: {usb.util.get_string(device, 256, device.iSerialNumber)}")
    print()


# check if initialized device already exists
dev = usb.core.find(idVendor=0x05a9, idProduct=0x058a) 
if dev is not None:
    print('PS4 camera Version 1 already initialized')
    sys.exit()

dev = usb.core.find(idVendor=0x05a9, idProduct=0x058b)
if dev is not None:
    print('PS4 camera Version 2 already initialized')
    sys.exit()

dev = usb.core.find(idVendor=0x05a9, idProduct=0x058c)
if dev is not None:
    print('PS5 camera already initialized')
    sys.exit()

# find uninitialized device
dev = usb.core.find(idVendor=0x05a9, idProduct=0x0580)
if dev is None:
    print('PS4 camera not found')
    sys.exit()

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()

# helper function for chunking a file
def read_chunks(infile, chunk_size):
    while True:
        chunk = infile.read(chunk_size)
        if chunk:
            yield chunk
        else:
            return

chunk_size=512
index=0x14
value=0

firmware=open("firmware_V2.bin","rb")

# transfer 512b chunks of the firmware
for chunk in read_chunks(firmware, chunk_size):
    ret = dev.ctrl_transfer(0x40, 0x0, value, index, chunk)
    value+=chunk_size
    if value>=65536:
        value=0
        index+=1
    if len(chunk)!=ret:
        print("sent %d/%d bytes" % (ret,len(chunk)))

# command reboots device with new firmware and product id
try:
    ret = dev.ctrl_transfer(0x40, 0x0, 0x2200, 0x8018, [0x5b])
except:
    print('PS4 camera firmware uploaded and device reset')
