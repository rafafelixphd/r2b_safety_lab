#!/usr/bin/env python
import sys
import usb.core
import usb.util
import numpy as np

from typing import Dict
from pathlib import Path
from dataclasses import dataclass

from ...logger import get_logger

logger = get_logger(namespace="PS4EyeFirmware")

@dataclass
class PS4EyeFirmwareData:
    firmware_version: float
    id_vendor: int
    id_product: int

ps4_cuh_zeh1 = PS4EyeFirmwareData(firmware_version=2.0, id_vendor=0x05a9, id_product=0x058a)
ps4_cuh_zeh2 = PS4EyeFirmwareData(firmware_version=2.0, id_vendor=0x05a9, id_product=0x058b)


ASSETS_DIR = Path(__file__).parent / "assets"

class PS4EyeFirmware:
    def __init__(self, interface: PS4EyeFirmwareData, *args, **kwargs):

        self.dev = usb.core.find(idVendor=interface.id_vendor, idProduct=interface.id_product)
        if self.dev is None:
            raise Exception(f'PS4 camera not found {interface=}')
        logger.info(f'PS4 camera interface found | {interface=}')

    def __call__(self, *args, **kwargs):
        self.upload_firmware()

    def update(self, *args, **kwargs):
        self.upload_firmware()

    def upload_firmware(self, *args, **kwargs):
        self.dev.set_configuration()
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

        with open(ASSETS_DIR / "firmware_V2.bin","rb") as firmware_file:
            for chunk in read_chunks(firmware_file, chunk_size):
                ret = self.dev.ctrl_transfer(0x40, 0x0, value, index, chunk)
                value+=chunk_size
                if value>=65536:
                    value=0
                    index+=1
                if len(chunk)!=ret:
                    logger.info("sent %d/%d bytes" % (ret,len(chunk)))

        # command reboots device with new firmware and product id
        try:
            ret = self.dev.ctrl_transfer(0x40, 0x0, 0x2200, 0x8018, [0x5b])
        except:
            logger.info('PS4 camera firmware uploaded and device reset')