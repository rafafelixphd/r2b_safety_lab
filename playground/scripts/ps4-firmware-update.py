import argparse
from r2b.video.ps4.firmware import PS4EyeFirmware, PS4EyeFirmwareData, ps4_cuh_zeh1, ps4_cuh_zeh2

from r2b.logger import get_logger
logger = get_logger(namespace="PS4EyeFirmware")

def id2interface(choice_id: int) -> PS4EyeFirmwareData:
    choice_id = int(choice_id)
    
    if choice_id == 1:
        return ps4_cuh_zeh1
    elif choice_id == 2:
        return ps4_cuh_zeh2
    else:
        raise ValueError(f"Invalid choice_id: {choice_id}")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Upload PS4 Eye firmware')
    parser.add_argument('--interface', type=id2interface, help='PS4 Eye interface')
    parser.add_argument('--update', action='store_true', help='Update PS4 Eye firmware')

    args = parser.parse_args()
    ps4_firmware = PS4EyeFirmware(args.interface)
    if args.update:
        logger.info("Uploading firmware...")
        ps4_firmware.update()
