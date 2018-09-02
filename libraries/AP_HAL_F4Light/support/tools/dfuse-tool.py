#!/usr/bin/env python3
import dfuse
import usb.core
import usb.util
import argparse
import sys

def find_device(args):
    usbdev = usb.core.find(idVendor=args.vid, idProduct=args.pid)

    if usbdev is not None:
        dfu = dfuse.DfuDevice(usbdev)
        for _,alt in dfu.alternates():
            if alt.configuration == args.cfg and alt.bInterfaceNumber == args.intf and alt.bAlternateSetting == args.alt:
                dfu.set_alternate(alt)
                status = dfu.get_status()
                if status[1] == dfuse.DfuState.DFU_ERROR:
                    print("Error cleared: %r" % (status,))
                    dfu.clear_status() # Clear left-over errors
                return dfu

    raise ValueError('No DfuSe compatible device found, check device information options (see --help)')


def list_dfu(args):
    usbdev = usb.core.find(idVendor=args.vid, idProduct=args.pid)

    if usbdev is None:
        raise ValueError('No STM32 DfuSe device found.')

    dfu = dfuse.DfuDevice(usbdev)
    for name, alt in dfu.alternates():
        print ("Device: [%.4x:%.4x] Cfg: %d Intf: %d Alt: %d '%s'" % ( \
                alt.device.idVendor, \
                alt.device.idProduct, \
                alt.configuration, \
                alt.bInterfaceNumber, \
                alt.bAlternateSetting, \
                name))

def leave_dfu(args):
    dfu = find_device(args)
    dfu.leave()
    status = dfu.get_status()
    if status[0] > 0:
        raise RuntimeError("An error occured. Status: %r" % status)

def erase(args):
    dfu = find_device(args)

    print ("Erasing. Please wait this might be long ...")
    dfu.erase(args.erase)
    status = dfu.wait_while_state(dfuse.DfuState.DFU_DOWNLOAD_BUSY)

    if status[1] != dfuse.DfuState.DFU_DOWNLOAD_IDLE:
        raise RuntimeError("An error occured. Device Status: %r" % status)

    print ("Done !")

def flash(args):
    dfufile = args.flash[0]
    dfu = find_device(args)

    if (dfufile.devInfo['vid'] != dfu.dev.idVendor or dfufile.devInfo['pid'] != dfu.dev.idProduct) and not args.force:
        raise ValueError("Vendor/Product id mismatch: [%.4x:%.4x] (file) [%.4x:%.4x] (device). Trying running with --force" % ( \
                dfufile.devInfo['vid'], \
                dfufile.devInfo['vid'], \
                dfu.dev.idVendor, \
                dfu.dev.idProduct))

    targets = [t for t in dfufile.targets if t['alternate'] == dfu.intf.bAlternateSetting]

    if len(targets) == 0:
        raise ValueError("No file target matches the device. Check the --alt setting")


    print ("Flashing. Please wait this might be long ...")
    for t in targets:
        print ("Found target %r" % t['name'])
        for idx, image in enumerate(t['elements']):
            print("Flashing image %d at 0x%.8X" % (idx, image['address']))

            print("Erasing ...")
            dfu.erase(image['address'])
            status = dfu.wait_while_state(dfuse.DfuState.DFU_DOWNLOAD_BUSY)
            if status[1] != dfuse.DfuState.DFU_DOWNLOAD_IDLE:
                raise RuntimeError("An error occured. Device Status: %r" % status)

            print("Flashing ...")
            transfer_size = 1024
            dfu.set_address(image['address'])
            status = dfu.wait_while_state(dfuse.DfuState.DFU_DOWNLOAD_BUSY)
            if status[1] != dfuse.DfuState.DFU_DOWNLOAD_IDLE:
                raise RuntimeError("An error occured. Device Status: %r" % status)
            
            data = image['data']
            blocks = [data[i:i + transfer_size] for i in range(0, len(data), transfer_size)]
            for blocknum, block in enumerate(blocks):
                print("Flashing block %r" % blocknum)
                dfu.write(blocknum, block)
                status = dfu.wait_while_state(dfuse.DfuState.DFU_DOWNLOAD_BUSY)
                if status[1] != dfuse.DfuState.DFU_DOWNLOAD_IDLE:
                    raise RuntimeError("An error occured. Device Status: %r" % status)

            print("Done")


    return

    status = dfu.wait_while_state(dfuse.DfuState.DFU_DOWNLOAD_BUSY)
    
    if status[1] != dfuse.DfuState.DFU_IDLE and status[1] != dfuse.DfuState.DFU_DOWNLOAD_IDLE:
        raise RuntimeError("An error occured. Status: %r" % status)

    print ("Done !")

parser = argparse.ArgumentParser(description="DfuSe flashing util for STM32")

action = parser.add_mutually_exclusive_group(required = True)
action.add_argument('--list', action='store_true', help='List available DfuSe interfaces')
action.add_argument('--leave', action='store_true', help='Leave DFU mode')
action.add_argument('--flash', nargs=1, action='store', help='Flash a DfuSe file', metavar='FILE', type=dfuse.DfuFile)
action.add_argument('--erase', action='store', help='Erase page at ADDRESS (must be page aligned)', metavar=('ADDRESS'), type=int)

devinfo = parser.add_argument_group('Device information')
devinfo.add_argument('--vid', action='store', type=int, default=0x0483, help='Device\'s USB vendor id, defaults to 0x0483')
devinfo.add_argument('--pid', action='store', type=int, default=0xdf11, help='Device\'s USB product id, defaults to 0xdf11')
devinfo.add_argument('--cfg', action='store', type=int, default=0, help='Device\'s configuration number, default to 0')
devinfo.add_argument('--intf', action='store', type=int, default=0, help='Device\'s interface number, defaults to 0')
devinfo.add_argument('--alt', action='store', type=int, default=0, help='Device\'s alternate setting number, defaults to 0')

others = parser.add_argument_group('Other Options')
others.add_argument('--force', '-f', action='store_true', help='Bypass sanity checks')

args = parser.parse_args()

#try:
if args.list:
    list_dfu(args)
elif args.leave:
    leave_dfu(args)
elif args.erase is not None:
    erase(args)
elif args.flash is not None:
    flash(args)
#except Exception as e:
#    print(e, file=sys.stderr)
#    sys.exit(-1)
