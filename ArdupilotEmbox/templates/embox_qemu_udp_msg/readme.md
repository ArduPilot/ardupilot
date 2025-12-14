## Run embox
./scripts/qemu/auto_qemu_with_sd_card

## Run mavproxy
mavproxy.py --out 10.0.2.16:14550 --master tcp:10.0.2.16:5760 --sitl 10.0.2.16:5501 --map --console
