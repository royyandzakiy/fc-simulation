In Windows

```bash
usbipd bind --busid 1-5
usbipd attach --wsl --busid 1-5
```

In WSL

```bash
sudo modprobe usbhid hid-generic joydev evdev && ls -l /dev/input/
```