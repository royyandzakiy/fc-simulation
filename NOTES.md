In Windows

```bash
usbipd bind --busid 1-5
usbipd attach --wsl --busid 1-5
```

In WSL

```bash
# To check if the radio is discoverable in wsl
sudo modprobe usbhid hid-generic joydev evdev && ls -l /dev/input/

# to run the sim with joystick
~/.venvs/fc-sim/bin/python simulator/5_fc_sitl_cpp/plant.py --fc-args=--input=joystick
```