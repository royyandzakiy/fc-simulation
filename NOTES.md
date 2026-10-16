In Windows

```bash
usbipd bind --busid 1-5
usbipd attach --wsl --busid 1-5
```

In WSL

```bash
# To check if the radio is discoverable in wsl
sudo modprobe usbhid hid-generic joydev evdev && ls -l /dev/input/
```
```bash
# run the SITL
/mnt/c/project-coding/cpp/202609/fc-simulation/bin/clang/fc_sitl_cpp

# run the simulator (plant), and will automatically run the SITL
~/.venvs/fc-sim/bin/python simulator/5_fc_sitl_cpp/plant.py --sticks=plant
~/.venvs/fc-sim/bin/python simulator/5_fc_sitl_cpp/plant.py --sticks=fc # still not working
```