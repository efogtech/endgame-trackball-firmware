## Dongle 1k

<details>
<summary>Learn how to assemble</summary>

1. Place the dongle PCB into the shell.
2. Align the pin on the second part with the hole, and click the parts together.

<img width="1644" height="1405" alt="step1" src="https://github.com/user-attachments/assets/2e693c2d-d5e3-49e1-85e0-3948a8627295" />
<img width="4000" height="3000" alt="step2" src="https://github.com/user-attachments/assets/1012f294-5dbc-4e75-9270-be51a34af98e" />
<img width="2516" height="1845" alt="step3" src="https://github.com/user-attachments/assets/6ea102c8-0563-42b6-bbec-bbb76bff94bc" />
</details>

### Flashing

> [!TIP]
> Your dongle might come not flashed. Just copy the latest UF2 onto it. 

To enter DFU (Device Firmware Update) mode, hold the side button while inserting the dongle.  
The LED should turn green, and a new mass storage device will appear on your host.  
Copy your UF2 firmware there directly, without renaming, replacing, or doing anything else.  
After flashing, it will reboot itself into the firmware.  

[Latest firmware, UF2](https://efog.tech/storage/dongle-1k-firmware.uf2)   

### Using

<details>
<summary>Learn what LED colors mean</summary>

1. 🟦 Blue, solid  — not bonded, doing nothing.
2. 🟦 Blue, blinking — attempting to bond.
3. 🟨 Yellow, solid — bonded, peer idle.
4. 🟩 Green, solid — active communication.
5. 🟪 Purple, solid — shell relay mode.
6. 🟧 Orange, solid — link lost, reconnecting.
7. 🟧 Orange, blinking — link degraded. 
</details>

When you power up the dongle for the first time, it will not be bonded with the trackball.  
To create a bond, activate the dongle profile on the trackball (by switching to the 6th wireless slot).
Then, click a button on the dongle. Bonding will be performed automatically, and you're ready to go. 

### Shell relay mode

As the [Marshmellow UI](https://efog.tech/marshmellow-ui) is just a front-end for the Zephyr shell, this mode will enable you to connect to MUI using the dongle. 
A shell session must be explicitly requested, either by invoking an assignable behavior on the trackball side or by pressing the button on the dongle.  
   
When shell relay mode is active, you will be able to connect to the dongle via USB and use most of the MUI functionality.  
What doesn't work yet: storage partition backup/restore, monitor mode for acceleration curves.  
Shell relay mode is terminal automatically after a minute of shell inactivity. 

### Misc

1. Hold the dongle button for 5 seconds to remove the existing bond.
2. Execute `esb unpair` with MUI on the trackball side to forget the dongle.
3. The dongle has priority over USB. Switch to another slot to use USB.
