### Critical notice!
In the latest Raspbian kernels stereoscopic support has been occasionally broken by implementing new AWB algorithm. You can read some details [here](https://github.com/raspberrypi/firmware/issues/1253). Kernels affected: 4.19.57 and up to 4.19.97. Our Raspbian image provided has no such a problem (as it is based on older kernel).

The solution (**obsolete**): 

after boot run once this command before accessing your cameras:
```
sudo vcdbg set awb_mode 0
```
This will turn AWB algo to the previous mode, and stereo works again untill next reboot. So run this command after reboot, or add it to your autorun script. 

New solution (**recommended**):

`sudo rpi-update`

As of 10 of April, 2020, issue is fixed in the latest firmware update. So just use rpi-update.
 

# Installing OpenCV 3.x or 4.x on Raspbian Buster:

#### 1. Download Raspbian Buster and write image to your micro SD card.


#### 2. Enable two cameras support:
Put [dt-blob.bin](http://wiki.stereopi.com/files/dt-blob.bin.zip) file to /BOOT partition. Do not forget to extract it from ZIP archive.

#### 3. First boot

After the first boot you'll see setup wizard. 

Choose all settings you need, install all recommended updates and reboot the StereoPi.

Please do 'sudo rpi-update' after that. This will fix stereoscopic mode.

Enable camera:

`sudo raspi-config`

Go to "Interfacing options", choose "Enable camera", press "Yes", reboot your system.

#### 4. Set python 3 as a default:

`sudo nano ~/.bashrc`

add this row at the end of file:

`alias python='/usr/bin/python3'`

Save the file (Esc X X, "yes").

Activate new settings:

`source ~/.bashrc`

Check if the settings are applied correctly.

`python`

You should see "Python 3.7.3" in the first row.

If you see "Python 2.7.x" - please repeat previous steps.

(Type exit(0) to exit Python command line).

#### 5. Install OpenCV with PiWheels:

Workaround: to avoid "undefined symbol: __atomic_fetch_add8" error while "import cv2",
please install specific version of OpenCV.

For OpenCV 3.x use:

`sudo pip3 install opencv-python==3.4.6.27`

For OpenCV 4.x use:

`sudo pip3 install opencv-contrib-python==4.1.0.25`

#### 6. Install additional libs:

```
sudo apt-get install libhdf5-dev
sudo apt-get install libhdf5-serial-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libjasper-dev 
sudo apt-get install libqtgui4 
sudo apt-get install libqt4-test
```

You may also need matplotlib. Go this way:

`sudo pip3 install matplotlib`

#### 7. To check if everything Ok try to run our first test script:

`python 1_test.py`

You should see preview image. Press 'Q' for exit.

#### 8. Check raspistill 3D mode:

`raspistill -3d sbs -o 1.jpg`

You should see 5 seconds preview, and stereoscopic image "1.jpg" should be saved after 
running this code.

#### 9. How to avoid occasional stereoscopic support breaking?

Stereoscopic support is broken on kernels from 4.19.57 and up to 4.19.97

To fix it, just use "sudo rpi-update". This fix works from ~10 of April, 2020.

Stay tuned! 

And good luck with your OpenCV experiments! :-)
