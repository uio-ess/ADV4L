# AreaDetector-Video4Linux (ADV4L)

This is a Video4Linux driver for EPICS AreaDetector.
It is used to get data from a source supported by Video4Linux (USB webcams, analog framegrabbers, V4L loopback...) and make it available as an EPICS AreaDetector device.

To build, clone this folder into your AreaDetector folder (e.g. synApps_6_2/support/areaDetector-R3-10) and run `make` inside the ADV4L folder.

To run an example IOC, run `start_epics.sh` from `ADV4L/iocs/ADV4LIOC/iocBoot/iocADV4L`.
By default it connects to `/dev/video0`, which is typically the first webcam.
To see the images, run the very simple viewer app in python/viewerApp/viewerApp.py.
