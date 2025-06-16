# ADTucam
EPICS areaDetector driver for communicting with detectors using the TUCAM SDK, written for the Tucsen Dhyana XF4040BSI at CSX.

May need minor changes to support other Tucsen cameras.

## Quick Setup 

> NOTE: You must have the TUCAM SDK and EPICS Area Detector installed on your machine before starting.

### Copy the TUCAM SDK headers and library files

See [tucamSupport/README.md](./tucamSupport/README.md) for instructions.

### Create your startup script

If you want to launch your IOC from this repository, create a file under `iocs/tucamIOC/iocBoot/st.cmd` and set up your configuration there.

See https://areadetector.github.io/areaDetector/ADSimDetector/st_cmd.html for an example startup file.
### Launching the IOC

Then from the base of the repo, run:
```bash
make install
cd iocs/tucamIOC/iocBoot
./st.cmd
```
