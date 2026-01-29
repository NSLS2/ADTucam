# ADTucam
EPICS areaDetector driver for communicating with detectors using the TUCAM SDK, originally written for the Tucsen Dhyana XF4040BSI at CSX which also uses a ActiveSilicon FireBird frame grabber PCIe card.

May need minor changes to support other Tucsen cameras.

> [!NOTE]
> This has only been tested on the [AXIS-SXRF-40](https://www.axis-photon.com/streak-camera/axis-sxrf-40-16mpix-soft-x-ray-scmos-camera-on-a-flange/). It has the [Tucsen Dhyana 4040BSI](https://www.tucsen.com/dhyana-4040-bsi-product/) camera electronics and uses the TUCAM SDK to interact with the camera. We use a CameraLink connection and the [ActiveSilicon FireBird frame grabber (model number: AS-FBD-1XCLD-2PE4)](https://www.activesilicon.com/products/firebird-camera-link-frame-grabber-1xcld-2pe4l/). You must download and install the required software separately.

> [!WARNING]
> There are still some issues that are not resolved with the TUCAM SDK.
> * Timeouts on `waitForFrame` don't always work
>   * We have a separate timeout thread as a fallback
> * Software triggering is completely unreliable and suffers waiting forever for the frame to arrive
>   * Use hardware triggering or free run mode
>
> We are unsure if these issues persist on other cameras, frame grabbers, software versions, etc.

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

## Ophyd Device Support

There is an implementation of an Ophyd device for this IOC here: https://github.com/NSLS2/csx-profile-collection/blob/11aed998787c442ceb02c38873fc34879efc3c03/startup/csx1/devices/areadetector.py#L673
