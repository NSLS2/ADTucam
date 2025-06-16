# Tucam Support

Here is where you should add your TUCam SDK driver files.


## Linux Instructions

Copy the TUCam SDK header files and shared object files.

```bash
mkdir os
cp /path/to/tucam/* ./os/linux-x86_64/
```

Should result in:
```bash
$ ls ./os/linux-x86_64
TUCamApi.h  TUDefine.h  libTUCam.so  libTUCam.so.1  libTUCam.so.1.0  libTUCam.so.1.0.0  libphxapi-x86_64.so
```


## Windows Instructions

Copy the TUCam SDK header files and Windows library file.

```bash
mkdir os
cp /path/to/tucam/* ./os/windows-x64/
```

Should result in:
```bash
$ ls ./os/windows-x64/
TUCam.lib  TUCamApi.h  TUDefine.h
```

