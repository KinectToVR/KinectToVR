## <ins>__[Discord server](https://discord.gg/YBQCRDG)__</ins> | <ins>__[Site](https://k2vr.tech/)__</ins>

## Experiments
This branch is dedicated to experiments and other funny stuff<br> 
that should never come off to the daylight.<br>
Generally, this is a sacrifice for the development of the [K2APP](https://github.com/KinectToVR/k2vr-application),<br>
but also if something works, it may be pushed to production and used, *yay!*

## Authors
AutoCalibration scripts are written by **[コレヂャン](https://github.com/korejan)**<br>
KinectToVR base is **[Sharky's](https://github.com/sharkyh20/)**<br>
**[Triping](https://github.com/TripingPC)** organizes the whole project<br>
The installer's property of **[Himbeer](https://github.com/HimbeersaftLP)**<br>
Rest is probably written by **[公彦赤屋先](https://github.com/KimihikoAkayasaki)**<br>

## License
This project is licensed under the GNU GPL v3 License 

## Build
You'll need:
 - Visual Studio 2019 (with: C++, v142 tools, ATL)<br>or just build tools for same (see GitHub Actions script)
 - Kinect SDK 1.8 & 2.0 installed and visible in PATH
 - Working installation of SteamVR for testing

Follow **[GitHub Actions script](https://github.com/KimihikoAkayasaki/KinectToVR/blob/master/.github/workflows/main.yml)**, or:<br>

- Clone the latest OpenVR and Eigen3 into ```external/```:<br>
   ```git clone https://github.com/ValveSoftware/openvr external/openvr```<br>
   ```git clone https://gitlab.com/libeigen/eigen external/eigen```

- Install ```vcpkg``` and its Visual Studio integration<br>
   ```git clone https://github.com/Microsoft/vcpkg.git```<br>
   ```cd vcpkg```<br>
   ```./bootstrap-vcpkg.sh```<br>
   ```./vcpkg integrate install```
   
- Install needed libraries (You should choose one linking method for all packages)<br>
   ```vcpkg install glm:x64-windows boost:x64-windows opencv3[world]:x64-windows curlpp:x64-windows sfml:x64-windows sfgui:x64-windows glew:x64-windows ```<br>
   (Now you may rest a bit, also consider using a drive other than ```C:/```, it'll be about 6-7GB)

- Build all in ```KinectToVR``` in ```x64/Release```

## Deploy
All needed dlls are automatically copied to the output folder with vcpkg.<br>
Please note that everything from vcpg is being linked dynamically,<br>
so to avoid errors with the driver, build it alone and copy to the desired folder, <br>
including all present dlls inside the output folder.
