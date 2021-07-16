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

- Clone Valve's ```OpenVR``` to ```external/``` (eventually remove ```-master``` from folder name)<br>
- Restore NuGet packages for ```KinectToVR``` <br>(Actually, if you have all the stuff in vcpkg, just enable it in project settings)
- Build all in ```KinectToVR``` in ```x64/Release```

## Deploy
Grab all needed files from your current KinectToVR installation folder.<br>
This also applies to OpenVR driver folders structure and files.
