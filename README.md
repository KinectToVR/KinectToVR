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

Follow **[GitHub Actions script](https://github.com/KinectToVR/KinectToVR/blob/experiments/.github/workflows/main.yml)**, or:<br>

- Install ```vcpkg``` and its Visual Studio integration<br>
   (cd into somewhere you want it to be)<br>
   ```git clone https://github.com/Microsoft/vcpkg.git```<br>
   ```cd vcpkg```<br>
   ```./bootstrap-vcpkg.sh```<br>
   ```./vcpkg integrate install```

- Install needed libraries (You should choose one linking method for all packages)<br>
   ```vcpkg install opencv3[world]:x64-windows boost:x64-windows glm:x64-windows curlpp:x64-windows cereal:x64-windows sfml:x64-windows glew:x64-windows```<br>
   (Now you may rest a bit, also consider using a drive other than ```C:```, it'll be about 6-7GB without cleaned buildtrees)

- Clone the latest OpenVR, GLM and Eigen3 into ```external/```:<br>
   ```git clone https://github.com/ValveSoftware/openvr external/openvr```<br>
   ```git clone https://gitlab.com/libeigen/eigen external/eigen```
   ```git clone https://github.com/g-truc/glm external/glm```

- Fix min/max error in GLM (unresolved with NOMINMAX for now)
   ```sed -i '/#include <limits>/c\#include <limits>\n\n#undef min\n#undef max' external/glm/glm/gtx/component_wise.inl```

- Clone the latest K2APP and setup shortcuts to external deps:<br>
   ```git clone https://github.com/KinectToVR/k2vr-application external/KTVR```<br>
   ```New-Item -ItemType Junction -Path external/KTVR/external/openvr -Target external/openvr```<br>
   ```New-Item -ItemType Junction -Path external/KTVR/external/eigen -Target external/eigen```<br>
   ```New-Item -ItemType Junction -Path external/KTVR/external/glm -Target external/glm```

- Setup GLog<br>
   ```git clone https://github.com/google/glog.git external/glog```<br>
   ```git reset --hard f8c8e99fdfb998c2ba96cfb470decccf418f0b30```<br>
   ```cd external/glog```<br>
   ```mkdir vcbuild; cd vcbuild```<br>
   ```cmake -DBUILD_SHARED_LIBS=ON ..```<br>
   ```msbuild glog.vcxproj "/p:Configuration=Release;Platform=x64```<br>
   ```WindowsTargetPlatformVersion=10.0"```<br>

- Setup GFlags<br>
   ```git clone https://github.com/gflags/gflags.git external/gflags```<br>
   ```git reset --hard 827c769e5fc98e0f2a34c47cef953cc6328abced```<br>
   ```cd external/gflags```<br>
   ```mkdir vcbuild; cd vcbuild```<br>
   ```cmake -DBUILD_SHARED_LIBS=ON ..```<br>
   ```msbuild gflags.vcxproj "/p:Configuration=Release;Platform=x64;WindowsTargetPlatformVersion=10.0"```<br>

- Setup SFGUI<br>
   ```git clone https://github.com/KimihikoAkayasaki/SFGUI external/SFGUI```<br>
   ```cd external/SFGUI```<br>
   ```mkdir build; cd build```<br>
   ```cmake "-DBUILD_SHARED_LIBS=ON [YOUR VCPKG TOOLCHAIN COMMAND HERE]" ..```<br>
   ```msbuild SFGUI.vcxproj "/p:Configuration=Release;Platform=x64;WindowsTargetPlatformVersion=10.0"```<br>

- Build the K2API & K2APP's OpenVR driver:<br>
   ```msbuild "/t:KinectToVR_API" "/p:Configuration=Release;Platform=x64;WindowsTargetPlatformVersion=10.0"```<br>
   ```msbuild "/t:Driver_KinectToVR" "/p:Configuration=Release;Platform=x64;WindowsTargetPlatformVersion=10.0"```

- Build the K2EX:<br>
   ```msbuild "/p:Configuration=Release;Platform=x64;WindowsTargetPlatformVersion=10.0"```

## Deploy
All needed dlls are automatically copied to the output folder.<br>
Please note that everything from vcpkg is being linked dynamically,<br>
so to avoid errors with the driver, if you've added any additional libraries,<br>
build it alone and copy to the desired folder, including all present dlls inside the output folder.<br>
For now, the folder structure is generated and dlls are copied automatically.<br>
You will find the built driver in ```external/KTVR/x64/$(Configuration)/driver/``` <br>and the built K2EX in ```x64/$(Configuration)```. (```Release``` configuration should be used, though)
