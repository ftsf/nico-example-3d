# Package

version = "0.1.0"
author = "impbox"
description = "nico-3d-example"
license = "?"

# Deps
requires "nim >= 1.2.0"
requires "nico >= 0.2.5"

srcDir = "src"

import strformat

const releaseOpts = "-d:danger"
const debugOpts = "-d:debug"

task runr, "Runs nico-3d-example for current platform":
 exec &"nim c -r {releaseOpts} -o:nico-3d-example src/main.nim"

task rund, "Runs debug nico-3d-example for current platform":
 exec &"nim c -r {debugOpts} -o:nico-3d-example src/main.nim"

task release, "Builds nico-3d-example for current platform":
 exec &"nim c {releaseOpts} -o:nico-3d-example src/main.nim"

task webd, "Builds debug nico-3d-example for web":
 exec &"nim c {debugOpts} -d:emscripten -o:nico-3d-example.html src/main.nim"

task webr, "Builds release nico-3d-example for web":
 exec &"nim c {releaseOpts} -d:emscripten -o:nico-3d-example.html src/main.nim"

task debug, "Builds debug nico-3d-example for current platform":
 exec &"nim c {debugOpts} -o:nico-3d-example_debug src/main.nim"

task deps, "Downloads dependencies":
 exec "curl https://www.libsdl.org/release/SDL2-2.0.12-win32-x64.zip -o SDL2_x64.zip"
 exec "unzip SDL2_x64.zip"
 #exec "curl https://www.libsdl.org/release/SDL2-2.0.12-win32-x86.zip -o SDL2_x86.zip"

task androidr, "Release build for android":
  if defined(windows):
    exec &"nicoandroid.cmd"
  else:
    exec &"nicoandroid"
  exec &"nim c -c --nimcache:android/app/jni/src/armeabi {releaseOpts}  --cpu:arm   --os:android -d:androidNDK --noMain --genScript src/main.nim"
  exec &"nim c -c --nimcache:android/app/jni/src/arm64   {releaseOpts}  --cpu:arm64 --os:android -d:androidNDK --noMain --genScript src/main.nim"
  exec &"nim c -c --nimcache:android/app/jni/src/x86     {releaseOpts}  --cpu:i386  --os:android -d:androidNDK --noMain --genScript src/main.nim"
  exec &"nim c -c --nimcache:android/app/jni/src/x86_64  {releaseOpts}  --cpu:amd64 --os:android -d:androidNDK --noMain --genScript src/main.nim"
  withDir "android":
    if defined(windows):
      exec &"gradlew.bat assembleDebug"
    else:
      exec "./gradlew assembleDebug"
