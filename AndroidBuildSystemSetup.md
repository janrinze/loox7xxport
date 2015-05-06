# Introduction #

This page summarizes the steps needed to set up the Android build system for the Loox port. The steps below get you all Android binaries _except_ for Linux kernel, Wifi modules, and Wifi and Bluetooth firmware (though see further below on how to add these).

# Steps for main Android build #

  1. Be sure to have sufficient disk space available, a complete build takes about 11GB, sources only (including git repos) take about 7GB.
  1. Get the original Android source tree, using the **Repo** tool as explained at http://source.android.com/source/download.html. When initializing your Repo client, specify branch **froyo**. Get everything that Repo wants to fetch by default.
  1. Get the following non-default Android repositorires as well, again check out branch **froyo** (either by manually using git clone, the repositories are at git://android.git.kernel.org, or by editing **.repo/manifest.xml** and adding the additinal repositories there):
    * platform/hardware/alsa\_sound.git (put into your local tree at hardware/alsa\_sound)
    * platform/external/alsa-lib.git (put into your local tree at external/alsa-lib)
    * platform/external/alsa-utils.git (put into your local tree at external/alsa-utils)
  1. Change into directory **external/wpa\_supplicant\_6** and do
```
git remote add github git://github.com/MartinR/android-external-wpa_supplicant_6.git

git checkout -t github/froyo-loox
```
> This will fetch the modified wpa\_supplicant\_6 from github.
  1. Change back to root of your Android tree and do
```
mkdir vendor
cd vendor
mkdir loox
cd loox
git clone git://github.com/MartinR/android-vendor-loox.git .
git checkout -t 
```
> This will fetch the vendor specifics for the Loox from github.

Now you have all the relevant Android sources checked out and ready to be built. For building, change into the root of your Android tree and do
```
. build/envsetup.sh
make PRODUCT-loox-eng
```
The Android binaries will end up in **out/target/product/loox720**. The file **system.img** contains the Android root filesystem (an ext2 image of it), **ramdisk.img** contains the initramfs to be given to the kernel (a gziped cpio archive).

# Adding Wifi modules and Wifi and Bluetooth firmware #

The wifi modules are created by the kernel build. After kernel build has completed, put the kernel tree files **drivers/net/wireless/acx/acx-mac80211.ko** and **drivers/net/wireless/acx/platform-pcmcia/pcmcia\_acx.ko** into the firectory **lib/modules/** of your Android root filesystem. For the firmware files, get the archive from http://loox7xxport.googlecode.com/files/firmware.tgz and extract into directory **etc/firmware/** of your Android root filesystem (be sure to not miss the symbolic links contained in the archive).