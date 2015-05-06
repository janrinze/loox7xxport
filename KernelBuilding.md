# Introduction #

To build a kernel for the loox on Linux you can download the cross compiler from the Downloads section.

# Details #
short list of building a kernel:

  * Download the cross compiler from the Downloads section.
  * unpack the archive in /opt :
```
 sudo tar xjf xscale_toolchain.tar.bz2 -C /opt
```
  * get the mercurial package for your distro to be able to clone the kernel sources.
  * clone the repo:
```
hg clone https://loox7xxport.googlecode.com/hg/ loox7xxport
```
  * change to the kernel directory:
```
cd loox7xxport
```
  * setup the cross compiler environment vars
```
export ARCH=arm
export CROSS_COMPILE=/opt/xscale/bin/arm-xscale-linux-gnu- 
```
  * configure the kernel:
```
make defconfig loox720_defconfig
```
  * build the kernel:
```
make zImage
```
  * building kernel modules:
```
make modules
```
  * create archive of modules:
```
mkdir install
export INSTALL_MOD_PATH=`pwd`/install
make modules
make modules_install
cd install
tar czf modules.tgz ./lib
```
  * the created modules.tgz file will contain the modules for this specific kernel.

WIP..