# Introduction #

Once you have a kernel for the Loox there are some steps that need to be take to get Linux running on your Loox.


# Details #

## requirements ##
  * Haret.exe for booting Linux
  * kernel
  * SDcard with at least 1 GB
  * Linux PC to partition and format the SDcard
  * rootfs for Loox to be put on SDcard
> _**there are test files in the Downloads section that contain the rootfs, the kernel and haret.exe**_

## preparing the SDcard ##
  * partition the SDcard using either fdisk or the tools provided with you Linux distribution.
  * first partition will only need to be small to have sufficient room for Haret the kernel and some small utilities that can make life easier.
  * Assign 64 MB for the first partition and leave the rest for the second.
  * format the first partition with FAT
  * format the second with EXT3
  * copy Haret.exe and zImage (the kernel) to the FAT partition
  * untar using root privileges the rootfs in the second partition

## starting Linux ##
  * make backups of anything you want to keep from your Loox to your PC. Using Linux wipes the memory and may trash the looxstore.
  * navigate to the SDcard dir
  * tap on Haret.exe
  * tap on start
  * linux will now boot.
  * there will be a small prompt at the bottom of the LCD

## networking with the Loox ##
  * Linux supports networking over USB
  * connect your Loox running Linux with the USB to a Linux PC
  * type: `ifconfig -a`
  * to get connection you need to give usb0 an ip address:
  * type: `sudo ifconfig usb0 192.168.0.1`
  * connect to the Loox using ssh
  * type: `ssh root@192.168.0.2`
  * password for the demo rootfs is 'debiansid'
  * if all is well you will now have a shell login

## internet access from your Loox ##
  * enable NAT on your Linux PC
  * type: `sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"`
  * assuming your pc has internet connection over eth0:
  * type: `sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE`
  * check internet access from your Loox
  * type: `ping www.google.com`

## update software on the loox ##
  * using 'apt-get' on the Loox it is possible to access the internet repository of Debain Sid.
  * update the local info of the repository
  * type: `apt-get update`
  * update all current software
  * type: `apt-get upgrade`