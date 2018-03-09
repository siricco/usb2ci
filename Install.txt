Installation of the USB-Common Interface from Hauppauge or Terratec for the usage with VDR and plugin ddci2:

1) Getting the Firmware
    Download the latest Windows Drivers for
    
    the Hauppauge WinTV-CI from here
    http://www.hauppauge.de/files/drivers/WinTV-CI_1_3607_26283_WHQL.zip
    
    or for the Terratec Cinergy-USBCI from here
    http://terratec.ultron.info/Receiver/Cinergy_CI_USB/Update/Cinergy_CI_USB_Drv_Vista_XP_3.5.0.1.exe
    
    Extract the archives, find and copy the File hcw11.sys or USB2CIUSB.sys into the "scripts" folder.
    
    After running the Perl-script "WintvCI_extract_firmware.pl" within "scripts" copy/move all 4 resp. 5 created
    firmwares (*.fw) to the right place for your Linux distribution - usually this is "/lib/firmware"
    
2) Building the Linux Kernel Modul
    Simply run "runmake". This creates the Module "wintv_usb2ci.ko" and installs it under 
    /lib/modules/KERNELVERSION/extra"

3) Make sure to have the vdr-plugin ddci2 installed and activated

4) Insert CAM and smartcard and plug in the USB-CI-Adapter

4) run VDR and switch to a scrambled channel

