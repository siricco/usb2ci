Zur Installation des USB-Common Interface von Hauppauge oder Terratec:

1) Firmware

    Das Perl-Script "WintvCI_extract_firmware.pl" im Verzeichnis "WinTV-CI/scripte" ausführen.
    Aus den beiden vorhandenen Windows Treiberdateien wird die Firmware extrahiert.
    Alle "*.fw" Dateien oder auch nur die für den Hersteller des Adapters in das Verzeichnis /lib/firmware verschieben.

    Die vollständigen WIndows Treiber sind hier zu finden:

    http://www.hauppauge.de/files/drivers/WinTV-CI_1_3607_26283_WHQL.zip
    http://terratec.ultron.info/Receiver/Cinergy_CI_USB/Update/Cinergy_CI_USB_Drv_Vista_XP_3.5.0.1.exe

2) Linux Kernel Modul Ver. 0.2

    Im Ordner WinTV-USB2CI einfach das Shell-Script "runMake" ausführen.
    Das Modul "wintv-usb2ci.ko" wird erstellt und unter /lib/modules/KERNELVERSION/extra installiert

3) Falls noch nicht vorhanden das VDR-Plugin ddci2 installieren

3) USB-CI Adapter mit CI-Modul und Smartcard anschließen

4) VDR mit Plugin ddci2 starten (wird etwas länger dauern als ohne CI) und dann zu einem zur Smartcard passenden Sender wechseln.

