#!/usr/bin/perl

# Wintv-CI: Extract the firmwre(s) found in the windows driver.
# The driver contains one EZ-USB code-banker firmware (with A2,A3,..commands)
# and up to four CI-firmwares matching to different revisions of the device

# (c) 2017, Helmut Binder <cco@aon.at>
# Released under GPLv2

use strict;
use POSIX;

# mc-edit: tab-size = 8,fake half-tabs

#----- handled drivers

my $wintvci_fn = "hcw11.sys";
my $usb2ci_fn  = "USB2CIUSB.sys";

#----- file r/w

sub __read_file($) {
    my ($fn) = @_;

    if ( !(-r $fn) ) {
	print "file '$fn' not found\n";
	return();
    }

    my $data = ();
    open R,"<",$fn or die "cannot open $fn";
    my $rc = read (R,$data,-s R);
    close R;

    return($data);
}

sub __write_file($$) {
    my ($fn,$data) = @_;

    open R,">",$fn or die "cannot open $fn";
    my $rc = syswrite (R,$data);
    close R;
    print "Wrote file '$fn'\n";
}

#----- exe parsing fo find data-section

my $mz_sig = "MZ";
my $pe_sig = "PE\x0\x0";

my $SZ_NT_HEADER = 4 + 20; # sig32 + image-file-header
my $SZ_SECTION_HEADER = 40;

sub parse_exe($$) {
    my ($fdata,$filename) = @_;
    print "\nLooking for the '.data' section in file '$filename' ...\n";

    # EXE-Header
    my $ofs = 0;
    my ($mz, $hdsz, $newexe) = unpack("x${ofs}a2x6Sx50L",$fdata);
    #printf "%d: %04X,%X %X\n",$rc, $mz, $hdsz, $newexe;
    if ($mz ne $mz_sig) {
	print "MZ-signature not found!\n";
	return(0);
    }
    # NT-header
    $ofs = $newexe;
    my ($pe, $mach, $num_sect, $sz_opthd) = unpack("x${ofs}a4SSx12S",$fdata);
    #printf "%d: %08X,%X %X %X\n",$rc, $pe, $mach, $num_sect, $sz_opthd;
    if ($pe ne $pe_sig) {
	print "PE-signature not found!\n";
	return(0);
    }

    # Sections
    $ofs += $SZ_NT_HEADER + $sz_opthd;

    for (my $i = 0; $i < $num_sect; $i++) {
	my ($name, $misc, $vaddr, $sz_raw, $ptr_raw, $character) =
		unpack("x${ofs}A8LLLLx12L ",$fdata);
	#printf "%9s %8X va:%8X sz:%8X ptr:%8X ch:%8X\n"
	#	$name, $misc, $vaddr, $sz_raw, $ptr_raw, $character;
	if ($name eq ".data") {
	    printf " --> found at offset 0x%04X, size 0x%04X (%d.) bytes.\n",
			$ptr_raw,$sz_raw,$sz_raw;
	    if ($ptr_raw & 0xF) {
		print " --> NOT paragraph aligned - unknown result\n";
		$ptr_raw = 0;
	    }
	    return($ptr_raw, $sz_raw);
	}
	$ofs += $SZ_SECTION_HEADER;
    }
    print "--> NOT found!\n";
    return(0);
}

#-----find firmwares in .data section

#     find all firmwares, continous from .data[0], on 8-byte aligned addresses
#     block-format:
#     data[0x16|0x46] { int16 len, int16 adr, char last, data[0x10|0x40], char zero }

my $FX2_RAMSIZE = 0x4000; # max internal ram ?
my $FW_BSZ16 = 0x16;
my $FW_BSZ46 = 0x46;

sub parse_firmware($$$) {
    my ($fdata, $start, $end) = @_;

    my @bszs = ($FW_BSZ16, $FW_BSZ46); # the possible block-sizes
    foreach (@bszs) {
	my $bsz = $_;
	my ($ram_min,$ram_max) = ($FX2_RAMSIZE,0);

	for ( my $i = $start, my $bcnt = 0; $i <= ($end-$bsz); $i += $bsz, $bcnt++ ) {
	    my ($len,$adr,$last) = unpack("x${i}SSC",$fdata);
	    if ( ($last == 1) && ($len == 0) && ($adr == 0) && $bcnt ) {
		# FW-end-marker and valid blocks found
		$bcnt++;	# take the end-marker block too
		printf "Firmware found at [0x%05X] Bsz: 0x%02X Bcnt: %d. -> ",
			    $start,$bsz,$bcnt;
		printf "FW-Size %5d. bytes, RAM: [0x%04X - 0x%04X]\n",
			    $bsz*$bcnt, $ram_min, $ram_max;
		return($bsz, $bcnt, $ram_min, $ram_max);
	    }
	    elsif ( (!$last) && ($len) && ($len < ($bsz-5)) && ($adr < ($FX2_RAMSIZE-$len)) ) {
		# valid block
		($adr < $ram_min) and $ram_min = $adr;
		(($adr+$len) > $ram_max) and $ram_max = $adr+$len;
	    }
	    else {    # bad block
		last; # restart check with next block-size
	    }
	}
    }
    printf "No Firmware at [0x%05X]\n",$start;
    return(0);
}

#----- this do'es the firmware extraction

sub extract_firmwares($) {
    my ($fname) = @_;

    #read
    my $fdata = __read_file($fname);
    (length $fdata) or return;

    # find .fata section
    my ($dptr,$dsize) = parse_exe($fdata, $fname);
    ($dptr) or return;

    # parse
    my @firmwares = ();
    my $end = $dptr+$dsize;
    for ( my $start = $dptr; $start < $end; ) {
	my ($bsz, $bcnt, $ram_min, $ram_max) = parse_firmware($fdata,$start,$end);

	(!$bsz) and last; # done

	my $fw = substr($fdata, $start, $bsz*$bcnt);
	push(@firmwares,$fw);

	my $fwsz = $bsz * ($bcnt);
	$start += $fwsz;

	# firmwares starts at 8-byte alignmnt
	if ($start & 0x7) {
	    $start &= ~0x7;
	    $start += 8;    # align to next 8-byte adr
	}
    }
    my $fw_cnt = @firmwares;
    ($fw_cnt) or return;

    # my base-name and the position of the EZ-USB code-banker firmware
    my ($fw_base, $i_cb) = 
	    ($fname eq $wintvci_fn) ? ( "wintvci", -1) :	# code-banker is last
	    ($fname eq $usb2ci_fn)  ? ( "usb2ci",   0)  : ();	# code-banker is first
    ($fw_base) or return;

    my $i_min = 1 + $i_cb; 	# pos of first ci-firmware (rev.1)
    my $i_max = $fw_cnt + $i_cb;# pos of last ci-firmware (rev.3 / rev.4)

    # FX2 code-banker-firmware
    __write_file("${fw_base}_cb.fw",$firmwares[$i_cb]);

    # CI-firmwares
    for (my $rev = 1, my $i = $i_min; $i < $i_max; $i++, $rev++ ) { # is sorted by rev.1, rev.2, ,...
	__write_file("${fw_base}_r${rev}.fw",$firmwares[$i]);
    }
}
#---

extract_firmwares($wintvci_fn);
extract_firmwares($usb2ci_fn);
