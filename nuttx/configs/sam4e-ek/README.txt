README
======

This README discusses issues unique to NuttX configurations for the Atmel
SAM4E-EK development.  This board features the SAM4E16 MCU running at 96
or 120MHz.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Atmel Studio 6.1
  - Loading Code with J-Link
  - Writing to FLASH using SAM-BA
  - LEDs
  - Serial Console
  - Networking Support
  - AT25 Serial FLASH
  - SAM4E-EK-specific Configuration Options
  - Configurations

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
=====================

  The NuttX make system can be configured to support the various different
  toolchain options.  All testing has been conducted using the NuttX buildroot
  toolchain.  To use alternative toolchain, you simply need to add change of
  the following configuration options to your .config (or defconfig) file:

    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=y        : Atollic toolchain for Windos
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=y      : devkitARM under Windows
    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y      : Generic GCC ARM EABI toolchain for Linux
    CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y      : Generic GCC ARM EABI toolchain for Windows

  You may also have to modify the PATH in the setenv.h file if your
  make cannot find the tools.

  NOTE about Windows native toolchains
  ------------------------------------

  There are basically three kinds of GCC toolchains that can be used:

    1. A Linux native toolchain in a Linux environment,
    2. The buildroot Cygwin tool chain built in the Cygwin environment,
    3. A Windows native toolchain.

  There are several limitations to using a Windows based toolchain (#3) in a
  Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath'
     utility but you might easily find some new path problems.  If so, check
     out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
     links are used in Nuttx (e.g., include/arch).  The make system works
     around these problems for the Windows tools by copying directories
     instead of linking them. But this can also cause some confusion for
     you:  For example, you may edit a file in a "linked" directory and find
     that your changes had no effect.  That is because you are building the
     copy of the file in the "fake" symbolic directory.  If you use a
     Windows toolchain, you should get in the habit of making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This
     is because the dependencies are generated using Windows paths which do
     not work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

IDEs
====

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project (There is a simple RIDE project
  in the RIDE subdirectory).

  Makefile Build
  --------------
  Under Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/arm/src/sam34,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/sam34/sam_vectors.S.  You may need to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by RIDE.

NuttX EABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh sam4e-ek/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-eabi-defconfig-4.6.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE:  Unfortunately, the 4.6.3 EABI toolchain is not compatible with the
  the NXFLAT tools.  See the top-level TODO file (under "Binary loaders") for
  more information about this problem. If you plan to use NXFLAT, please do not
  use the GCC 4.6.3 EABI toochain; instead use the GCC 4.3.3 OABI toolchain.
  See instructions below.

NuttX OABI "buildroot" Toolchain
================================

  The older, OABI buildroot toolchain is also available.  To use the OABI
  toolchain:

  1. When building the buildroot toolchain, either (1) modify the cortexm3-eabi-defconfig-4.6.3
     configuration to use EABI (using 'make menuconfig'), or (2) use an exising OABI
     configuration such as cortexm3-defconfig-4.3.3

  2. Modify the Make.defs file to use the OABI conventions:

    +CROSSDEV = arm-nuttx-elf-
    +ARCHCPUFLAGS = -mtune=cortex-m3 -march=armv7-m -mfloat-abi=soft
    +NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-gotoff.ld -no-check-sections
    -CROSSDEV = arm-nuttx-eabi-
    -ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld -no-check-sections

NXFLAT Toolchain
================

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX SourceForge download site
  (https://sourceforge.net/projects/nuttx/files/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh sam4e-ek/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly builtNXFLAT binaries.

Atmel Studio 6.1
================

  You can use Atmel Studio 6.1 to load and debug code.

  - To load code into FLASH:

    Tools menus:  Tools -> Device Programming.

    Configure the debugger and chip and you are in business.

  - Debugging the NuttX Object File:

    1) Rename object file from nutt to nuttx.elf.  That is an extension that
       will be recognized by the file menu.

    2) Select the project name, the full path to the NuttX object (called
       just nuttx with no extension), and chip.  Take the time to resolve
       all of the source file linkages or else you will not have source
       level debug!

       File menu: File -> Open -> Open object file for debugging
       - Select nuttx.elf object file
       - Select AT91SAM4E16
       - Select files for symbols as desired
       - Select debugger

    3) Debug menu: Debug -> Start debugging and break
       - This will reload the nuttx.elf file into FLASH

    STATUS: At this point, Atmel Studio 6.1 claims that my object files are
    not readable.  A little more needs to be done to wring out this procedure.

Loading Code into SRAM with J-Link
==================================

  Loading code with the Segger tools and GDB
  ------------------------------------------

    1) Change directories into the directory where you built NuttX.
    2) Start the GDB server and wait until it is ready to accept GDB
       connections.
    3) Then run GDB like this:

         $ arm-none-eabi-gdb
         (gdb) target remote localhost:2331
         (gdb) mon reset
         (gdb) load nuttx
         (gdb) ... start debugging ...

  Loading code using J-Link Commander
  ----------------------------------

    J-Link> r
    J-Link> loadbin <file> <address>
    J-Link> setpc <address of __start>
    J-Link> ... start debugging ...

  STATUS:  As of this writing, I have no been successful writing to FLASH
  using the GDB server.  I think that this is because of issues with GPNVM1
  settings and flash lock bits.  In any event, the GDB server works great for
  debugging after writing the program to FLASH using SAM-BA.

Writing to FLASH using SAM-BA
=============================

  Assumed starting configuration:

    1. You have installed the J-Link USB driver

  Using SAM-BA to write to FLASH:

    1. Start the SAM-BA application, selecting (1) the SAM-ICE/J-Link
       port, and (2) board = at91sam4e16-ek.
    2. The SAM-BA menu should appear.
    3. Select the FLASH tab and enable FLASH access
    4. "Send" the file to flash
    5. Enable "Boot from Flash (GPNVM1)
    6. Reset the board.

  STATUS: Works great!

LEDs
====

  The SAM4E-EK board has three, user-controllable LEDs labelled D2 (blue),
  D3 (amber), and D4 (green) on the board.  Usage of these LEDs is defined
  in include/board.h and src/up_leds.c. They are encoded as follows:

    SYMBOL              Meaning                 D3*     D2      D4
    ------------------- ----------------------- ------- ------- -------
    LED_STARTED         NuttX has been started  OFF     OFF     OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF     OFF     ON
    LED_IRQSENABLED     Interrupts enabled      OFF     ON      OFF
    LED_STACKCREATED    Idle stack created      OFF     ON      ON
    LED_INIRQ           In an interrupt**       N/C     FLASH   N/C
    LED_SIGNAL          In a signal handler***  N/C     N/C     FLASH
    LED_ASSERTION       An assertion failed     FLASH   N/C     N/C
    LED_PANIC           The system has crashed  FLASH   N/C     N/C

  * If D2 and D4 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is D3=OFF, D4=ON and D2 faintly glowing.  This faint
    glow is because of timer interrupts that result in the LED being
    illuminated on a small proportion of the time.
*** D4 may also flicker normally if signals are processed.

Serial Console
==============

  By default, all of these configurations use UART0 for the NuttX serial
  console.  UART0 corresponds to the DB-9 connector J17 labelled "DBGU".
  This is a male connector and will require a female-to-female, NUL modem
  cable to connect to a PC.

  An alternate is USART1 which connects to the other DB-9 connector labelled
  "USART1".  USART1 is not enabled by default unless specifically noted
  otherwise in the configuration description.  A NUL modem cable must be
  used with the port as well.

  NOTE:  To avoid any electrical conflict, the RS232 and RS485 transceiver
  are isolated from the receiving line PA21.

  - Chose RS485 channel: Close 1-2 pins on JP11 and set PA23 to high level
  - Chose RS232 channel: Close 2-3 pins on JP11 and set PA23 to low level

  By default serial console is configured for 115000, 8-bit, 1 stop bit, and
  no parity.

Networking Support
==================

  Networking support via the can be added to NSH by selecting the following
  configuration options.

  Selecting the EMAC peripheral
  -----------------------------

  System Type -> SAM34 Peripheral Support
    CONFIG_SAM34_EMAC=y                 : Enable the EMAC peripheral

  System Type -> EMAC device driver options
    CONFIG_SAM34_EMAC_NRXBUFFERS=16     : Set aside some RS and TX buffers
    CONFIG_SAM34_EMAC_NTXBUFFERS=4
    CONFIG_SAM34_EMAC_PHYADDR=1         : KSZ8051 PHY is at address 1
    CONFIG_SAM34_EMAC_AUTONEG=y         : Use autonegotiation
    CONFIG_SAM34_EMAC_MII=y             : Only the MII interface is supported
    CONFIG_SAM34_EMAC_PHYSR=30          : Address of PHY status register on KSZ8051
    CONFIG_SAM34_EMAC_PHYSR_ALTCONFIG=y : Needed for KSZ8051
    CONFIG_SAM34_EMAC_PHYSR_ALTMODE=0x7 : "    " " " "     "
    CONFIG_SAM34_EMAC_PHYSR_10HD=0x1    : "    " " " "     "
    CONFIG_SAM34_EMAC_PHYSR_100HD=0x2   : "    " " " "     "
    CONFIG_SAM34_EMAC_PHYSR_10FD=0x5    : "    " " " "     "
    CONFIG_SAM34_EMAC_PHYSR_100FD=0x6   : "    " " " "     "

  PHY selection.  Later in the configuration steps, you will need to select
  the KSZ8051 PHY for EMAC (See below)

  Networking Support
    CONFIG_NET=y                        : Enable Neworking
    CONFIG_NET_SOCKOPTS=y               : Enable socket operations
    CONFIG_NET_BUFSIZE=562              : Maximum packet size (MTD) 1518 is more standard
    CONFIG_NET_RECEIVE_WINDOW=536       : Should be the same as CONFIG_NET_BUFSIZE
    CONFIG_NET_TCP=y                    : Enable TCP/IP networking
    CONFIG_NET_TCPBACKLOG=y             : Support TCP/IP backlog
    CONFIG_NET_TCP_READAHEAD_BUFSIZE=536  Read-ahead buffer size
    CONFIG_NET_UDP=y                    : Enable UDP networking
    CONFIG_NET_BROADCAST=y              : Needed for DNS name resolution
    CONFIG_NET_ICMP=y                   : Enable ICMP networking
    CONFIG_NET_ICMP_PING=y              : Needed for NSH ping command
                                        : Defaults should be okay for other options
  Device drivers -> Network Device/PHY Support
    CONFIG_NETDEVICES=y                 : Enabled PHY selection
    CONFIG_ETH0_PHY_KSZ8051=y           : Select the KSZ8051 PHY (for EMAC)

  Application Configuration -> Network Utilities
    CONFIG_NETUTILS_RESOLV=y            : Enable host address resolution
    CONFIG_NETUTILS_TELNETD=y           : Enable the Telnet daemon
    CONFIG_NETUTILS_TFTPC=y             : Enable TFTP data file transfers for get and put commands
    CONFIG_NETUTILS_UIPLIB=y            : Network library support is needed
    CONFIG_NETUTILS_WEBCLIENT=y         : Needed for wget support
                                        : Defaults should be okay for other options
  Application Configuration -> NSH Library
    CONFIG_NSH_TELNET=y                 : Enable NSH session via Telnet
    CONFIG_NSH_IPADDR=0x0a000002        : Select a fixed IP address
    CONFIG_NSH_DRIPADDR=0x0a000001      : IP address of gateway/host PC
    CONFIG_NSH_NETMASK=0xffffff00       : Netmask
    CONFIG_NSH_NOMAC=y                  : Need to make up a bogus MAC address
                                        : Defaults should be okay for other options

  You can also enable enable the DHCPC client for networks that use
  dynamically assigned address:

  Application Configuration -> Network Utilities
    CONFIG_NETUTILS_DHCPC=y             : Enables the DHCP client

  Networking Support
    CONFIG_NET_UDP=y                    : Depends on broadcast UDP

  Application Configuration -> NSH Library
    CONFIG_NET_BROADCAST=y
    CONFIG_NSH_DHCPC=y                  : Tells NSH to use DHCPC, not
                                        : the fixed addresses

  Using the network with NSH
  --------------------------

  So what can you do with this networking support?  First you see that
  NSH has several new network related commands:

    ifconfig, ifdown, ifup:  Commands to help manage your network
    get and put:             TFTP file transfers
    wget:                    HTML file transfers
    ping:                    Check for access to peers on the network
    Telnet console:          You can access the NSH remotely via telnet.

  You can also enable other add on features like full FTP or a Web
  Server or XML RPC and others.  There are also other features that
  you can enable like DHCP client (or server) or network name
  resolution.

  By default, the IP address of the SAM4E-EK will be 10.0.0.2 and
  it will assume that your host is the gateway and has the IP address
  10.0.0.1.

    nsh> ifconfig
    eth0    HWaddr 00:e0:de:ad:be:ef at UP
            IPaddr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0

  You can use ping to test for connectivity to the host (Careful,
  Window firewalls usually block ping-related ICMP traffic).  On the
  target side, you can:

    nsh> ping 10.0.0.1
    PING 10.0.0.1 56 bytes of data
    56 bytes from 10.0.0.1: icmp_seq=1 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=2 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=3 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=4 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=5 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=6 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=7 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=8 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=9 time=0 ms
    56 bytes from 10.0.0.1: icmp_seq=10 time=0 ms
    10 packets transmitted, 10 received, 0% packet loss, time 10100 ms

  NOTE: In this configuration is is normal to have packet loss > 0%
  the first time you ping due to the default handling of the ARP
  table.

  On the host side, you should also be able to ping the SAM4E-EK:

    $ ping 10.0.0.2

  You can also log into the NSH from the host PC like this:

    $ telnet 10.0.0.2
    Trying 10.0.0.2...
    Connected to 10.0.0.2.
    Escape character is '^]'.
    sh_telnetmain: Session [3] Started

    NuttShell (NSH) NuttX-6.31
    nsh> help
    help usage:  help [-v] [<cmd>]

      [           echo        ifconfig    mkdir       mw          sleep
      ?           exec        ifdown      mkfatfs     ping        test
      cat         exit        ifup        mkfifo      ps          umount
      cp          free        kill        mkrd        put         usleep
      cmp         get         losetup     mh          rm          wget
      dd          help        ls          mount       rmdir       xd
      df          hexdump     mb          mv          sh

    Builtin Apps:
    nsh>

  NOTE:  If you enable this feature, you experience a delay on booting.
  That is because the start-up logic waits for the network connection
  to be established before starting NuttX.  In a real application, you
  would probably want to do the network bringup on a separate thread
  so that access to the NSH prompt is not delayed.

  This delay will be especially long if the board is not connected to
  a network because additional time will be required to fail with timeout
  errors.

AT25 Serial FLASH
=================

  Connections
  -----------

  Both the SAM4E-EK include an Atmel AT25DF321A, 32-megabit, 2.7-volt
  SPI serial flash.  The SPI
  connection is as follows:

    ------ ------- ---------------
    SAM4E  AT25    SAM4E
    GPIO   PIN     FUNCTION
    ------ ------- ---------------
    PA13   SI      MOSI
    PA12   SO      MIS0
    PA14   SCK     SPCK
    PA5    /CS     NPCS3 (pulled high externally)
    ------ ------- ---------------

  Configuration
  -------------

  Support for the serial FLASH can be enabled in these configurations.  These
  are the relevant configuration settings.  These settings (1) Enable SPI0,
  (2) Enable DMAC0 to support DMA transfers on SPI for best performance,
  (3) Enable the AT25 Serial FLASH, and (3) Set up NuttX to configure the
  file system on the AT25 FLASH:

    System Type -> ATSAM3/4 Peripheral Support
      CONFIG_SAM34_SPI0=y                   : Enable SPI0
      CONFIG_SAM34_DMAC0=y                  : Enable DMA controller 0

    System Type -> SPI device driver options
      CONFIG_SAM34_SPI_DMA=y                : Use DMA for SPI transfers
      CONFIG_SAM34_SPI_DMATHRESHOLD=4       : Don't DMA for small transfers

    Device Drivers -> SPI Driver Support
      CONFIG_SPI=y                          : Enable SPI support
      CONFIG_SPI_EXCHANGE=y                 : Support the exchange method

    Device Drivers -> Memory Technology Device (MTD) Support
      CONFIG_MTD=y                          : Enable MTD support
      CONFIG_MTD_AT25=y                     : Enable the AT25 driver
      CONFIG_AT25_SPIMODE=0                 : Use SPI mode 0
      CONFIG_AT25_SPIFREQUENCY=20000000     : Use SPI frequency 12MHz

    The AT25 is capable of operation at 20MHz.  However, if you experience
    any issues with the AT25, then lower this frequency may give more
    predictable performance.

    File Systems -> FAT
      CONFIG_FS_FAT=y                       : Enable and configure FAT
      CONFIG_FAT_LCNAMES=y                  : Upper/lower case names
      CONFIG_FAT_LFN=y                      : Long file name support (See NOTE)
      CONFIG_FAT_MAXFNAME=32                : Limit filename sizes to 32 bytes

    NOTE: Use care if you plan to use FAT long file name feature in a product;
    There are issues with certain Microsoft patents on the long file name
    technology.

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                 : NSH board-initialization

    Board Selection
      CONFIG_SAM4EEK_AT25_AUTOMOUNT=y       : Mounts AT25 for NSH
      CONFIG_SAM4EEK_AT25_FTL=y             : Create block driver for FAT

  You can then format the AT25 FLASH for a FAT file system and mount the
  file system at /mnt/at25 using these NSH commands:

    nsh> mkfatfs /dev/mtdblock0
    nsh> mount -t vfat /dev/mtdblock0 /mnt/at25

  Then you an use the FLASH as a normal FAT file system:

    nsh> echo "This is a test" >/mnt/at25/atest.txt
    nsh> ls -l /mnt/at25
    /mnt/at25:
     -rw-rw-rw-      16 atest.txt
    nsh> cat /mnt/at25/atest.txt
    This is a test

SAM4E-EK-specific Configuration Options
=======================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP="sam34"

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_SAM34
       CONFIG_ARCH_CHIP_SAM3U
       CONFIG_ARCH_CHIP_ATSAM3U4

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=sam4e-ek (for the SAM4E-EK development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_SAM4EEK=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00020000 (128Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

    CONFIG_ARCH_IRQPRIO - The SAM3U supports interrupt prioritization

       CONFIG_ARCH_IRQPRIO=n

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

  Individual subsystems can be enabled:

    CONFIG_SAM34_SPI0          - Serial Peripheral Interface 0 (SPI0)
    CONFIG_SAM34_SPI1          - Serial Peripheral Interface 1 (SPI1)
    CONFIG_SAM34_SSC           - Synchronous Serial Controller (SSC)
    CONFIG_SAM34_TC0           - Timer/Counter 0 (TC0)
    CONFIG_SAM34_TC1           - Timer/Counter 1 (TC1)
    CONFIG_SAM34_TC2           - Timer/Counter 2 (TC2)
    CONFIG_SAM34_TC3           - Timer/Counter 3 (TC3)
    CONFIG_SAM34_TC4           - Timer/Counter 4 (TC4)
    CONFIG_SAM34_TC5           - Timer/Counter 5 (TC5)
    CONFIG_SAM34_TC6           - Timer/Counter 6 (TC6)
    CONFIG_SAM34_TC7           - Timer/Counter 7 (TC6)
    CONFIG_SAM34_TC8           - Timer/Counter 6 (TC8)
    CONFIG_SAM34_PWM           - Pulse Width Modulation (PWM) Controller
    CONFIG_SAM34_TWIM0         - Two-wire Master Interface 0 (TWIM0)
    CONFIG_SAM34_TWIS0         - Two-wire Slave Interface 0 (TWIS0)
    CONFIG_SAM34_TWIM1B        - Two-wire Master Interface 1 (TWIM1)
    CONFIG_SAM34_TWIS1         - Two-wire Slave Interface 1 (TWIS1)
    CONFIG_SAM34_UART0         - UART 0
    CONFIG_SAM34_UART1         - UART 1
    CONFIG_SAM34_USART0        - USART 0
    CONFIG_SAM34_USART1        - USART 1
    CONFIG_SAM34_USART2        - USART 2
    CONFIG_SAM34_USART3        - USART 3
    CONFIG_SAM34_AFEC0         - Analog Front End 0
    CONFIG_SAM34_AFEC1         - Analog Front End 1
    CONFIG_SAM34_DACC          - Digital-to-Analog Converter
    CONFIG_SAM34_ACC           - Analog Comparator
    CONFIG_SAM34_EMAC          - Ethernet MAC
    CONFIG_SAM34_CAN0          - CAN 0
    CONFIG_SAM34_CAN1          - CAN 1
    CONFIG_SAM34_SMC           - Static Memory Controller
    CONFIG_SAM34_NAND          - NAND support
    CONFIG_SAM34_PDCA          - Peripheral DMA controller
    CONFIG_SAM34_DMAC0         - DMA controller
    CONFIG_SAM34_UDP           - USB 2.0 Full-Speed device
    CONFIG_SAM34_CHIPID        - Chip ID
    CONFIG_SAM34_RTC           - Real Time Clock
    CONFIG_SAM34_RTT           - Real Time Timer
    CONFIG_SAM34_WDT           - Watchdog Timer
    CONFIG_SAM34_EIC           - Interrupt controller
    CONFIG_SAM34_HSMCI         - High Speed Multimedia Card Interface

  Some subsystems can be configured to operate in different ways. The drivers
  need to know how to configure the subsystem.

    CONFIG_GPIOA_IRQ
    CONFIG_GPIOB_IRQ
    CONFIG_GPIOC_IRQ
    CONFIG_GPIOD_IRQ
    CONFIG_GPIOE_IRQ
    CONFIG_GPIOF_IRQ
    CONFIG_GPIOG_IRQ
    CONFIG_GPIOH_IRQ
    CONFIG_GPIOJ_IRQ
    CONFIG_GPIOK_IRQ
    CONFIG_GPIOL_IRQ
    CONFIG_GPIOM_IRQ
    CONFIG_GPION_IRQ
    CONFIG_GPIOP_IRQ
    CONFIG_GPIOQ_IRQ

    CONFIG_USART0_ISUART
    CONFIG_USART1_ISUART
    CONFIG_USART2_ISUART
    CONFIG_USART3_ISUART

  SAM3U specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=0,1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

  LCD Options.  Other than the standard LCD configuration options
  (see configs/README.txt), the SAM4E-EK driver also supports:

    CONFIG_LCD_PORTRAIT - Present the display in the standard 240x320
       "Portrait" orientation.  Default:  The display is rotated to
       support a 320x240 "Landscape" orientation.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each SAM4E-EK configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh sam4e-ek/<subdir>
    cd -
    . ./setenv.sh

  Before sourcing the setenv.sh file above, you should examine it and perform
  edits as necessary so that BUILDROOT_BIN is the correct path to the directory
  than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       and misc/tools/

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART0 (J3).

  3. All of these configurations are set up to build under Linux using the
     EABI buildroot toolchain (unless stated otherwise in the description of
     the configuration).  That build selection can easily be reconfigured
     using 'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_LINUX=y                 : Linux or other pure POSIX invironment

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARMV7M_OABI_TOOLCHAIN=n      : EABI (Not OABI

     If you want to use the Atmel GCC toolchain, for example, here are the
     steps to do so:

     Build Setup:
       CONFIG_HOST_WINDOWS=y   : Windows
       CONFIG_HOST_CYGWIN=y    : Using Cygwin or other POSIX environment

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : General GCC EABI toolchain under windows

     Library Routines ->
       CONFIG_CXX_NEWLONG=n                : size_t is an unsigned int, not long

     This re-configuration should be done before making NuttX or else the
     subsequent 'make' will fail.  If you have already attempted building
     NuttX then you will have to 1) 'make distclean' to remove the old
     configuration, 2) 'cd tools; ./configure.sh sam4e-ek/ksnh' to start
     with a fresh configuration, and 3) perform the configuration changes
     above.

     Also, make sure that your PATH variable has the new path to your
     Atmel tools.  Try 'which arm-none-eabi-gcc' to make sure that you
     are selecting the right tool.  setenv.sh is available for you to
     use to set or PATH variable.  The path in the that file may not,
     however, be correct for your installation.

     See also the "NOTE about Windows native toolchains" in the section call
     "GNU Toolchain Options" above.

  Configuration sub-directories
  -----------------------------

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

    NOTES:

    1. NSH built-in applications are supported.  However, there are
       no built-in applications built with the default configuration.

       Binary Formats:
         CONFIG_BUILTIN=y                    : Enable support for built-in programs

       Applicaton Configuration:
         CONFIG_NSH_BUILTIN_APPS=y           : Enable starting apps from NSH command line

    2. This configuration has the network enabled by default.  This can be
       easily disabled or reconfigured (See see the network related
       configuration settings above in the section entitled "Networking").

       NOTE: In boot-up sequence is very simple in this example; all
       initialization is done sequential (vs. in parallel) and so you will
       not see the NSH prompt until all initialization is complete.  The
       network bring-up in particular will add some delay before the NSH
       prompt appears.  In a real application, you would probably want to
       do the network bringup on a separate thread so that access to the
       NSH prompt is not delayed.

       This delay will be especially long if the board is not connected to
       a network because additional time will be required to fail with
       timeout errors.

    3. This configuration supports a network with fixed IP address.  You
       may have to change these settings for your network:

       CONFIG_NSH_IPADDR=0x0a000002        : IP address: 10.0.0.2
       CONFIG_NSH_DRIPADDR=0x0a000001      : Gateway:    10.0.0.1
       CONFIG_NSH_NETMASK=0xffffff00       : Netmask:    255.255.255.0

       You can also enable enable the DHCPC client for networks that use
       dynamically assigned address:

       CONFIG_NETUTILS_DHCPC=y             : Enables the DHCP client
       CONFIG_NET_UDP=y                    : Depends on broadcast UDP
       CONFIG_NET_BROADCAST=y
       CONFIG_NSH_DHCPC=y                  : Tells NSH to use DHCPC, not
                                           : the fixed addresses

    4. This configuration has the DMA-based SPI0 and AT25 Serial FLASH
       support enabled by default.  This can be easily disabled or
       reconfigured (See see the configuration settings and usage notes
       above in the section entitled "AT25 Serial FLASH").

    5. This configuration has been used for verifying the touchscreen on
       on the SAM4E-EK LCD.  With these modifications, you can include the
       touchscreen test program at apps/examples/touchscreen as an NSH built-in
       application.  You can enable the touchscreen and test by modifying the
       default configuration in the following ways:

          Device Drivers
            CONFIG_SPI=y                      : Enable SPI support
            CONFIG_SPI_EXCHANGE=y             : The exchange() method is supported
            CONFIG_SPI_OWNBUS=y               : Smaller code if this is the only SPI device

            CONFIG_INPUT=y                    : Enable support for input devices
            CONFIG_INPUT_ADS7843E=y           : Enable support for the XPT2046
            CONFIG_ADS7843E_SPIDEV=2          : Use SPI CS 2 for communication
            CONFIG_ADS7843E_SPIMODE=0         : Use SPI mode 0
            CONFIG_ADS7843E_FREQUENCY=1000000 : SPI BAUD 1MHz
            CONFIG_ADS7843E_SWAPXY=y          : If landscpe orientation
            CONFIG_ADS7843E_THRESHX=51        : These will probably need to be tuned
            CONFIG_ADS7843E_THRESHY=39

          System Type -> Peripherals:
            CONFIG_SAM34_SPI0=y                : Enable support for SPI

          System Type:
            CONFIG_GPIO_IRQ=y                 : GPIO interrupt support
            CONFIG_GPIOA_IRQ=y                : Enable GPIO interrupts from port A

          RTOS Features:
            CONFIG_DISABLE_SIGNALS=n          : Signals are required

          Library Support:
            CONFIG_SCHED_WORKQUEUE=y          : Work queue support required

          Applicaton Configuration:
            CONFIG_EXAMPLES_TOUCHSCREEN=y     : Enable the touchscreen built-int test

          Defaults should be okay for related touchscreen settings.  Touchscreen
          debug output on UART0 can be enabled with:

          Build Setup:
            CONFIG_DEBUG=y                    : Enable debug features
            CONFIG_DEBUG_VERBOSE=y            : Enable verbose debug output
            CONFIG_DEBUG_INPUT=y              : Enable debug output from input devices

    6. Enabling HSMCI support. The SAM3U-KE provides a an SD memory card
       slot.  Support for the SD slot can be enabled with the following
       settings:

       System Type->ATSAM3/4 Peripheral Support
         CONFIG_SAM34_HSMCI=y                 : Enable HSMCI support
         CONFIG_SAM34_DMAC0=y                 : DMAC support is needed by HSMCI

       System Type
         CONFIG_SAM34_GPIO_IRQ=y              : PIO interrupts needed
         CONFIG_SAM34_GPIOA_IRQ=y             : Card detect pin is on PIOA

       Device Drivers -> MMC/SD Driver Support
         CONFIG_MMCSD=y                       : Enable MMC/SD support
         CONFIG_MMSCD_NSLOTS=1                : One slot per driver instance
         CONFIG_MMCSD_HAVECARDDETECT=y        : Supports card-detect PIOs
         CONFIG_MMCSD_SDIO=y                  : SDIO-based MMC/SD support
         CONFIG_SDIO_DMA=y                    : Use SDIO DMA
         CONFIG_SDIO_BLOCKSETUP=y             : Needs to know block sizes

       Library Routines
         CONFIG_SCHED_WORKQUEUE=y             : Driver needs work queue support

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y                : NSH board-initialization

    STATUS:
      2014-3-13: The basic NSH serial console is working.  Network support
        has been verified.  HSMCI and touchscreen have not been tested (the
        above notes came from the SAM3U-EK and have not been yet been tested
        on the SAM4E-EK).
      2014-3-14: The DMA-based SPI appears to be functional and can be used
        to support a FAT file system on the AT25 Serial FLASH.
