CsrUsbSpiDevice-Tiva
===
Description
---
This is a reverse engineered re-implementation of CSR's USB<->SPI Converter on a TI Stellaris Launchpad. It is compatible with CSR's own drivers and BlueSuite tools, and should work on any BlueCore chip that supports programming through SPI.

Original version by Frans-Willem for original Stellaris Launchpad - https://github.com/Frans-Willem/CsrUsbSpiDeviceRE

Minor tweaks to build under TivaWare Launchpad part # EK-TM4C123GXL by Richard Aplin, Dec2013
Turbo mode (which may or may not work for you; YMMV) by Rich also.

Disclaimer
---
I make no guarantees about this code. For me it worked, but it might not for you. If you break your BlueCore module or anything else by using this software this is your own responsibility.

How to use
---
* Get a TI Tiva C Launchpad (EK-TM4C123GXL, $12 at time of writing)

* Connect your BlueCore module. 3.3v -> 3.3v, GND -> GND, PC4 -> SPI_CSB, PC5 -> SPI_MOSI, PC6 -> SPI_MISO, PC7 -> SPI_CLK. (PC4 - PC7 are located on the second column from the right)

EITHER Build from source:
* Install from TI site: TivaWare, TI's Code Composer (free with the launchpad)
* Import this project into Code composer as usual.
* Ensure SW_ROOT is set to your TivaWare directory in *both* of these locations:
    * Project -> Properties -> Resource -> Linked Resources -> Path Variables -> SW_ROOT
    * Window -> Preferences -> C/C++ -> Build -> Environment -> SW_ROOT.
* Build using Project -> Build Project
* Connect your Tiva C Launchpad with the microUSB port at the top (next to power select), and set power select to DEBUG.
* Debug using Run -> Debug
* When paused on main, do Run -> Resume

OR just flash the supplied binary using LM Flash Programmer:
* Run LM Flash Programmer (in TI toolkit), plug in Launchpad
* On "Configuration" tab select TM4C123G Launchpad
* On "Program" tab select the bin file Release\CSR_USB-SPI_TivaC_Launchpad.bin (included in this project)
* Hit Program, you're done.

FINALLY 
* Plug in "device" microUSB port to a host computer with CSR BlueSuite (or Bluelab) installed.
* Device should be recognized, Drivers can be found at csrsupport.com (needs registration), underBrowse category tree -> Bluetooth PC Software/Tools -> USB-SPI Converter.
* Use any of the BlueSuite or BlueLabs tools to play with your bluecore module! (BlueSuite can be found on CSRSupport.com under Browser category tree -> Bluetooth PC Software/Tools -> Current BlueSuite Development Tools)
* Green led will light up during reading, Red during writing, Blue when using BCCMD's.

Optional Turbo mode
---
This is a hack. If you hold down the LEFT button on the launchpad when booting (i.e. when you first plug it in to USB after programming, or when hitting reset button) the SPI interface will run without any delays for flash page read/writes. This makes it *much* faster - standard mode takes 65sec to read 8mbit, turbo mode takes 25 seconds. If you want to use this speedup, you should ensure it works reliably on your CSR device; use BlueFlash to "dump" whatever is in flash, then Erase the whole chip, then reprogram from the dump. 
Turbo mode works fine for me (on a BlueCore4-EXT) but YMMV.

Notes
---
* The SPI clock may not be accurate (certainly not in Turbo mode)
* The code was written to match CSR's original firmware as closely as possible, it might not be the most efficient way to things on the Stellaris.
* If something does or does not work, let me know (Frans-Willem)

TODO
---
* Clean up: The code works, but should still be cleaned up, and some bits aren't fully reverse engineered yet (e.g. the BCCMD timeout)
* Refactor/rewrite: The code as it is now is as close to CSR's code as I could get, but it's not the most readable. I intend to refactor this at some point (maybe in a different project) to make it more readable and portable to other platforms.

Original code - and all the hard work - by Frans-Willem Dec2012; minor hacks and porting to TivaWare by Richard Aplin, Dec2013
