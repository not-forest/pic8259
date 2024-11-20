//! A small module represents the bios port for debug codes.

use x86_64::instructions::port::PortWriteOnly;

/// A Debug Board for POST codes.
/// 
/// While the system is booting, the BIOS will output a series of debug codes to I/O port 0x80. 
/// These are indended for debugging a non-booting system. In most desktop PCs, you can install
/// a POST code debug board, which is basically a small PCI (or ISA) slot board that decodes 
/// I/O writes to I/O port 0x80 and displays the value via 7-segment LEDs.
/// 
/// This port is used as a small delay generator, which provides a software little sleep(). It
/// could be used like that in places when a small delay is needed, but it must be really tiny.
/// 
/// For real debug codes, visit: http://www.bioscentral.com.
pub fn post_debug_delay() {
    unsafe { PortWriteOnly::<u32>::new(0x80).write(0) }
} // Public for possible external use.
