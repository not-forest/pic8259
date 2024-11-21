#![no_std]
//! Support for the 8259 Programmable Interrupt Controller, which handles basic I/O interrupts.  
//! In multicore mode, we would apparently need to replace this with an APIC interface.
//!
//! A single PIC handles up to eight vectored priority interrupts for the CPU. By cascading 8259
//! chips, we can increase interrupts up to 64 interrupt lines, however we only have two chained
//! instances that can handle 16 lines. Can be programmed either in edge triggered, or in level
//! triggered mode. PIC uses CHANNEL0 from the PIT (Programmable Interval Timer), which's frequency
//! can be adjusted based on it's configuration. Individual bits of IRQ register within the PIC can
//! be masked out by the software.
//!
//! The basic idea here is that we have two PIC chips, PIC1 and PIC2, and that PIC2 is slaved to 
//! interrupt 2 on PIC 1. You can find the whole story at http://wiki.osdev.org/PIC (as usual).

mod commands;
mod chip;
mod regs;
mod post;

use bitflags::Flags;
use commands::*;
use chip::*;

pub use regs::*;
use x86_64::structures::idt::InterruptStackFrame;

/// **Operation Mode for PIC Controller**.
///
/// PIC supports several operation mode, most of which are most likely to be ignored on x86
/// architecture, however some of them can be used to obtain some interesting results. See more
/// information for each of them below.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PicOperationMode {
    /// Fully Nested Mode (Default Mode)
    ///
    /// This mode is entered after intiialization unless another mode is programmed. The interrupt
    /// requests are ordered in priority from 0 through 7, where 0 is the highest priority. When
    /// interrupt is acknowledged the highest priority interrupt will be issued before the rest.
    ///
    /// On a regular x86 PC with two chained PICs, the IRQ0, which is an output of a PIT timer,
    /// will be handled before others IRQ lines. Slave PIC has even smaller priority than first 7
    /// masters IRQ lines, because it is mapped after the master PIC.
    ///
    /// # Use Case
    ///
    /// Use when the default priority level suits your needs. For example, if a PS/2 keyboard interrupt
    /// (IRQ1) will be always services before the real time clock (IRQ8).
    FullyNested,
    /// Automatic Rotation Mode (Equal Priority Mode)
    ///
    /// Rotates the priority by the value specified in the current highest priority interrupt
    /// within the ISR register. Basically each time the highest priority interrupt occur, it will 
    /// then be defined as a lowest priority interrupt. This way giving all interrupt sources an
    /// equal service time from the CPU.
    ///
    /// # Use Case
    ///
    /// Use if you think that all interrupts deserve to be handled equally. This sometimes might
    /// cause troubles with timers, specifically the PIT and RTC.
    AutomaticRotation,
    /// Special Mask Mode (Manual Priority Mode)
    ///
    /// Some applications might want to have a different priority mapping for the full software
    /// control over the sequence of interrupts. During this mode the mask register is now used to 
    /// temporarly disable certain interrupt levels (not interrupt lines) as well as manually
    /// changing the priority level.
    ///
    /// # Use Case
    ///
    /// Critical sections that wish to disable some interrupts from the PIC but not all of them, or
    /// some applications with specific timing requirements that require to temporarly inhibit some
    /// of interrupt levels to make sure that lower priority interrupts will meet timings accordigly.
    SpecialMask(IrqMask),
    /// Polled Mode (No interrupts)
    ///
    /// Do not use interrupts to obtain information from the peripherals but only listen for
    /// upcoming changes. After the polled mode is enabled, data bus will provide a binary value of a
    /// highest priority issued interrupt. Each read from the data port will be treated as an
    /// interrupt acknowledge.
    ///
    /// # Use Case
    ///
    /// Probably the most useless one. Since it is very quick to turn this mode on and off, it can
    /// be used to handle several interrupts in one handler by reading all values from the data
    /// port until it will be equal to zero.
    PolledMode,
}

/// A x86 setup of **Chained PICs**.
///
/// In most PCs there are one master and one slace PIC configuration, each having 8 inputs
/// servicing 16 interrupts. This structure allows to easily initialize and control the x86
/// configuration of PICs and configure all 16 interrupts for further handling.
///
/// Provides a minimal set of functions required to properly handle interrupts based on the
/// currently used mode for each PIC.
pub struct ChainedPics {
    initialized: bool,
    pub master: Pic,
    pub slave: Pic,
}

impl ChainedPics {
    /// Creates a new instance of Chained Pics.
    /// 
    /// The master offset and slave offset are two offsets that are pointing to the first
    /// interrupt vector of each 8259 chip.
    /// 
    /// # Panics
    /// 
    /// This function will panic if the provided offsets will overlap with each other or
    /// collide with CPU exceptions.
    pub const fn new(master_offset: u8, slave_offset: u8) -> Self {
        assert!(master_offset >= 32 || slave_offset >= 32, "Both master and slave offsets must not overlap with CPU exceptions.");
        assert!(master_offset.abs_diff(slave_offset) >= 8, "The master and slave offsets are overlapping with each other.");

        unsafe { Self::new_unchecked(master_offset, slave_offset) }
    }

    /// Creates a new instance of a Chained Pics.
    /// 
    /// The offset must point to the the chosen 16 entries from the IDT that will be used 
    /// for the software interrupts.
    /// 
    /// This is a convenience function that maps the PIC1 and PIC2 to a
    /// contiguous set of interrupts. This function is equivalent to
    /// `Self::new(primary_offset, primary_offset + 8)`.
    ///
    /// # Panics
    /// 
    /// This function will panic if the provided offset will overlap with cpu exceptions. It
    /// will always prevent the overlapping between master and slave chips, because it makes
    /// an offset for them sequentially.
    pub const fn new_contiguous(primary_offset: u8) -> Self {
        Self::new(primary_offset, primary_offset + 8)
    }

    /// Returns true if initialized at least once.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Initializes the PICs.
    ///
    /// This performs an initialization that is compatible with most x86 PC devices. Some archaic
    /// devices may use only one PIC. For such possibilities a manual initialization of PIC
    /// structures must be performed.
    ///
    /// # Automatic Interrupts (Both Chips)
    ///
    /// With this flag enabled PIC will automatically perform a EOI operation at the trailing edge of the
    /// last interrupt acknowledge pulse from the CPU. This setting can only be used with a single
    /// master chip. Basically that means that the end of interrupt command is not necessary after
    /// a corresponding handler function handles the interrupt, however it does not work well with
    /// chained PICs.
    pub fn initialize(&mut self, automatic_interrupts: bool) {
        unsafe {
            self.master.init(
                PicIRQMapping::Master(Some(ICW3_MASTER::SLAVE2)), 
                automatic_interrupts,
            );
            self.slave.init(
                PicIRQMapping::Slave(ICW3_SLAVE::MASTER2), 
                automatic_interrupts,
            );
        }
        if !self.is_initialized() { self.initialized = true }
    }

    /// Checks if the provided interrupt vector was caused by PIC properly.
    ///
    /// When an IRQ occurs, the PIC chip tells the CPU (via. the PIC's INTR line) that there's an interrupt, 
    /// and the CPU acknowledges this and waits for the PIC to send the interrupt vector. This creates a race 
    /// condition: if the IRQ disappears after the PIC has told the CPU there's an interrupt but before the 
    /// PIC has sent the interrupt vector to the CPU, then the CPU will be waiting for the PIC to tell it 
    /// which interrupt vector but the PIC won't have a valid interrupt vector to tell the CPU.
    ///
    /// Here we read if the interrupt is written within the ISR register, to be sure that we are
    /// dealing with real interrupt. If a spurious interrupt was also sent from the slave PIC, the
    /// master shall clear this flag, because he also thinks that it was a legit IRQ.
    ///
    /// # Important
    ///
    /// This is also the reason why sometimes an unimplemented handler functions are causing general protection 
    /// faults. PIC will cause an interrupt on the lowest priority IRQ, and the interrupt service
    /// routine for something like hard disk controller is most likely not implemented in the stage
    /// of configuring PICs.
    ///
    /// # Note (Fully Nested Mode)
    ///
    /// Spurious interrupts can only happen when the lowest priority IRQ are called. The fake interrupt number 
    /// is the lowest priority interrupt number for the corresponding PIC chip (IRQ 7 for the master PIC, and 
    /// IRQ 15 for the slave PIC).
    ///
    /// **Basically this means that you shall only check for spurious IRQs when it is a parallel
    /// port interrupt (IRQ7) or secondary ATA channel interrupt (IRQ15)**
    ///
    /// # Note (Rotations)
    ///
    /// When modes with rotations are used: [´PicOperationMode::AutomaticRotation´],
    /// [´PicOperationMode::SpecialMask´], the lowest priority priority IRQ is changed in time. For
    /// such configurations, calling this function on every interrupt caused by PIC is probably
    /// fine.
    ///
    /// # Unsafe 
    ///
    /// This function is only unsafe as it shall be only used within the interrupt handler function
    /// at the very start, to make sure that we are not handling a spurious interrupt. It is
    /// completely forbidden to send an end of interrupt after this function. 
    pub unsafe fn is_spurious(&mut self, vec_id: u8) -> bool {
        if self.slave.is_spurious(vec_id) { self.master.end_of_interrupt(); true } 
        else if self.master.is_spurious(vec_id) { true } 
        else { false }
    }

    /// Notify a proper PIC chip that the interrupt was succesfully handled and shall be cleared
    /// within the ISR register.
    ///
    /// # Important
    ///
    /// To prevent spurious interrupts on lowest priority IRQs, use [´ChainedPics::is_spurious´]
    /// and jump to the end of interrupt handler function if it returns true. If some interrupt was
    /// caused by a hardware|software mistake, it should not be handled.
    ///
    /// **PIC must not receive a EOI command, when it is a spurious interrupts. It will prevent
    /// other interrupts from being handled, which is a bigger trouble.**
    ///
    /// Lower priority interrupts vary based on the current mode. The function mentioned above
    /// handles all logic required for each.
    ///
    /// # Unsafe 
    ///
    /// This command must be used at the end of every interrupt that was issued by any of two PICs.
    /// Make sure that this is a last command withon the interrupt service routine.
    ///
    /// # Note
    ///
    /// Does nothing on chips with automatic interrupts flag enabled. See more in [´Pic´]
    pub unsafe fn notify_end_of_interrupt(&mut self, vec_id: u8) {
        if self.slave.handles_interrupt(vec_id) {
            self.slave.end_of_interrupt();
            self.master.end_of_interrupt();
        } else
        if self.master.handles_interrupt(vec_id) {
            self.master.end_of_interrupt();
        }
    }

    /// Disable both PICs interrupts.
    ///
    /// # Note
    ///
    /// This must be used when switching to APIC controller for handling interrupts. This is also
    /// mandatory even if the chip was never initialized by the OS.
    pub fn disable(&mut self) {
        unsafe { self.write_mask(IrqMask::all()) };
    }

    /// Gets the current IRQ mask.
    pub fn get_mask(&mut self) -> IrqMask {
        IrqMask::from_bits_truncate(
            u16::from_le_bytes([
                self.master.mask_read().bits(), self.slave.mask_read().bits()
            ])
        )
    }

    /// Masks the IRQ lines of chained PICs.
    ///
    /// # Unsafe
    ///
    /// Even though masking just disabled some interrupt lines, this function is marked as unsafe
    /// due to undefined behavior that might happen when the OCW1 command is not right.
    pub unsafe fn write_mask(&mut self, mask: IrqMask) {
        let bytes = mask.bits().to_le_bytes();
        unsafe {
            self.master.mask_write(OCW1::from_bits_truncate(bytes[0]));
            self.slave.mask_write(OCW1::from_bits_truncate(bytes[1]));
        }
    }

    /// Creates a new instance of PIC controller.
    /// 
    /// The master offset and slave offset are two offsets that are pointing to the first
    /// interrupt vector of each 8259 chip.
    /// 
    /// # Unsafe
    /// 
    /// This function will not check if the chosen offsets overlap with each other or do they
    /// overlap with CPU exceptions.
    pub const unsafe fn new_unchecked(master_offset: u8, slave_offset: u8) -> Self {
        Self {
            initialized: false,
            master: Pic::new(master_offset, 0x20, 0x21),
            slave: Pic::new(slave_offset, 0xa0, 0xa1),
        }
    }

    /// Reads the interrupt masks of both PICs.
    #[deprecated(since = "1.0.0", note = "Use [´get_mask´] to get a convenient 16-bit [´IrqMask´] structure instead.")]
    pub unsafe fn read_masks(&mut self) -> [u8; 2] {
        [self.master.mask_read().bits(), self.slave.mask_read().bits()]
    }
    
    /// Writes the interrupt masks of both PICs.
    #[deprecated(since = "1.0.0", note = "Use [´set_mask´] to apply the mask conveniently via [´IrqMask´] structure.")]
    pub unsafe fn write_masks(&mut self, mask1: u8, mask2: u8) {
        self.master.mask_write(OCW1::from_bits_truncate(mask1));
        self.slave.mask_write(OCW1::from_bits_truncate(mask2));
    }
}

bitflags::bitflags! {
    /// IRQ Flags for 16 PIC Interrupts.
    ///
    /// These represent the 16 possible IRQ lines that the PIC can handle. Each line corresponds to a specific hardware 
    /// interrupt source.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct IrqMask: u16 {
        /// **IRQ0** - System timer interrupt.
        /// Triggered by the system timer (PIT). Essential for task switching and system ticks.
        const IRQ0_TIMER = 1 << 0;

        /// **IRQ1** - PS/2 Keyboard interrupt.
        /// Generated when a key is pressed or released on the primary keyboard.
        const IRQ1_PS2_KEYBOARD = 1 << 1;

        /// **IRQ3** - Serial port 2 (COM2) interrupt.
        /// Triggered by activity on the second serial port.
        const IRQ3_SERIAL_PORT2 = 1 << 3;

        /// **IRQ4** - Serial port 1 (COM1) interrupt.
        /// Triggered by activity on the first serial port.
        const IRQ4_SERIAL_PORT1 = 1 << 4;

        /// **IRQ5** - Parallel port 2 interrupt (or sound card).
        /// Often used for parallel port 2, but may be reassigned to other devices like a sound card.
        const IRQ5_PARALLEL_PORT2 = 1 << 5;

        /// **IRQ6** - Diskette drive (floppy disk controller) interrupt.
        /// Used for floppy disk read/write operations.
        const IRQ6_DISKETTE_DRIVE = 1 << 6;

        /// **IRQ7** - Parallel port 1 interrupt.
        /// Commonly associated with parallel port 1, typically used for printers.
        const IRQ7_PARALLEL_PORT1 = 1 << 7;

        /// **IRQ8** - Real-Time Clock (RTC) interrupt.
        /// Generated by the RTC for timekeeping purposes.
        const IRQ8_RTC = 1 << 8;

        /// **IRQ9** - CGA vertical retrace interrupt (or general use).
        /// Historically used for CGA video cards. Now typically available for general-purpose use.
        const IRQ9_CGA_VERTICAL_RETRACE = 1 << 9;

        /// **IRQ10** - Free for general-purpose use (first available line).
        /// Not assigned to specific hardware by default.
        const IRQ10_FREE_1 = 1 << 10;

        /// **IRQ11** - Free for general-purpose use (second available line).
        /// Not assigned to specific hardware by default.
        const IRQ11_FREE_2 = 1 << 11;

        /// **IRQ12** - PS/2 Mouse interrupt.
        /// Triggered by activity on the PS/2 mouse.
        const IRQ12_PS2_MOUSE = 1 << 12;

        /// **IRQ13** - Floating Point Unit (FPU) interrupt.
        /// Used for floating-point arithmetic errors or related conditions.
        const IRQ13_FPU = 1 << 13;

        /// **IRQ14** - Primary ATA channel interrupt.
        /// Handles interrupts from devices on the primary ATA (IDE) bus, such as the main hard drive.
        const IRQ14_PRIMARY_ATA = 1 << 14;

        /// **IRQ15** - Secondary ATA channel interrupt.
        /// Handles interrupts from devices on the secondary ATA (IDE) bus, such as additional drives.
        const IRQ15_SECONDARY_ATA = 1 << 15;
    }
}
