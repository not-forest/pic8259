//! Defines a single configurable PIC chip.
//!
//! For most systems it is not necessary to manually configure each individual PIC. Please use
//! [´ChainedPics´] for convenient structure to manipulate on chained set of PICs located on the
//! x86 architecture.

use x86_64::instructions::port::Port;
use crate::post::post_debug_delay;
use crate::regs::*;
use crate::*;

/// Defines the PIC IRQ mappings (hardwired lines) for the PIC controller.
///
/// The PIC can be configured either as a master or a slave device. This will change the upcoming
/// ICW3 command during the initialization.
///
/// The master shall define it's IRQ line on which the slave is connected. The slave shall define 
/// it's IRQ line on which it is connected to the master. Only one master can be used in the whole
/// connection. This allows up to 64 IRQ lines, when 8 slaves are 
///
/// # x86
///
/// For regular PC systems two PICs are chained together in a MASTER <-> SLAVE pair, allowing to
/// use up to 16 different IRQs.
#[derive(Debug, Clone, Copy)]
pub enum PicIRQMapping {
    /// Master PIC chip mapping.
    ///
    /// Defines all interrupt request bits on which the slave PIC is connected. 
    Master(Option<ICW3_MASTER>),
    /// Slave PIC chip mapping.
    ///
    /// Defines one IRQ interrupt request line, which it is connected to.
    Slave(ICW3_SLAVE),
}

/// **PIC Chip** - Programmable Interrupt Controller.
/// 
/// In x86 this can be either master or slave chip. Each chip has it's command port and data port.
/// The offset is used to handle different interrupt events.
#[derive(Debug)]
pub struct Pic {
    /// The base offset to which our interrupts are mapped.
    pub offset: u8,
    /// Current operation mode for this specific PIC.
    op_mode: PicOperationMode,
    /// Automatic EOI flags.
    automatic_interrupts: bool,
    /// The processor I/O port on which we send commands.
    command: Port<u8>,
    /// The processor I/O port on which we send and receive data.
    data: Port<u8>,
}

impl Pic {
    /// Creates a new instance of a PIC chip.
    ///
    /// According to the datasheet, PIC should be fully reinitialized with all four initialization
    /// words again to exit this mode.
    ///
    /// # Offset
    ///
    /// Offset must be provided to not collide with x86 exceptions. Changing an offset means
    /// reinitializing the PIC again completely from the very start. When several PICs are used,
    /// their offsets shall not collide with each others. 
    ///
    /// # Warn
    ///
    /// This does not include the initialization of the PIC chip. Use the [´Pic::init´] function to
    /// perform a proper initialization.
    /// 
    /// # Note
    /// 
    /// This only creates a single PIC chip. Classic x86/x64 PC includes 2 chained PICs within. This function
    /// is a public API only due to a possibility of only one PIC chip on some really archaic PC/XT systems.
    pub const unsafe fn new(offset: u8, command_port: u16, data_port: u16) -> Self {
        Self {
            offset,
            op_mode: PicOperationMode::FullyNested, // Used after initialization.
            automatic_interrupts: false,
            command: Port::new(command_port),
            data: Port::new(data_port),
        }
    }

    /// Initialization of the PIC controller.
    ///
    /// All initialization words shall be provided to the controller in a very strict order:
    /// - ICW1 → command port;
    /// - ICW2 → data port;
    /// - ICW3 → command port, IF (ICW1 bit D1 == 0) ELSE ignored;
    /// - ICW4 → data port;
    ///
    /// For swapping to a different configuration this whole process must be repeated from the very
    /// start. After this function OCW commands can be sent to the PIC controller. 
    ///
    /// For using PICs configuration on regular x86 PCs, use [´ChainedPics´] structure, which provides 
    /// even safer function.
    ///
    /// # PIC Mapping
    ///
    /// This desides if the PIC is a master or a slave device. It also defines which lines are
    /// connected to which depending on it's value.
    ///
    /// # Automatic Interrupts
    ///
    /// With this flag enabled PIC will automatically perform a EOI operation at the trailing edge of the
    /// last interrupt acknowledge pulse from the CPU. This setting can only be used with a single
    /// master chip. Basically that means that the end of interrupt command is not necessary after
    /// a corresponding handler function handles the interrupt, however it does not work well with
    /// chained PICs.
    ///
    /// Note that from a system standpoint, this mode should be used only when a nested multilevel interrupt 
    /// structure is not required within a single 8259A.
    pub unsafe fn init(&mut self, pic_map: PicIRQMapping, automatic_interrupts: bool) {
        unsafe {
            // Saving the values that was before the data change.
            let mask = self.data.read();
            // Generating initialization commands based on the chosen operation mode.
            let icw1 = 
                ICW1::IC4 | 
                match pic_map {
                    // Using single mode when only one chip is presented.
                    PicIRQMapping::Master(opt) => if opt.is_none() { 
                        ICW1::SNGL 
                    } else { ICW1::empty() },
                    _ => ICW1::empty(),
                };
            // Only implementing the x86 compatible version here.
            let icw2 = self.offset & ICW2::INTERRUPT_VECTOR_ADDRESS_8086_8088.bits();
            // In most PC systems two PICs are used. One PIC or more than two is also allowed.
            let icw3 = pic_map;
            // In x86 systems only this bit is used.
            let icw4 = 
                ICW4::MPM | 
                if automatic_interrupts { ICW4::AEOI } else { ICW4::empty() };

            // A short delay is required between each write due to the slowness of the controller.
            /* ICW1 command. */
            self.command.write(icw1.bits());
            post_debug_delay();
            /* ICW2 command. */
            self.data.write(icw2);
            post_debug_delay();
            /* ICW3 command. (If some) */
            match icw3 {
                // Master might be alone or in a chained configuration.
                PicIRQMapping::Master(opt) => opt.map(|some| { 
                    self.command.write(some.bits());
                    post_debug_delay();
                }).unwrap_or(()),
                // If slave, at least one more chip should exist.
                PicIRQMapping::Slave(some) => {
                    self.command.write(some.bits());
                    post_debug_delay();
                },
            }
            /* ICW4 command. */
            self.data.write(icw4.bits());
            post_debug_delay();

            /* OCW1 command. */
            self.data.write(mask);
        }
    }

    /// Checks if the provided IRQ id from the IDT matches this PIC.
    ///
    /// Each PIC may only handle up to 8 interrupts.
    pub fn handles_interrupt(&self, interrupt_id: u8) -> bool {
        (self.offset..self.offset + 8).contains(&interrupt_id)
    }

    /// Reads the value of current operation mode used on this PIC.
    pub fn operation_mode_current(&self) -> PicOperationMode {
        self.op_mode
    }

    /// Changes the current operation mode of this PIC.
    ///
    /// This sends the OCW2 command and configurest the current operation mode of the PIC logic.
    /// Refer to [´PicOperationMode´] enum for more details.
    ///
    /// # Warn.
    ///
    /// When switching from polled mode a mask must be restored to the previously used one.
    pub fn operation_mode_change(&mut self, new_op_mode: PicOperationMode) {
        use PicOperationMode::*;

        unsafe {
            /* Default behaviour when switching the mode. */
            let fully_nested_arm = |s: &mut Self| // Restoring the disturbed fully nested structure. 
                s.set_lowest_priority(7);
            let automatic_rotation_arm = |s: &mut Self| 
                if s.automatic_interrupts {
                    s.command.write(OCW2::ROTATE_IN_AUTOMATIC_EOI_MODE_SET.bits());
                };
            let special_mask_arm = |s: &mut Self| s.command.write(OCW3::SET_SPECIAL_MASK.bits());
            let polled_mode_arm = |s: &mut Self| {
                s.mask_write(OCW1::all());
                s.command.write(OCW3::POLL.bits());
            };

            match self.op_mode {
                FullyNested => match new_op_mode {
                    FullyNested => return,
                    AutomaticRotation => automatic_rotation_arm(self),
                    SpecialMask => special_mask_arm(self),
                    PolledMode => polled_mode_arm(self),
                },
                AutomaticRotation => {
                    if self.automatic_interrupts {
                        self.command.write(OCW2::ROTATE_IN_AUTOMATIC_EOI_MODE_CLEAR.bits());
                    }
                    match new_op_mode {
                        FullyNested => fully_nested_arm(self),
                        AutomaticRotation => return,
                        SpecialMask => special_mask_arm(self),
                        PolledMode => polled_mode_arm(self),
                    }
                },
                SpecialMask => {
                    self.command.write(OCW3::RESET_SPECIAL_MASK.bits());
                    match new_op_mode {
                        FullyNested => fully_nested_arm(self),
                        AutomaticRotation => automatic_rotation_arm(self),
                        SpecialMask => return,
                        PolledMode => polled_mode_arm(self)
                    }
                },
                PolledMode => match new_op_mode {
                    FullyNested => fully_nested_arm(self),
                    AutomaticRotation => automatic_rotation_arm(self),
                    SpecialMask => special_mask_arm(self),
                    PolledMode => return,
                },
            }
        }
        self.op_mode = new_op_mode;
    }

    /// Checks if the provided interrupt vector was caused by PIC properly.
    ///
    /// When an IRQ occurs, the PIC chip tells the CPU (via. the PIC's INTR line) that there's an interrupt, 
    /// and the CPU acknowledges this and waits for the PIC to send the interrupt vector. This creates a race 
    /// condition: if the IRQ disappears after the PIC has told the CPU there's an interrupt but before the 
    /// PIC has sent the interrupt vector to the CPU, then the CPU will be waiting for the PIC to tell it 
    /// which interrupt vector but the PIC won't have a valid interrupt vector to tell the CPU.
    ///
    /// Basically if the ISR bit for this flag is not set, but the interrupt service routine was
    /// executed, that means it is spurious and interrupt must end right away. 
    ///
    /// # Unsafe 
    ///
    /// This function is only unsafe as it shall be only used within the interrupt handler function
    /// at the very start, to make sure that we are not handling a spurious interrupt. It is
    /// completely forbidden to send an EOI, if this function evaluates to true! 
    pub unsafe fn is_spurious(&mut self, id: u8) -> bool {
        assert!(id >= 32 && self.offset + 8 > id, "The provided interrupt vector is outside of scope of this PIC chip."); 

        let irq = ISR::from_bits_truncate(1 << id.saturating_sub(self.offset));
        !self.read_isr().contains(irq)
    }

    /// Reads the value of the ISR.
    /// 
    /// The interrupt status register inside the PIC chip, shows the info about which interrupts are
    /// being serviced at that moment. The value will be flushed after the end_of_interrupt method.
    ///
    /// # Note
    ///
    /// Always 0x0 in polled mode.
    pub fn read_isr(&mut self) -> ISR {
        unsafe {
            // ISR is guaranteed to be empty during in polled mode.
            if self.op_mode == PicOperationMode::PolledMode { ISR::empty() } else {
                self.command.write(OCW3::READ_REG_ISR.bits());
                ISR::from_bits_truncate(self.command.read())
            }
        }
    }

    /// Reads the value of the IRR.
    /// 
    /// The interrupt request register shows the requested interrupts that have been raised
    /// but are not being acknowledged yet. The value will be flushed after the end_of_interrupt method.
    pub fn read_irr(&mut self) -> IRR {
        unsafe {
            self.command.write(OCW3::READ_REG_IRR.bits());
            let irr = IRR::from_bits_truncate(self.command.read());
            // Polled mode will override the register read. The same goes vice-versa.
            if self.op_mode == PicOperationMode::PolledMode {
                self.command.write(OCW3::POLL.bits())
            }
            irr
        }
    }

    /// Read the current PIC mask.
    ///
    /// The mask defines all IRQ lines, that shall be ignored and not sent to the CPU. 
    pub fn mask_read(&mut self) -> OCW1 {
        unsafe {
            OCW1::from_bits_truncate(self.data.read())
        }
    }

    /// Poll the interrupt with highest priority.
    ///
    /// The value returned is a binary code of the highest priority level requesting service. Will
    /// return None if the current mode is not [´PicOperationMode::PolledMode´].
    ///
    /// # Note
    ///
    /// The interrupt is immediately acknowledged after the first read, according to the datasheet:
    /// When poll command is issued, the 8259 treats the next RD pulse as an interrupt acknowledge.
    pub fn poll(&mut self) -> Option<u8> {
        match self.op_mode {
            PicOperationMode::PolledMode => unsafe { Some(self.command.read()) },
            _ => None
        }
    }

    /// Masks the requested IRQ lines.
    ///
    /// Sends the OCW1 command and masks unused IRQ lines.
    ///
    /// # Huge Warn
    ///
    /// On special mask mode, this inhibits the priority level, not masks the interrupts
    /// completely. See more info in [´PicOperationMode::SpecialMask´]
    ///
    /// # Unsafe
    ///
    /// Even though masking just disabled some interrupt lines, this function is marked as unsafe
    /// due to undefined behavior that might happen when the OCW1 command is not right.
    pub unsafe fn mask_write(&mut self, ocw1: OCW1) {
        self.data.write(ocw1.bits());
    }

    /// Sends a proper end of interrupt.
    ///
    /// # Special Mask
    ///
    /// Before calling this function in a special mask mode [´PicOperationMode::SpecialMask´], a
    /// mask can be applied to the data port of the PIC to inhibit some interrupts. Priority can
    /// also be changed.
    ///
    /// # Note
    ///
    /// Does nothing if PIC is configured with automatic EOI flag or in poll mode.
    pub fn end_of_interrupt(&mut self) {
        if !self.automatic_interrupts {
            match self.op_mode {
                PicOperationMode::AutomaticRotation => unsafe { 
                    self.command.write(OCW2::ROTATE_ON_NON_SPECIFIC_EOI_COMMAND.bits()); 
                },
                PicOperationMode::PolledMode => (), // Interrupt is acknowledged once the command port is read.
                _ => unsafe { self.non_specified_eoi() },
            }
        }
    }

    /// Sends a proper specific end of interrupt.
    ///
    /// # Special Mask
    ///
    /// Before calling this function in a special mask mode [´PicOperationMode::SpecialMask´], a
    /// mask can be applied to the data port of the PIC to inhibit some interrupts. Priority can
    /// also be changed.
    ///
    /// # Unsafe
    ///
    /// A proper irq must be used, or new interrupts won't appear.
    ///
    /// # Note
    ///
    /// Does nothing if PIC is configured with automatic EOI flag or in poll mode.
    pub unsafe fn end_of_interrupt_specific(&mut self, irq: u8) {
        assert!(irq < 8, "Level is written in binary format (0 .. 7).");

        if !self.automatic_interrupts {
            match self.op_mode {
                PicOperationMode::AutomaticRotation => unsafe { 
                    self.command.write(OCW2::ROTATE_ON_SPECIFIC_EOI_COMMAND.bits() | irq << OCW2::LEVEL.bits()); 
                },
                PicOperationMode::PolledMode => (), // Interrupt is acknowledged once the command port is read.
                _ => unsafe { self.specified_eoi(irq) },
            }
        }
    }

    /// Manually change the lowest priority of this PIC.
    ///
    /// The lowest priority can be fixed on some IRQ and thus fixing other priorities, where lower
    /// IRQs grow priority, i.e if the IRQ5 is lowest priority, then IRQ6 is the highest priority 
    /// and IRQ4 is the second lowest priority (it is circular). By default in fully nested mode, 
    /// the IRQ0 is the highest and IRQ7 is the lowest.
    ///
    /// The value is expected in binary format.
    ///
    /// # Note
    ///
    /// Note that PIC will generate a spurious interrupt on IRQ7 regardless of the priority level.
    pub unsafe fn set_lowest_priority(&mut self, level: u8) {
        assert!(level < 8, "Level is written in binary format (0 .. 7).");

        self.command.write(
            OCW2::SET_PRIORITY_COMMAND.bits() | (level << OCW2::LEVEL.bits())
        );
    }

    /// Performs an unsafe specified end of interrupt.
    ///
    /// The value is expected in binary format.
    ///
    /// # Unsafe 
    ///
    /// Specified end of interrupt must be written together with an interrupt level to reset.
    /// Reseting a wrong level will cause the interrupt handler to enter a loop. 
    pub unsafe fn specified_eoi(&mut self, level: u8) {
        self.command.write(
            OCW2::SPECIFIC_EOI_COMMAND.bits() | (level << OCW2::LEVEL.bits())
        );
    }

    /// Performs an unsafe non specified end of interrupt.
    ///
    /// The value is expected in binary format.
    ///
    /// # Unsafe
    ///
    /// Non specific EOI resets the highest ISR bit of those that are set. It is safe to use in
    /// fully nested mode, which is the default and mostly used mode on PC, however will cause
    /// wrong flags being cleared on different operation modes.
    pub unsafe fn non_specified_eoi(&mut self) { 
        self.command.write(
            OCW2::NON_SPECIFIC_EOI_COMMAND.bits()
        );
    }
}
