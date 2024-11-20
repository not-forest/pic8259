//! Defines a single configurable PIC chip.
//!
//! For most systems it is not necessary to manually configure each individual PIC. Please use
//! [´ChainedPics´] for convenient structure to manipulate on chained set of PICs located on the
//! x86 architecture.

use x86_64::instructions::port::Port;
use crate::post::post_debug_delay;
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
    offset: u8,
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

    /// Are we in charge of handling the specified interrupt?
    /// (Each PIC handles 8 interrupts.)
    pub fn handles_interrupt(&self, interrupt_id: u8) -> bool {
        self.offset <= interrupt_id && interrupt_id < self.offset + 8
    }

    /// Reads the value of current operation mode used on this PIC.
    pub fn operation_mode_current(&self) -> PicOperationMode {
        self.op_mode
    }

    /// Changes the current operation mode of this PIC.
    ///
    /// This sends the OCW2 command and configurest the current operation mode of the PIC logic.
    /// Refer to [´PicOperationMode´] enum for more details.
    pub fn operation_mode_change(&mut self, new_op_mode: PicOperationMode) {
        if self.op_mode != new_op_mode {
            unsafe {
                self.command.write(match new_op_mode {
                    PicOperationMode::FullyNested => OCW2::NON_SPECIFIC_EOI_COMMAND.bits(),
                    PicOperationMode::AutomaticRotation => 
                        if self.automatic_interrupts {
                            OCW2::ROTATE_IN_AUTOMATIC_EOI_MODE_SET.bits()
                        } else {
                            OCW2::ROTATE_ON_NON_SPECIFIC_EOI_COMMAND.bits()
                        },
                    PicOperationMode::SpecialMask => OCW3::SET_SPECIAL_MASK.bits(),
                    PicOperationMode::PolledMode => OCW3::POLL.bits(),
                });
            };
            self.op_mode = new_op_mode;
        }
    }

    /// Reads the value of the ISR.
    /// 
    /// The interrupt status register inside the PIC chip, shows the info about which interrupts are
    /// being serviced at that moment. The value will be flushed after the end_of_interrupt method.
    pub fn read_isr(&mut self) -> ISR {
        unsafe {
            self.command.write(
                OCW3::READ_REG_ISR.bits()
            );
            ISR::from_bits_truncate(self.command.read())
        }
    }

    /// Reads the value of the IRR.
    /// 
    /// The interrupt request register shows the requested interrupts that have been raised
    /// but are not being acknowledged yet. The value will be flushed after the end_of_interrupt method.
    pub fn read_irr(&mut self) -> IRR {
        unsafe {
            self.command.write(
                OCW3::READ_REG_IRR.bits()
            );
            IRR::from_bits_truncate(self.command.read())
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

    /// Masks the requested IRQ lines.
    ///
    /// Sends the OCW1 command and masks unused IRQ lines.
    ///
    /// # Unsafe
    ///
    /// Even though masking just disabled some interrupt lines, this function is masked as unsafe
    /// due to undefined behavior that might happen when the OCW1 command is not right.
    pub unsafe fn mask_write(&mut self, ocw1: OCW1) {
        self.data.write(ocw1.bits());
    }

    /// Sends a proper end of interrupt.
    ///
    /// # Note
    ///
    /// Does nothing if PIC is configured with automatic EOI flag.
    pub fn end_of_interrupt(&mut self) {
        match self.op_mode {
            PicOperationMode::FullyNested => unsafe { self.non_specified_eoi() },
            PicOperationMode::SpecialMask => unimplemented!(),
            _ => (),
        }
    }

    /// Performs an unsafe specified end of interrupt.
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

