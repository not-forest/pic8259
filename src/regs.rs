//! Defines PIC's internal ISR and IRR registers.
//!
//! The interrupts at the IR input lines are handled by two registers in cascade, the Interrupt Request Register (IRR) and the 
//! In-Service (ISR). The IRR is used to store all the interrupt levels which are requesting service; and the ISR is used to 
//! store all the interrupt levels which are being serviced.
//!
//! OS shall read those registers when deciding on sending the end of interrupt (EOI command). This way spurious IRQs can be 
//! prevented and properly handled. 

bitflags::bitflags! {
    /// Read the Interrupt Request Register (IRR).
    ///
    /// Holds an IR1 bit vector with all interrupt events which are
    /// awaiting to be services. Highest level interrupt is reset when
    /// the CPU acknowledges it.
    ///
    /// The interrupt request register shows the requested interrupts that have been raised
    /// but are not being acknowledged yet. The value will be flushed after the end_of_interrupt method.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct IRR: u8 {
        const IRQ0 = 1;
        const IRQ1 = 1 << 1;
        const IRQ2 = 1 << 2;
        const IRQ3 = 1 << 3;
        const IRQ4 = 1 << 4;
        const IRQ5 = 1 << 5;
        const IRQ6 = 1 << 6;
        const IRQ7 = 1 << 7;
    }
        
    /// Read the Interrupt Service Register (ISR).    
    ///
    /// Tracks IRQ line currently being services. Updated by EOI command.
    /// The interrupt status register inside the PIC chip, shows the info about which interrupts are
    /// being serviced at that moment. The value will be flushed after the end_of_interrupt method.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct ISR: u8 {
        const IRQ0 = 1;
        const IRQ1 = 1 << 1;
        const IRQ2 = 1 << 2;
        const IRQ3 = 1 << 3;
        const IRQ4 = 1 << 4;
        const IRQ5 = 1 << 5;
        const IRQ6 = 1 << 6;
        const IRQ7 = 1 << 7;
    }
}
