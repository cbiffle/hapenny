#![no_main]
#![no_std]

// World's cheapest RISC-V "runtime" - only works because we don't use non-stack
// RAM (as ensured by our linker script)
core::arch::global_asm! {
    "
    .pushsection .start,\"ax\",%progbits
    .globl __start
    __start:
        # initialize stack pointer
1:      auipc sp, %pcrel_hi(__stack_start)
        addi sp, sp, %pcrel_lo(1b)
        # No need to fill in a return address, main won't return
        j main

    .popsection
    "
}

#[no_mangle]
pub extern "C" fn main() -> ! {
    let mut a: *mut u32 = core::ptr::null_mut();
    let mut c: u32 = 0;
    loop {
        match getc() {
            0 => unsafe {  // Call
                ack();
                core::arch::asm!(
                    "
                    la ra, __start      # restart monitor if it returns
                    jr a0               # activate routine
                    ",
                    in("a0") a,
                    options(noreturn),
                );
            }
            1 => {  // Write
                while c > 0 {
                    c -= 1;
                    let word = get32();
                    unsafe {
                        a.write_volatile(word);
                        a = a.add(1);
                    }
                }
                ack();
            }
            2 => {  // Read
                ack();
                while c > 0 {
                    c -= 1;
                    let word = unsafe { a.read_volatile() };
                    unsafe { a = a.add(1); }
                    put32(word);
                }
            }
            3 => {  // Load A
                a = get32() as _;
                ack();
            }
            4 => {  // Load C
                c = get32();
                ack();
            }
            5 => { // Just ping
                ack();
            }
            _ => {
                putb(0xFF);
            }
        }
    }
}

#[inline(never)]
fn ack() {
    putb(0xAA);
    flush();
}

#[inline(never)]
fn get32() -> u32 {
    let mut word = u32::from(getc());
    word |= u32::from(getc()) << 8;
    word |= u32::from(getc()) << 16;
    word |= u32::from(getc()) << 24;
    word
}

#[inline(never)]
fn put32(word: u32) {
    for b in word.to_le_bytes() {
        putb(b);
    }
}

cfg_if::cfg_if! {
    // This is gross, but, Cargo doesn't really provide a way to parameterize
    // builds on numbers, soooooo
    if #[cfg(feature = "uart-at-00000400")] {
        const UARTRX: *mut i16 = 0x400 as _;
        const UARTTX: *mut u16 = 0x402 as _;
    } else if #[cfg(feature = "uart-at-00009000")] {
        const UARTRX: *mut i16 = 0x09000 as _;
        const UARTTX: *mut u16 = 0x09002 as _;
    } else if #[cfg(feature = "uart-at-00018000")] {
        const UARTRX: *mut i16 = 0x18000 as _;
        const UARTTX: *mut u16 = 0x18002 as _;
    } else {
        compile_error!("no uart address feature defined");
    }
}

fn txbusy() -> bool {
    unsafe {
        UARTTX.read_volatile() != 0
    }
}

fn flush() {
    while txbusy() {
        // spin
    }
}

fn putb(b: u8) {
    flush();
    unsafe {
        UARTTX.write_volatile(u16::from(b));
    }
}

fn getc() -> u8 {
    loop {
        let status = unsafe { UARTRX.read_volatile() };
        if status >= 0 {
            return status as u8;
        }
    }
}

extern "C" {
    // This function is deliberately not implemented to cause a link error if we
    // include a panic.
    fn panic_handler_should_be_optimized_out() -> !;
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    unsafe {
        panic_handler_should_be_optimized_out()
    }
}
