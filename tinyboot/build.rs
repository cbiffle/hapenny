use std::{env::VarError, path::PathBuf};
use std::io::Write;

fn main() {
    println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rerun-if-env-changed=TINYBOOT_UART_ADDR");

    let addr_input = match std::env::var("TINYBOOT_UART_ADDR") {
        // Ugh why is this not an Option
        Err(VarError::NotPresent) => None,
        Ok(result) => Some(result),
        e => panic!("{:?}", e),
    };

    let addr = match addr_input {
        None => {
            println!("cargo:warning=note: UART address not provided, defaulting to 0x200");
            0x200
        }
        Some(text) => {
            parse_int::parse::<u32>(&text).unwrap()
        }
    };

    let mut out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    out.push("peripherals.rs");

    let mut f = std::fs::File::create(&out).unwrap();
    writeln!(f, "pub const UART_ADDR: u32 = 0x{addr:x};").unwrap();
}
