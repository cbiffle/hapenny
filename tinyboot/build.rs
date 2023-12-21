use std::{env::VarError, path::PathBuf};
use std::io::Write;

use askama::Template;

// Template the link file
#[derive(Template)]
#[template(path="link.txt")]
struct LinkFile { 
    stack_start: u32,
}

impl LinkFile { 
    fn new(stack_start: u32) -> Self { 
        Self { 
            stack_start,
        }
    }
}

fn main() {
    //println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rerun-if-env-changed=TINYBOOT_UART_ADDR");
    println!("cargo:rerun-if-env-changed=TINYBOOT_STACK_START");

    // Uart address
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

    // Stack start address
    let stack_addr_input = match std::env::var("TINYBOOT_STACK_START") {
        // Ugh why is this not an Option
        Err(VarError::NotPresent) => None,
        Ok(result) => Some(result),
        e => panic!("{:?}", e),
    };

    let stack_start = match stack_addr_input {
        None => {
            println!("cargo:warning=note: TINYBOOT_STACK_START address not provided, defaulting to 0x400");
            0x400
        }
        Some(text) => {
            parse_int::parse::<u32>(&text).unwrap()
        }
    };

    // Generate the files 
    
    // Uart include
    let mut out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    out.push("peripherals.rs");

    let mut f = std::fs::File::create(&out).unwrap();
    writeln!(f, "pub const UART_ADDR: u32 = 0x{addr:x};").unwrap();

    // Link file include 
    let mut out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    // Tell the linker where to find it
    println!("cargo:rustc-link-search=native={}", out.display());
    out.push("link.x");

    let link_file = LinkFile::new(stack_start);
    let mut f = std::fs::File::create(&out).unwrap();
    writeln!(f, "{}",link_file.render().unwrap()).unwrap();
}
