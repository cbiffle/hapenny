use std::{time::Duration, path::PathBuf, io::ErrorKind, io::Write};

use anyhow::{Context, Result, bail};
use indicatif::ProgressBar;
use serialport::SerialPort;
use clap::Parser;

/// A tool for interacting with the hapenny tinyboot serial monitor.
#[derive(Debug, Parser)]
#[clap(version)]
struct BootTool {
    /// Path to serial port on your machine, e.g. /dev/ttyUSB0 or COM1:
    port: String,
    /// Baud rate of serial port.
    #[clap(long, short, global = true, default_value_t = 115_200)]
    baud_rate: u32,

    #[clap(subcommand)]
    cmd: SubCmd,
}

#[derive(Debug, Parser)]
enum SubCmd {
    /// Perform a basic check to see if tinyboot appears to be running.
    Ping,
    /// Load a single 32-bit word from an address in the target.
    Peek {
        /// Address to read.
        #[clap(value_parser = parse_int::parse::<u32>)]
        address: u32,
    },
    /// Write a single 32-bit word into the taget.
    Poke {
        /// Address to write.
        #[clap(value_parser = parse_int::parse::<u32>)]
        address: u32,
        /// Value to write.
        #[clap(value_parser = parse_int::parse::<u32>)]
        value: u32,
    },
    /// Write the contents of a file into the target. Useful for loading a
    /// program from a .bin file.
    Write {
        /// Address to begin writing.
        #[clap(value_parser = parse_int::parse::<u32>)]
        address: u32,
        /// File containing bytes to write; will be padded out to a multiple of
        /// 4.
        image_file: PathBuf,
    },
    /// Call into an address in the target.
    Call {
        /// Address to call.
        #[clap(value_parser = parse_int::parse::<u32>)]
        address: u32,
        /// If provided, the tool will immediately begin echoing back data
        /// received on the serial report until you kill it. This is useful for
        /// loading and running programs that are chatty, such as Dhrystone.
        #[clap(long)]
        then_echo: bool,
    },
}

fn main() -> Result<()> {
    let args = BootTool::parse();

    let mut port = serialport::new(&args.port, args.baud_rate)
        .timeout(Duration::from_millis(500))
        .open()
        .with_context(|| format!("opening serial port {}", args.port))?;

    drain(&mut port)?;

    match args.cmd {
        SubCmd::Ping => {
            do_cmd(&mut port, &[5])
                .context("pinging")?;
        }
        SubCmd::Peek { address } => {
            // load addr register
            let mut cmd = [3, 0, 0, 0, 0];
            cmd[1..].copy_from_slice(&address.to_le_bytes());
            do_cmd(&mut port, &cmd)
                .context("loading A")?;
            // load count register
            let cmd = [4, 1, 0, 0, 0];
            do_cmd(&mut port, &cmd)
                .context("loading C")?;
            // read out the data
            let cmd = [2];
            do_cmd(&mut port, &cmd)
                .context("sending GET")?;
            let mut data = [0; 4];
            port.read_exact(&mut data)
                .context("waiting for data")?;
            println!("{:#x}", u32::from_le_bytes(data));
        }
        SubCmd::Poke { address, value } => {
            // load addr register
            let mut cmd = [3, 0, 0, 0, 0];
            cmd[1..].copy_from_slice(&address.to_le_bytes());
            do_cmd(&mut port, &cmd)
                .context("loading A")?;
            // load count register
            let cmd = [4, 1, 0, 0, 0];
            do_cmd(&mut port, &cmd)
                .context("loading C")?;
            // deposit the data.
            let mut cmd = [1, 0, 0, 0, 0];
            cmd[1..].copy_from_slice(&value.to_le_bytes());
            do_cmd(&mut port, &cmd)
                .context("sending PUT")?;
        }
        SubCmd::Write { address, image_file } => {
            let mut image = std::fs::read(&image_file)?;
            while image.len() % 4 != 0 {
                image.push(0);
            }
            // load addr register
            let mut cmd = [3, 0, 0, 0, 0];
            cmd[1..].copy_from_slice(&address.to_le_bytes());
            do_cmd(&mut port, &cmd)
                .context("loading A")?;
            let bar = ProgressBar::new(image.len() as u64);
            for chunk in image.chunks(256) {
                // load count register
                let word_count = u32::try_from(chunk.len() / 4)?;
                let mut cmd = [4, 0, 0, 0, 0];
                cmd[1..].copy_from_slice(&word_count.to_le_bytes());
                do_cmd(&mut port, &cmd)
                    .context("loading C")?;
                let mut packet = vec![1];
                packet.extend_from_slice(chunk);
                // deposit the data.
                do_cmd(&mut port, &packet)
                    .context("sending PUT")?;
                bar.inc(chunk.len() as u64);
            }
            bar.finish();
        }
        SubCmd::Call { address, then_echo } => {
            // load addr register
            let mut cmd = [3, 0, 0, 0, 0];
            cmd[1..].copy_from_slice(&address.to_le_bytes());
            do_cmd(&mut port, &cmd)
                .context("loading A")?;
            // go!
            do_cmd(&mut port, &[0])
                .context("sending CALL")?;

            if then_echo {
                let stdout = std::io::stdout();
                let mut stdout = stdout.lock();
                loop {
                    let mut b = [0];
                    match port.read_exact(&mut b) {
                        Ok(()) => {
                            write!(stdout, "{}", b[0] as char)?;
                            stdout.flush()?;
                        },
                        Err(e) if e.kind() == ErrorKind::TimedOut => {
                            // meh
                        }
                        other => other?,
                    }
                }
            }
        }
    }

    Ok(())
}

fn do_cmd(port: &mut Box<dyn SerialPort>, cmd: &[u8]) -> Result<()> {
    port.write_all(&cmd).context("writing command")?;
    let mut response = [0; 1];
    port.read_exact(&mut response).context("collecting response byte")?;
    match response[0] {
        0xAA => Ok(()),
        0xFF => {
            bail!("Received NACK");
        }
        x => {
            bail!("Received unexpected response: {x:#x}");
        }
    }
}

fn drain(port: &mut Box<dyn SerialPort>) -> Result<()> {
    let saved_timeout = port.timeout();

    port.set_timeout(Duration::from_millis(1))
        .context("reducing timeout for drain")?;

    let mut buffer = [0; 32];
    let mut cruft = 0_usize;
    loop {
        match port.read(&mut buffer) {
            Ok(n) => cruft += n,
            Err(e) if e.kind() == ErrorKind::TimedOut => {
                break;
            }
            Err(e) => return Err(e)
                .context("attempting to drain buffer"),
        }
    }
    port.set_timeout(saved_timeout)
        .context("restoring timeout after drain")?;

    if cruft > 0 {
        println!("note: {cruft} bytes of cruft drained from serial port");
    }

    Ok(())
}

