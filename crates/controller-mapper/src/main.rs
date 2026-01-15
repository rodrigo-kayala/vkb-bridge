use anyhow::{Context, Result, bail};
use evdev::{AbsoluteAxisCode, Device, EventSummary};
use std::collections::HashMap;
use std::io;

#[derive(Debug, Default, Clone)]
struct DeviceState {
    axes: HashMap<AbsoluteAxisCode, i32>,
}

fn main() -> Result<()> {
    let selected_device = select_device()?;

    println!(
        "selected device: {}\nvendor:{:04x} product:{:04x}",
        selected_device.name().unwrap_or("no_name"),
        selected_device.input_id().vendor(),
        selected_device.input_id().product()
    );

    let vendor = selected_device.input_id().vendor();
    let product = selected_device.input_id().product();

    let mut dev = open_vkb_device(vendor, product)
        .with_context(|| "Could not open VKB device. Check permissions (/dev/input/event*)")?;

    println!(
        "Supported absolute axes: {:?}",
        dev.supported_absolute_axes()
    );
    println!("Supported keys: {:?}", dev.supported_keys());
    let mut state = DeviceState::default(); // Print a snapshot at most every 250ms so logs stay readable

    loop {
        for ev in dev.fetch_events()? {
            match ev.destructure() {
                EventSummary::AbsoluteAxis(_, axis, value) => {
                    state.axes.insert(axis, value);

                    if (value - 2048).abs() > 1000 {
                        println!("ABS {:?} = {}", axis, ev.value());
                    }
                }
                EventSummary::Key(_, key_code, value) => {
                    let pressed = value != 0;
                    if pressed {
                        println!("KEY {:?} = {}", key_code.0, pressed as u8);
                    }
                }
                _ => {}
            }
        }
    }
}

fn open_vkb_device(target_vendor: u16, target_product: u16) -> Result<Device> {
    for (_path, dev) in evdev::enumerate() {
        let id = dev.input_id();
        if id.vendor() == target_vendor && id.product() == target_product {
            return Ok(dev);
        }
    }
    bail!(
        "Device not found for vendor={:04x} product={:04x}",
        target_vendor,
        target_product
    )
}

fn select_device() -> Result<Device> {
    let devices: Vec<Device> = evdev::enumerate().map(|(_path, dev)| dev).collect();

    for (i, d) in devices.iter().enumerate() {
        let name = d.name().unwrap_or("<no name>");
        let id = d.input_id();
        println!(
            "[{}] {name} vendor={:04x} product={:04x}",
            i + 1,
            id.vendor(),
            id.product()
        );
    }

    let device_idx: usize = receive_user_input("select device to map:")
        .unwrap()
        .parse()
        .context("cannot parse input")?;

    devices
        .into_iter()
        .nth(device_idx - 1)
        .context("device index not found")
}

fn receive_user_input(prompt: &str) -> Result<String> {
    println!("{prompt}");

    let mut input = String::new();
    io::stdin()
        .read_line(&mut input)
        .context("failed to read from stdin")?;

    Ok(input.trim().to_owned())
}
