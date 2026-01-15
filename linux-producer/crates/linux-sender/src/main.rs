use anyhow::{bail, Context, Result};
use evdev::{AbsInfo, AbsoluteAxisCode, Device, EventSummary, KeyCode};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std::{fs, thread};

const CONFIG_FILE_PATH: &str = "config.toml";

const AXIS_CODES: [AbsoluteAxisCode; 8] = [
    AbsoluteAxisCode::ABS_X,
    AbsoluteAxisCode::ABS_Y,
    AbsoluteAxisCode::ABS_Z,
    AbsoluteAxisCode::ABS_RX,
    AbsoluteAxisCode::ABS_RY,
    AbsoluteAxisCode::ABS_RZ,
    AbsoluteAxisCode::ABS_THROTTLE,
    AbsoluteAxisCode::ABS_RUDDER,
];

const VJOY_AXIS_MAX: u16 = 0x8000; // 32768

#[derive(Debug, Deserialize, Serialize)]
struct Config {
    dest: SocketAddr,
    send_hz: u16,
    vjoy_device: BTreeMap<u8, VJoyDevice>,
}

#[derive(Debug, Deserialize, Serialize)]
struct VJoyDevice {
    vendor_id: u16,
    product_id: u16,
}

#[derive(Clone, Copy, Debug, Default)]
struct AxisRange {
    min: i32,
    max: i32,
}

#[derive(Clone, Copy, Debug)]
struct SharedState {
    axis_range: [AxisRange; 8],
    axes_raw: [i32; 8],
    hat_x: i8,
    hat_y: i8,
    buttons: [u8; 16], // 128 bits
    revision: u64,
}

impl Default for SharedState {
    fn default() -> Self {
        Self {
            axis_range: [AxisRange::default(); 8],
            axes_raw: [0; 8],
            hat_x: 0,
            hat_y: 0,
            buttons: [0; 16],
            revision: 0,
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

fn parse() -> Result<Config> {
    let toml_str =
        fs::read_to_string(CONFIG_FILE_PATH).with_context(|| "Failed to read config file")?;
    let decoded: Config =
        toml::from_str(&toml_str).with_context(|| "Failed to parse config.toml")?;

    Ok(decoded)
}

fn main() -> Result<()> {
    let config = parse()?;
    println!("Using config: {:?}", config);
    println!("Sending UDP to {}", config.dest);

    let shared_map: HashMap<u8, Arc<Mutex<SharedState>>> = config
        .vjoy_device
        .keys()
        .map(|k| (*k, Arc::new(Mutex::new(SharedState::default()))))
        .collect();

    for (k, vjoy_device) in config.vjoy_device.iter() {
        let dev = open_vkb_device(vjoy_device.vendor_id, vjoy_device.product_id)
            .with_context(|| "Could not open VKB device. Check permissions (/dev/input/event*)")?;

        println!("Using device: {}", dev.name().unwrap_or("<no name>"));

        // Stable key mapping: KeyCode -> button index (1..=128)
        let button_map = build_button_map(&dev)?;

        // Axis ranges for normalization (from kernel abs info)
        let axis_ranges = build_axis_ranges(&dev)?;

        // Thread A: input reader
        {
            let shared = Arc::clone(&shared_map.get(k).unwrap());
            {
                shared.lock().unwrap().axis_range = axis_ranges;
            }
            thread::spawn(move || {
                if let Err(e) = input_thread(dev, shared, button_map) {
                    eprintln!("input thread error: {:#}", e);
                }
            });
        }
    }

    // Thread B: sender
    sender_thread(config, shared_map)?;

    Ok(())
}

fn build_button_map(dev: &Device) -> Result<HashMap<KeyCode, u8>> {
    let mut keys: Vec<KeyCode> = dev.supported_keys().into_iter().flatten().collect();

    keys.sort_by_key(|k| k.code());

    let mut map = HashMap::new();
    let mut idx: u16 = 1; // 1-based button ids

    for k in keys {
        if idx > 128 {
            break;
        }
        map.insert(k, idx as u8);
        idx += 1;
    }

    Ok(map)
}

fn build_axis_ranges(dev: &Device) -> Result<[AxisRange; 8]> {
    // Build a lookup table from the iterator returned by get_absinfo()
    let absinfo_map: HashMap<AbsoluteAxisCode, AbsInfo> = dev.get_absinfo()?.collect();

    let mut out = [AxisRange::default(); 8];

    for (i, code) in AXIS_CODES.iter().enumerate() {
        let info = absinfo_map
            .get(code)
            .with_context(|| format!("Missing AbsInfo for {:?}", code))?;

        out[i] = AxisRange {
            min: info.minimum(),
            max: info.maximum(),
        };
    }

    Ok(out)
}

fn input_thread(
    mut dev: Device,
    shared: Arc<Mutex<SharedState>>,
    button_map: HashMap<KeyCode, u8>,
) -> Result<()> {
    loop {
        for ev in dev.fetch_events()? {
            match ev.destructure() {
                EventSummary::AbsoluteAxis(_, AbsoluteAxisCode::ABS_HAT0X, value) => {
                    let mut st = shared.lock().unwrap();
                    let v = value.clamp(-1, 1) as i8;
                    if st.hat_x != v {
                        st.hat_x = v;
                        st.revision = st.revision.wrapping_add(1);
                    }
                }
                EventSummary::AbsoluteAxis(_, AbsoluteAxisCode::ABS_HAT0Y, value) => {
                    let mut st = shared.lock().unwrap();
                    let v = value.clamp(-1, 1) as i8;
                    if st.hat_y != v {
                        st.hat_y = v;
                        st.revision = st.revision.wrapping_add(1);
                    }
                }
                EventSummary::AbsoluteAxis(_, axis, value) => {
                    // Axes (8 slots)
                    if let Some(slot) = axis_slot(axis) {
                        let mut st = shared.lock().unwrap();
                        if st.axes_raw[slot] != value {
                            st.axes_raw[slot] = value;
                            st.revision = st.revision.wrapping_add(1);
                        }
                    }
                }
                EventSummary::Key(_, key, value) => {
                    if let Some(btn_id) = button_map.get(&key).copied() {
                        let pressed = value != 0;
                        let (byte_i, bit_i) = button_bitpos(btn_id);

                        let mut st = shared.lock().unwrap();
                        let old = (st.buttons[byte_i] >> bit_i) & 1;
                        let new = if pressed { 1 } else { 0 };

                        if old != new {
                            if pressed {
                                st.buttons[byte_i] |= 1 << bit_i;
                            } else {
                                st.buttons[byte_i] &= !(1 << bit_i);
                            }
                            st.revision = st.revision.wrapping_add(1);
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

fn axis_slot(axis: AbsoluteAxisCode) -> Option<usize> {
    AXIS_CODES.iter().position(|c| *c == axis)
}

fn button_bitpos(btn_id_1_based: u8) -> (usize, u8) {
    // btn 1 -> bit 0, btn 8 -> bit 7, btn 9 -> next byte bit 0, etc
    let zero_based = (btn_id_1_based - 1) as usize;
    (zero_based / 8, (zero_based % 8) as u8)
}

fn sender_thread(config: Config, shared_map: HashMap<u8, Arc<Mutex<SharedState>>>) -> Result<()> {
    let sock = UdpSocket::bind("0.0.0.0:0")?;
    sock.connect(config.dest)?;

    let period = Duration::from_nanos((1_000_000_000u64 / config.send_hz as u64).max(1));
    let mut next = Instant::now();

    let mut seqs: HashMap<u8, u16> = shared_map.keys().map(|&k| (k, 0u16)).collect();
    let mut buf: [u8; 43] = [0; 43];

    loop {
        next += period;

        for (k, shared) in shared_map.iter() {
            let snapshot = { *shared.lock().unwrap() }; // cheap copy
            let seq = seqs.get_mut(k).unwrap();

            encode_vkb2(&mut buf, *seq, *k, &snapshot);
            *seq = seq.wrapping_add(1);

            sock.send(&buf)?;
        }

        let now = Instant::now();
        if next > now {
            thread::sleep(next - now);
        } else {
            next = now;
        }
    }
}

fn encode_vkb2(buf: &mut [u8; 43], seq: u16, device_id: u8, st: &SharedState) {
    // header
    buf[0..4].copy_from_slice(b"VKB2"); // magic
    buf[4] = 2; // version
    buf[5] = device_id; // vjoy device id
    buf[6] = 0; // reserved
    buf[7..9].copy_from_slice(&seq.to_le_bytes()); // sequence

    // axes: u16 normalized 0..=32768
    let mut off = 9;
    for i in 0..8 {
        let v = normalize_axis(st.axes_raw[i], st.axis_range[i]);
        buf[off..off + 2].copy_from_slice(&v.to_le_bytes());
        off += 2;
    }

    buf[off] = st.hat_x as u8;
    buf[off + 1] = st.hat_y as u8;
    off += 2;

    buf[off..off + 16].copy_from_slice(&st.buttons);
}

fn normalize_axis(raw: i32, r: AxisRange) -> u16 {
    if r.max == r.min {
        return VJOY_AXIS_MAX / 2;
    }
    let num = (raw as i64 - r.min as i64) * VJOY_AXIS_MAX as i64;
    let den = r.max as i64 - r.min as i64;
    let mut out = num / den;
    if out < 0 {
        out = 0;
    }
    if out > VJOY_AXIS_MAX as i64 {
        out = VJOY_AXIS_MAX as i64;
    }
    out as u16
}
