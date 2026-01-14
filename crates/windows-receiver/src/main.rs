use anyhow::{bail, Result};
use vjoy::{ButtonState, FourWayHat, HatState, VJoy};

const LISTEN_ADDR: &str = "0.0.0.0:46000";
const VJOY_DEVICE_ID: u32 = 1;

// VKB2 packet layout (42 bytes):
// 0..4   "VKB2"
// 4      version = 2
// 5      reserved
// 6..8   seq u16 LE
// 8..24  axes[8] u16 LE (0..=32768 suggested)
// 24     hat_x i8 (as u8 on wire)
// 25     hat_y i8
// 26..42 buttons bitset 16 bytes (128 buttons), bit0 = button1
const PKT_LEN: usize = 42;

#[derive(Clone, Copy, Debug)]
struct Packet {
    seq: u16,
    axes: [u16; 8],
    hat_x: i8,
    hat_y: i8,
    buttons: [u8; 16],
}

fn main() -> Result<()> {
    let sock = UdpSocket::bind(LISTEN_ADDR)?;
    println!("Listening on UDP {LISTEN_ADDR}");

    let mut vjoy = VJoy::from_default_dll_location()?;
    let device = vjoy.get_device_state_mut(VJOY_DEVICE_ID)?;

    println!(
        "vJoy device {}: axes={} buttons={} hats={}",
        VJOY_DEVICE_ID,
        device.num_axes(),
        device.num_buttons(),
        device.num_hats()
    );

    if device.num_buttons() < 128 {
        println!("Warning: vJoy device has fewer than 128 buttons enabled in vJoyConf.exe");
    }
    if device.num_axes() < 8 {
        println!("Warning: vJoy device has fewer than 8 axes enabled in vJoyConf.exe");
    }

    let hat_mode = device.hat_type();
    let hats_enabled = device.num_hats() >= 1;

    let mut buf = [0u8; 2048];

    let mut last_seq: Option<u16> = None;
    let mut last_buttons = [0u8; 16];

    // Stats (1 Hz)
    let mut received: u64 = 0;
    let mut applied: u64 = 0;
    let mut bad: u64 = 0;
    let mut dup: u64 = 0;
    let mut ooo: u64 = 0;
    let mut lost_est: u64 = 0;
    let mut last_report = Instant::now();

    loop {
        let (len, from) = sock.recv_from(&mut buf)?;
        received += 1;

        let pkt = match decode_vkb2(&buf[..len]) {
            Ok(p) => p,
            Err(_) => {
                bad += 1;
                continue;
            }
        };

        let should_apply = match last_seq {
            None => true,
            Some(prev) => {
                if pkt.seq == prev {
                    dup += 1;
                    false
                } else if is_newer_u16(pkt.seq, prev) {
                    let diff = pkt.seq.wrapping_sub(prev) as u32;
                    if diff > 1 {
                        lost_est += (diff - 1) as u64;
                    }
                    true
                } else {
                    ooo += 1;
                    false
                }
            }
        };

        if should_apply {
            last_seq = Some(pkt.seq);
            applied += 1;

            // Axes: map packet axes[0..8] to vJoy axis IDs 1..=8
            // If your sender uses 0..=32768, passing that as i32 is fine.
            for (i, v) in pkt.axes.iter().enumerate() {
                let axis_id = (i as u32) + 1;
                device.set_axis(axis_id, *v as i32)?;
            }

            // Hat: ABS_HAT0X/ABS_HAT0Y come as -1..=1.
            // If your vJoy hat is discrete, diagonals get reduced to a cardinal direction.
            if hats_enabled {
                let hs = hatstate_from_xy(pkt.hat_x, pkt.hat_y, hat_mode);
                device.set_hat(1, hs)?;
            }

            // Buttons: only update changed bits (keeps it fast)
            let delta = xor_16(pkt.buttons, last_buttons);
            if delta != [0u8; 16] {
                for byte_i in 0..16 {
                    let changed = delta[byte_i];
                    if changed == 0 {
                        continue;
                    }
                    for bit in 0..8 {
                        if (changed & (1 << bit)) == 0 {
                            continue;
                        }
                        let btn_id_1_based = (byte_i * 8 + bit + 1) as u8;
                        let pressed = (pkt.buttons[byte_i] & (1 << bit)) != 0;
                        device.set_button(
                            btn_id_1_based,
                            if pressed {
                                ButtonState::Pressed
                            } else {
                                ButtonState::Released
                            },
                        )?;
                    }
                }
                last_buttons = pkt.buttons;
            }

            vjoy.update_all_devices()?;
        }

        if last_report.elapsed() >= Duration::from_secs(1) {
            last_report = Instant::now();
            let last = last_seq
                .map(|s| s.to_string())
                .unwrap_or_else(|| "-".to_string());
            println!(
                "stats: from={} recv={} applied={} bad={} dup={} ooo={} lost~={} last_seq={}",
                from, received, applied, bad, dup, ooo, lost_est, last
            );
        }
    }
}

fn decode_vkb2(data: &[u8]) -> Result<Packet> {
    if data.len() < PKT_LEN {
        bail!("too short");
    }
    if &data[0..4] != b"VKB2" {
        bail!("bad magic");
    }
    if data[4] != 2 {
        bail!("bad version");
    }

    let seq = u16::from_le_bytes([data[6], data[7]]);

    let mut axes = [0u16; 8];
    let mut off = 8;
    for i in 0..8 {
        axes[i] = u16::from_le_bytes([data[off], data[off + 1]]);
        off += 2;
    }

    let hat_x = data[off] as i8;
    let hat_y = data[off + 1] as i8;
    off += 2;

    let mut buttons = [0u8; 16];
    buttons.copy_from_slice(&data[off..off + 16]);

    Ok(Packet {
        seq,
        axes,
        hat_x,
        hat_y,
        buttons,
    })
}

fn xor_16(a: [u8; 16], b: [u8; 16]) -> [u8; 16] {
    let mut out = [0u8; 16];
    for i in 0..16 {
        out[i] = a[i] ^ b[i];
    }
    out
}

fn is_newer_u16(a: u16, b: u16) -> bool {
    let diff = a.wrapping_sub(b);
    diff != 0 && diff < 0x8000
}

fn hatstate_from_xy(x: i8, y: i8, hat_mode: HatState) -> HatState {
    // Normalize to -1, 0, 1
    let x = x.clamp(-1, 1);
    let y = y.clamp(-1, 1);

    match hat_mode {
        HatState::Discrete(_) => {
            // Discrete is 4-way + centered. Diagonals collapse.
            let v = match (x, y) {
                (0, 0) => FourWayHat::Centered,
                (0, -1) => FourWayHat::North,
                (0, 1) => FourWayHat::South,
                (1, 0) => FourWayHat::East,
                (-1, 0) => FourWayHat::West,

                // diagonals: pick a rule; this prefers vertical when y != 0
                (1, -1) => FourWayHat::North,
                (-1, -1) => FourWayHat::North,
                (1, 1) => FourWayHat::South,
                (-1, 1) => FourWayHat::South,
                _ => FourWayHat::Centered,
            };
            HatState::Discrete(v)
        }
        HatState::Continuous(_) => {
            // Continuous hat: 360 degrees with 1/100 degree resolution.
            // Use u32::MAX for centered (neutral).
            let angle_deg = match (x, y) {
                (0, 0) => return HatState::Continuous(u32::MAX),
                (0, -1) => 0,    // N
                (1, -1) => 45,   // NE
                (1, 0) => 90,    // E
                (1, 1) => 135,   // SE
                (0, 1) => 180,   // S
                (-1, 1) => 225,  // SW
                (-1, 0) => 270,  // W
                (-1, -1) => 315, // NW
                _ => return HatState::Continuous(u32::MAX),
            };
            HatState::Continuous((angle_deg as u32) * 100)
        }
    }
}
