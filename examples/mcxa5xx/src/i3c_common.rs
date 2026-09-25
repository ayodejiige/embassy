//! Shared frame format for the two-board I3C DMA loopback and burst stress
//! examples.

/// Static address presented by the target before SETDASA.
pub const TARGET_STATIC_ADDR: u8 = 0x0a;
/// Dynamic address assigned by the controller.
pub const TARGET_DYNAMIC_ADDR: u8 = 0x0b;
/// Size of every write and returned read in the stress loop.
pub const FRAME_LEN: usize = 64;
/// Number of writes issued back-to-back before any burst readback.
pub const BURST_FRAMES: usize = 8;
/// Total bytes returned by the burst readback.
pub const BURST_BYTES: usize = BURST_FRAMES * FRAME_LEN;
/// Mandatory one-byte MDB used to announce that a readback is ready.
pub const IBI_MDB: u8 = 0x01;

const SINGLE_MARKER: u8 = 0x53;
const BURST_MARKER: u8 = 0x42;
const FRAME_CRC_LEN: usize = 2;
const FRAME_HEADER_LEN: usize = 8;
const CRC16: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_KERMIT);

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FrameKind {
    Single,
    Burst(usize),
}

/// Build one changing, self-checking frame.
pub fn build_frame(buf: &mut [u8; FRAME_LEN], sequence: u32, kind: FrameKind) {
    let split = FRAME_LEN - FRAME_CRC_LEN;
    buf[0] = FRAME_LEN as u8;
    match kind {
        FrameKind::Single => {
            buf[1] = SINGLE_MARKER;
            buf[2] = 0;
            buf[3] = 1;
        }
        FrameKind::Burst(index) => {
            buf[1] = BURST_MARKER;
            buf[2] = index as u8;
            buf[3] = BURST_FRAMES as u8;
        }
    }
    buf[4..FRAME_HEADER_LEN].copy_from_slice(&sequence.to_le_bytes());

    for (offset, byte) in buf[FRAME_HEADER_LEN..split].iter_mut().enumerate() {
        let shift = (offset & 3) * 8;
        *byte = ((sequence >> shift) as u8)
            .wrapping_add((offset as u8).wrapping_mul(0x5b))
            .rotate_left((offset & 7) as u32);
    }

    let crc = CRC16.checksum(&buf[..split]).to_le_bytes();
    buf[split..].copy_from_slice(&crc);
}

pub fn frame_kind(frame: &[u8]) -> Option<FrameKind> {
    match (frame.get(1), frame.get(2), frame.get(3)) {
        (Some(&SINGLE_MARKER), Some(&0), Some(&1)) => Some(FrameKind::Single),
        (Some(&BURST_MARKER), Some(&index), Some(&count))
            if count as usize == BURST_FRAMES && (index as usize) < BURST_FRAMES =>
        {
            Some(FrameKind::Burst(index as usize))
        }
        _ => None,
    }
}

pub fn frame_sequence(frame: &[u8]) -> Option<u32> {
    let bytes: [u8; 4] = frame.get(4..FRAME_HEADER_LEN)?.try_into().ok()?;
    Some(u32::from_le_bytes(bytes))
}

/// Check the fixed length, protocol metadata, and trailing CRC.
pub fn check_frame(frame: &[u8]) -> bool {
    if frame.len() != FRAME_LEN || frame[0] as usize != FRAME_LEN || frame_kind(frame).is_none() {
        return false;
    }

    let split = FRAME_LEN - FRAME_CRC_LEN;
    let expected = u16::from_le_bytes([frame[split], frame[split + 1]]);
    CRC16.checksum(&frame[..split]) == expected
}
