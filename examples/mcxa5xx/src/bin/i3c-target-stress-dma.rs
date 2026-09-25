//! Two-board I3C target stress test with loopback and queued bursts.
//!
//! The target validates one immediate-loopback frame, then allows the complete
//! eight-frame burst to queue before draining it. Every burst frame is
//! validated and returned in order through one one-byte-MDB IBI and one
//! logical burst read.
//!
//! Wiring: SCL P0_21 ↔ partner P0_21, SDA P0_20 ↔ partner P0_20, common GND.

#![no_std]
#![no_main]

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxa as hal;
use embassy_mcxa::bind_interrupts;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::config::Config;
use embassy_mcxa::i3c::target::{self, Div4, Event, I3c, I3cClockSel};
use embassy_mcxa::peripherals::I3C0;
use embassy_mcxa5xx_examples::i3c_common::{
    BURST_BYTES, BURST_FRAMES, FRAME_LEN, FrameKind, IBI_MDB, TARGET_STATIC_ADDR, check_frame, frame_kind,
    frame_sequence,
};
use embassy_time::{Duration, Timer, with_timeout};
use panic_probe as _;
use static_cell::ConstStaticCell;

const IO_TIMEOUT: Duration = Duration::from_millis(500);
const BURST_SETTLE_MS: u64 = 10;
const RX_BUF_SIZE: usize = target::rx_buffer_size(FRAME_LEN, BURST_FRAMES + 1);
static RX_BUF: ConstStaticCell<[u8; RX_BUF_SIZE]> = ConstStaticCell::new([0u8; RX_BUF_SIZE]);

bind_interrupts!(
    struct Irqs {
        I3C0 => target::InterruptHandler<I3C0>;
    }
);

async fn receive_frame(tgt: &mut I3c<'_>, frame: &mut [u8], iter: u32, frame_index: usize) {
    let n = match with_timeout(IO_TIMEOUT, tgt.dma_respond_to_write(frame)).await {
        Ok(Ok(n)) => n,
        Ok(Err(e)) => {
            error!("[tgt] iter {} frame {} receive error {:?}", iter, frame_index, e);
            panic!("stress receive failed");
        }
        Err(_) => {
            error!("[tgt] iter {} frame {} receive timeout", iter, frame_index);
            panic!("stress receive timed out");
        }
    };

    if n != FRAME_LEN || !check_frame(&frame[..n]) {
        error!(
            "[tgt] iter {} frame {} invalid n={} data={:?}",
            iter,
            frame_index,
            n,
            &frame[..n]
        );
        panic!("invalid stress frame");
    }
}

async fn echo_frame(tgt: &mut I3c<'_>, frame: &[u8], iter: u32, frame_index: usize) {
    match with_timeout(
        IO_TIMEOUT,
        tgt.dma_respond_to_read_with_ibi(&[IBI_MDB], frame),
    )
    .await
    {
        Ok(Ok(())) => {}
        Ok(Err(e)) => {
            error!("[tgt] iter {} frame {} response error {:?}", iter, frame_index, e);
            panic!("stress response failed");
        }
        Err(_) => {
            error!("[tgt] iter {} frame {} response timeout", iter, frame_index);
            panic!("stress response timed out");
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);

    let mut tgt_cfg = target::Config::default();
    tgt_cfg.address = Some(TARGET_STATIC_ADDR);
    tgt_cfg.ibi_capable = true;
    tgt_cfg.ibi_has_payload = true;
    tgt_cfg.max_write_len = FRAME_LEN as u16;
    tgt_cfg.clock_config.source = I3cClockSel::FroLfDiv;
    tgt_cfg.clock_config.div = Div4::from_divisor(1).unwrap();

    let mut tgt = target::I3c::new_dma(
        p.I3C0,
        p.P0_21,
        p.P0_20,
        p.DMA0_CH0,
        p.DMA0_CH1,
        Irqs,
        RX_BUF.take(),
        FRAME_LEN,
        tgt_cfg,
    )
    .unwrap();

    info!(
        "[tgt] loopback+burst ready static_addr={:#x} frame_len={} burst={} rx_buf={}B",
        TARGET_STATIC_ADDR, FRAME_LEN, BURST_FRAMES, RX_BUF_SIZE
    );

    let mut first = [0u8; FRAME_LEN];
    let mut burst = [0u8; BURST_BYTES];
    let mut expected_burst_sequence: Option<u32> = None;
    let mut iter: u32 = 0;

    loop {
        while tgt.pending_write_len().is_none() {
            if tgt.listen().await.unwrap() == Event::RxPending {
                break;
            }
        }

        Timer::after_millis(BURST_SETTLE_MS).await;
        receive_frame(&mut tgt, &mut first, iter, 0).await;

        match frame_kind(&first) {
            Some(FrameKind::Single) => {
                if expected_burst_sequence.is_some() {
                    error!("[tgt] iter {} received single before pending burst", iter);
                    panic!("missing burst");
                }
                expected_burst_sequence = frame_sequence(&first).map(|sequence| sequence.wrapping_add(1));
                echo_frame(&mut tgt, &first, iter, 0).await;
            }
            Some(FrameKind::Burst(0)) => {
                let first_sequence = frame_sequence(&first).unwrap();
                if expected_burst_sequence != Some(first_sequence) {
                    error!(
                        "[tgt] iter {} burst sequence {} expected {:?}",
                        iter, first_sequence, expected_burst_sequence
                    );
                    panic!("unexpected burst sequence");
                }

                burst[..FRAME_LEN].copy_from_slice(&first);
                for index in 1..BURST_FRAMES {
                    let start = index * FRAME_LEN;
                    let end = start + FRAME_LEN;
                    let frame = &mut burst[start..end];
                    receive_frame(&mut tgt, frame, iter, index + 1).await;
                    let expected_sequence = first_sequence.wrapping_add(index as u32);
                    if frame_kind(frame) != Some(FrameKind::Burst(index))
                        || frame_sequence(frame) != Some(expected_sequence)
                    {
                        error!("[tgt] iter {} burst frame {} out of order", iter, index);
                        panic!("burst order mismatch");
                    }
                }

                echo_frame(&mut tgt, &burst, iter, 1).await;

                expected_burst_sequence = None;
                iter = iter.wrapping_add(1);
                if iter == 1 || iter.is_multiple_of(100) {
                    info!(
                        "[tgt] {} iterations OK: {} single + {} burst loopbacks",
                        iter,
                        iter,
                        iter * BURST_FRAMES as u32
                    );
                }
            }
            _ => {
                error!("[tgt] iter {} unexpected first frame metadata", iter);
                panic!("unexpected stress phase");
            }
        }
    }
}
