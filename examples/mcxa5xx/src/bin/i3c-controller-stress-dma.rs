//! Two-board I3C controller stress test with loopback and queued bursts.
//!
//! Each iteration performs one immediate loopback, then writes eight frames
//! back-to-back before accepting any IBI or reading any response. Every
//! burst is returned as one ordered read and checked byte-for-byte against all
//! eight corresponding writes.
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
use embassy_mcxa::i3c::controller::{
    self, BusType, I3c, IbiEvent, IbiSlot, InterruptHandler, Operation, Payload,
};
use embassy_mcxa::peripherals::I3C0;
use embassy_mcxa5xx_examples::i3c_common::{
    BURST_BYTES, BURST_FRAMES, FRAME_LEN, FrameKind, IBI_MDB, TARGET_DYNAMIC_ADDR, TARGET_STATIC_ADDR,
    build_frame, check_frame,
};
use embassy_mcxa5xx_examples::util::verify_equal;
use embassy_time::{Duration, Timer, with_timeout};
use panic_probe as _;

const IO_TIMEOUT: Duration = Duration::from_millis(500);

bind_interrupts!(
    struct Irqs {
        I3C0 => InterruptHandler<I3C0>;
    }
);

type Ctrl<'d> = I3c<'d, hal::i3c::Dma<'d>>;

async fn set_dasa(i3c: &mut Ctrl<'_>) -> Result<(), controller::IOError> {
    i3c.async_transaction(
        &mut [
            Operation::Write {
                address: 0x7e,
                buf: &[0x87],
            },
            Operation::Write {
                address: TARGET_STATIC_ADDR,
                buf: &[TARGET_DYNAMIC_ADDR << 1],
            },
        ],
        BusType::I3cSdr,
    )
    .await
}

async fn write_frame(i3c: &mut Ctrl<'_>, frame: &[u8], iter: u32, frame_index: usize) {
    match with_timeout(
        IO_TIMEOUT,
        i3c.async_write(TARGET_DYNAMIC_ADDR, frame, BusType::I3cSdr),
    )
    .await
    {
        Ok(Ok(())) => {}
        Ok(Err(e)) => {
            error!("[ctrl] iter {} frame {} write error {:?}", iter, frame_index, e);
            panic!("stress write failed");
        }
        Err(_) => {
            error!("[ctrl] iter {} frame {} write timeout", iter, frame_index);
            panic!("stress write timed out");
        }
    }
}

async fn wait_for_echo(
    i3c: &mut Ctrl<'_>,
    ibi: &mut [u8; 8],
    iter: u32,
    frame_index: usize,
) {
    ibi.fill(0);
    match with_timeout(IO_TIMEOUT, i3c.async_wait_for_ibi(ibi)).await {
        Ok(Ok(IbiEvent::Ibi {
            address,
            payload_len,
        })) if address == TARGET_DYNAMIC_ADDR && payload_len == 1 && ibi[0] == IBI_MDB => {}
        Ok(Ok(event)) => {
            error!(
                "[ctrl] iter {} frame {} unexpected IBI {:?} payload={:?}",
                iter, frame_index, event, ibi
            );
            panic!("unexpected stress IBI");
        }
        Ok(Err(e)) => {
            error!("[ctrl] iter {} frame {} IBI error {:?}", iter, frame_index, e);
            panic!("stress IBI failed");
        }
        Err(_) => {
            error!("[ctrl] iter {} frame {} IBI timeout", iter, frame_index);
            panic!("stress IBI timed out");
        }
    }
}

async fn read_echo(
    i3c: &mut Ctrl<'_>,
    expected: &[u8; FRAME_LEN],
    rx: &mut [u8; FRAME_LEN],
    iter: u32,
    frame_index: usize,
) {
    rx.fill(0xa5);
    let n = match with_timeout(
        IO_TIMEOUT,
        i3c.async_read(TARGET_DYNAMIC_ADDR, rx, BusType::I3cSdr),
    )
    .await
    {
        Ok(Ok(n)) => n,
        Ok(Err(e)) => {
            error!("[ctrl] iter {} frame {} read error {:?}", iter, frame_index, e);
            panic!("stress read failed");
        }
        Err(_) => {
            error!("[ctrl] iter {} frame {} read timeout", iter, frame_index);
            panic!("stress read timed out");
        }
    };

    if n != FRAME_LEN || !check_frame(&rx[..n]) {
        error!(
            "[ctrl] iter {} frame {} invalid response n={} data={:?}",
            iter,
            frame_index,
            n,
            &rx[..n]
        );
        panic!("invalid stress response");
    }
    verify_equal(&rx[..n], expected, "i3c loopback");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);

    {
        use embassy_mcxa::i3c::PurPin;
        PurPin::<I3C0>::mux(&*p.P0_2);
    }

    let cfg = controller::Config::default();
    let mut i3c = I3c::new_async_with_dma(p.I3C0, p.P0_21, p.P0_20, p.DMA0_CH0, p.DMA0_CH1, Irqs, cfg).unwrap();

    Timer::after_secs(2).await;
    info!("[ctrl] RSTDAA");
    i3c.async_write(0x7e, &[0x06], BusType::I3cSdr).await.unwrap();

    info!("[ctrl] SETDASA static={:#x} dynamic={:#x}", TARGET_STATIC_ADDR, TARGET_DYNAMIC_ADDR);
    set_dasa(&mut i3c).await.unwrap();
    i3c.register_ibi(IbiSlot::Slot0, TARGET_DYNAMIC_ADDR, Payload::Yes)
        .unwrap();

    info!(
        "[ctrl] loopback+burst start frame_len={} burst_frames={}",
        FRAME_LEN, BURST_FRAMES
    );

    let mut single = [0u8; FRAME_LEN];
    let mut burst = [0u8; BURST_BYTES];
    let mut rx = [0u8; FRAME_LEN];
    let mut ibi = [0u8; 8];
    let mut iter: u32 = 0;

    loop {
        let base_sequence = iter.wrapping_mul((BURST_FRAMES + 1) as u32);

        build_frame(&mut single, base_sequence, FrameKind::Single);
        write_frame(&mut i3c, &single, iter, 0).await;
        wait_for_echo(&mut i3c, &mut ibi, iter, 0).await;
        read_echo(&mut i3c, &single, &mut rx, iter, 0).await;

        for index in 0..BURST_FRAMES {
            let start = index * FRAME_LEN;
            let end = start + FRAME_LEN;
            let frame: &mut [u8; FRAME_LEN] = (&mut burst[start..end]).try_into().unwrap();
            build_frame(
                frame,
                base_sequence.wrapping_add(index as u32 + 1),
                FrameKind::Burst(index),
            );
            write_frame(&mut i3c, frame, iter, index + 1).await;
        }

        wait_for_echo(&mut i3c, &mut ibi, iter, 1).await;
        for index in 0..BURST_FRAMES {
            let start = index * FRAME_LEN;
            let end = start + FRAME_LEN;
            let expected: &[u8; FRAME_LEN] = (&burst[start..end]).try_into().unwrap();
            read_echo(&mut i3c, expected, &mut rx, iter, index + 1).await;
        }

        iter = iter.wrapping_add(1);
        if iter == 1 || iter.is_multiple_of(100) {
            info!(
                "[ctrl] {} iterations OK: {} single + {} burst loopbacks",
                iter,
                iter,
                iter * BURST_FRAMES as u32
            );
        }
    }
}
