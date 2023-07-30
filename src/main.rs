//! Display text received via tcp server socket on a 2.7 inch waveshare epaper display.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::str::from_utf8;

use cyw43_pio::PioSpi;
use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config as NetConfig, DhcpConfig, Stack, StackResources};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH2, PIN_12, PIN_13, PIN_23, PIN_25, PIN_8, PIN_9, PIO0, SPI1};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::{bind_interrupts, spi};
use embassy_rp::{clocks::RoscRng, config::Config as RpConfig};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Duration;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use embedded_io::asynch::Write;
use epd_waveshare::epd2in7b::{Display2in7b, Epd2in7b};
use epd_waveshare::prelude::*;
use rand::RngCore;
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _, panic_probe as _};

static TEXT: Channel<ThreadModeRawMutex, heapless::String<32>, 16> = Channel::new();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const WIFI_NETWORK: &str = include_str!("ssid.txt");
const WIFI_PASSWORD: &str = include_str!("password.txt");

type Epaper = Epd2in7b<
    Spi<'static, SPI1, spi::Async>,
    Output<'static, PIN_9>,
    Input<'static, PIN_13>,
    Output<'static, PIN_8>,
    Output<'static, PIN_12>,
    embassy_time::Delay,
>;

#[embassy_executor::task]
async fn epaper_task(
    mut epaper: Epaper,
    mut buffer: Display2in7b,
    mut spi: Spi<'static, SPI1, spi::Async>,
    mut delay: embassy_time::Delay,
) -> ! {
    let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    unwrap!(epaper.sleep(&mut spi, &mut delay));
    loop {
        let string = TEXT.recv().await;
        yield_now().await;
        unwrap!(Text::new(string.as_str(), Point::new(20, 30), style).draw(&mut buffer));
        unwrap!(epaper.wake_up(&mut spi, &mut delay));
        unwrap!(epaper.update_frame(&mut spi, buffer.buffer(), &mut delay));
        unwrap!(epaper.display_frame(&mut spi, &mut delay));
        unwrap!(epaper.sleep(&mut spi, &mut delay));
    }
}

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<
        'static,
        Output<'static, PIN_23>,
        PioSpi<'static, PIN_25, PIO0, 0, DMA_CH2>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(RpConfig::default());

    let miso = p.PIN_28;
    let mosi = p.PIN_11;
    let clk = p.PIN_10;
    let cs_pin = Output::new(p.PIN_9, Level::Low);
    let busy_in = Input::new(p.PIN_13, Pull::None);
    let dc = Output::new(p.PIN_8, Level::Low);
    let rst = Output::new(p.PIN_12, Level::High);

    let mut delay = embassy_time::Delay;
    let mut display_spi = Spi::new(
        p.SPI1,
        clk,
        mosi,
        miso,
        p.DMA_CH0,
        p.DMA_CH1,
        SpiConfig::default(),
    );

    let epd = unwrap!(Epd2in7b::new(
        &mut display_spi,
        cs_pin,
        busy_in,
        dc,
        rst,
        &mut delay
    ));

    let display: Display2in7b = Display2in7b::default();

    unwrap!(spawner.spawn(epaper_task(epd, display, display_spi, embassy_time::Delay)));

    // NOTE: these files are symlinks to the actual binary files located in the embassy repo,
    // which is included as a git submodule of this repo.
    let fw = include_bytes!("../43439A0.bin");
    let clm = include_bytes!("../43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 224190) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let dio = p.PIN_24;
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        dio,
        p.PIN_29,
        p.DMA_CH2,
    );

    let state = make_static!(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = NetConfig::dhcpv4(DhcpConfig::default());

    let seed = RoscRng.next_u64();

    // Init network stack
    let stack = &*make_static!(Stack::new(
        net_device,
        config,
        make_static!(StackResources::<2>::new()),
        seed
    ));

    unwrap!(spawner.spawn(net_task(stack)));

    loop {
        //control.join_open(WIFI_NETWORK).await;
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(()) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(30)));

        control.gpio_set(0, false).await;
        info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            warn!("accept error: {:?}", e);
            continue;
        }

        info!("Received connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("read error: {:?}", e);
                    break;
                }
            };

            info!("rxd {}", from_utf8(&buf[..n]).unwrap());

            if let Err(e) = socket.write_all(&buf[..n]).await {
                warn!("write error: {:?}", e);
                break;
            };
            TEXT.send(from_utf8(&buf[..usize::min(n, 32)]).unwrap().into())
                .await;
        }
    }
}
