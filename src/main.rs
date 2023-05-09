#![no_main]
#![no_std]

use {defmt::error, defmt_rtt as _};

pub mod gps;

#[rtic::app(device = feather_m0::pac, peripherals = true, dispatchers = [EVSYS])]
mod app {
    use {
        defmt::{info, trace},
        feather_m0::{
            hal::{
                clock::{ClockGenId, ClockSource, GenericClockController},
                delay::Delay,
                pac::Peripherals,
                prelude::*,
                rtc::{Count32Mode, Duration, Rtc},
                sercom::i2c,
            },
            pin_alias, Pins, RedLed,
        },
        mpu6050::Mpu6050,
    };

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        // The LED could be a local resource, since it is only used in one task
        // But we want to showcase shared resources and locking
        red_led: RedLed,
    }

    #[monotonic(binds = RTC, default = true)]
    type RtcMonotonic = Rtc<Count32Mode>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        info!("init");

        let mut peripherals: Peripherals = cx.device;

        let pins = Pins::new(peripherals.PORT);

        let mut core: rtic::export::Peripherals = cx.core;

        let mut clocks = GenericClockController::with_external_32kosc(
            peripherals.GCLK,
            &mut peripherals.PM,
            &mut peripherals.SYSCTRL,
            &mut peripherals.NVMCTRL,
        );

        let _gclk = clocks.gclk0();
        let rtc_clock_src = clocks
            .configure_gclk_divider_and_source(ClockGenId::GCLK2, 1, ClockSource::XOSC32K, false)
            .unwrap();
        clocks.configure_standby(ClockGenId::GCLK2, true);

        let rtc_clock = clocks.rtc(&rtc_clock_src).unwrap();

        let rtc = Rtc::count32_mode(peripherals.RTC, rtc_clock.freq(), &mut peripherals.PM);

        let red_led: RedLed = pin_alias!(pins.red_led).into();

        core.SCB.set_sleepdeep();

        let gclk0 = clocks.gclk0();
        let sercom3_clock = &clocks.sercom3_core(&gclk0).unwrap();
        let (sda, scl) = (pins.sda, pins.scl);
        let pads = i2c::Pads::new(sda, scl);
        let mut i2c = i2c::Config::new(
            &peripherals.PM,
            peripherals.SERCOM3,
            pads,
            sercom3_clock.freq(),
        )
        .baud(100.khz())
        .enable();

        for addr in 0..128 {
            if i2c.write(addr, &[]).is_ok() {
                trace!("found device at {:X}", addr);
            }
        }

        let mut mpu = Mpu6050::new(i2c);
        mpu.init(&mut Delay::new(core.SYST, &mut clocks)).unwrap();

        blink::spawn().ok();

        info!("init done, {}", mpu.get_acc().unwrap()[0]);

        (Shared { red_led }, Local {}, init::Monotonics(rtc))
    }

    #[task(shared = [red_led])]
    fn blink(mut cx: blink::Context) {
        blink::spawn_after(Duration::millis(500)).ok();
        cx.shared.red_led.lock(|led| led.toggle().ok());
    }
}

#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    error!("panic!");
    loop {}
}
