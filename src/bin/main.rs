#![no_std]
#![no_main]
#![deny(clippy::mem_forget)]

use core::sync::atomic::{AtomicU8, Ordering};

use embassy_executor::Spawner;
use embassy_futures::join::join3; // <--- Changed to join3
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

// ====================================================
// 1. GLOBAL SHARED STATE
// ====================================================
// We use a global Atomic variable so the BLE task can write to it,
// and the Motor task can read from it simultaneously and safely.
static TARGET_DUTY: AtomicU8 = AtomicU8::new(80);

#[gatt_server]
struct FluesterServer {
    pwm_service: PwmService,
}

#[gatt_service(uuid = "a9c81b72-0f7a-4c59-b0a8-425e3bcf0a0e")]
struct PwmService {
    #[characteristic(
        uuid = "c79b2ca7-f39d-4060-8168-816fa26737b7",
        write, read, notify, value = 10
    )]
    duty_cycle: u8,
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    esp_println::println!("Panic occurred: {:?}", info);
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    #[cfg(target_arch = "riscv32")]
    let software_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        software_interrupt.software_interrupt0,
    );

    // ====================================================
    // 2. HARDWARE PWM SETUP
    // ====================================================
    let led = peripherals.GPIO3;
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .unwrap();

    let mut channel0 = ledc.channel(channel::Number::Channel0, led);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    // ====================================================
    // 3. BLUETOOTH HOST SETUP
    // ====================================================
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(bluetooth, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    let mut resources: HostResources<DefaultPacketPool, 1, 1> = HostResources::new();

    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random([0x11, 0x22, 0x33, 0x44, 0x55, 0x66]));

    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    let server = FluesterServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "FluesterLuefter",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    // ====================================================
    // 4. ASYNC CONCURRENT EXECUTION
    // ====================================================

    // TASK 1: The Motor Slew Rate Limiter
    let motor_task = async {
        let mut current_duty = 0u8; // only init value, no controll here
        let safe_step = 1u8; // Maximum change allowed per 250ms interval

        loop {
            // Read the target safely from the atomic variable
            let target = TARGET_DUTY.load(Ordering::Relaxed);

            if current_duty != target {
                // If we need to go UP
                if current_duty < target {
                    // Add the step, but don't overshoot the target
                    current_duty = current_duty.saturating_add(safe_step).min(target);
                } 
                // If we need to go DOWN
                else {
                    // Subtract the step, but don't drop below the target
                    current_duty = current_duty.saturating_sub(safe_step).max(target);
                }

                // Apply to hardware
                channel0.set_duty(current_duty).unwrap();
                esp_println::println!("Motor ramping... current duty: {}%", current_duty);
            }

            Timer::after(Duration::from_millis(50)).await;
        }
    };

    // TASK 2: Bluetooth Logic
    let ble_task = async {
        loop {
            esp_println::println!("Starting BLE Advertising...");
            let mut adv_data = [0; 31];
            let len = AdStructure::encode_slice(
                &[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::CompleteLocalName(b"FluesterLuefter"),
                ],
                &mut adv_data[..],
            ).unwrap();

            let advertiser = peripheral.advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_data[..len],
                    scan_data: &[], 
                },
            ).await.unwrap();

            let conn = advertiser.accept().await.unwrap().with_attribute_server(&server).unwrap();
            esp_println::println!("Smartphone Connected!");

            loop {
                match conn.next().await {
                    GattConnectionEvent::Disconnected { .. } => {
                        esp_println::println!("Smartphone Disconnected!");
                        break; 
                    }
                    GattConnectionEvent::Gatt { event } => match event {
                        GattEvent::Write(write_event) => {
                            if write_event.handle() == server.pwm_service.duty_cycle.handle {
                                
                                let new_duty = write_event.data()[0];
                                let safe_duty = new_duty.min(100);
                                esp_println::println!("App requested target duty: {}%", safe_duty);

                                // Write the new target to the global Atomic variable!
                                TARGET_DUTY.store(safe_duty, Ordering::Relaxed);
                            }
                        }
                        _ => {} 
                    },
                    _ => {}
                }
            }
        }
    };

    // TASK 3: BLE Background Driver
    let runner_task = async {
        runner.run().await;
    };

    // Run all 3 tasks cooperatively side-by-side
    join3(motor_task, ble_task, runner_task).await;
    
    loop {}
}