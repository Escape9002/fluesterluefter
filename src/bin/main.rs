#![no_std]
#![no_main]
#![deny(clippy::mem_forget)]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

// ====================================================
// 1. DEFINE GATT SERVER & SERVICES
// ====================================================

#[gatt_server]
struct FluesterServer {
    pwm_service: PwmService,
}

// A random, custom UUID for your specific Fan/PWM service
#[gatt_service(uuid = "a9c81b72-0f7a-4c59-b0a8-425e3bcf0a0e")]
struct PwmService {
    // The Characteristic holding the Duty Cycle (0 to 100).
    // We allow the client to 'write' (adjust it), 'read' (check it), 
    // and set a default 'value' of 10%.
    #[characteristic(uuid = "c79b2ca7-f39d-4060-8168-816fa26737b7", write, read, notify, value = 10)]
    duty_cycle: u8,
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_s: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();
    
    // Allocate heap for the internal C-based ESP Wi-Fi/Bluetooth drivers
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    #[cfg(target_arch = "riscv32")]
    let software_interrupt = esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        software_interrupt.software_interrupt0,
    );

    // ====================================================
    // 2. HARDWARE PWM SETUP
    // ====================================================
    let led = peripherals.GPIO2; // Adjust to your pin
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    
    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0.configure(timer::config::Config {
        duty: timer::config::Duty::Duty5Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(24),
    }).unwrap();

    let mut channel0 = ledc.channel(channel::Number::Channel0, led);
    channel0.configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 10,
        drive_mode: DriveMode::PushPull,
    }).unwrap();

    // ====================================================
    // 3. BLUETOOTH HOST SETUP
    // ====================================================
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(bluetooth, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    // FIX: Use DefaultPacketPool instead of a constant number
    let mut resources: HostResources<DefaultPacketPool, 1, 1> = HostResources::new();
    
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random([0x11, 0x22, 0x33, 0x44, 0x55, 0x66]));
        
    let Host { mut peripheral, mut runner, .. } = stack.build();

    let server = FluesterServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "FluesterLuefter",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    })).unwrap();

    // ====================================================
    // 4. ASYNC CONCURRENT EXECUTION
    // ====================================================

    let ble_runner_task = async {
        let _ = runner.run().await;
    };

    let app_task = async {
        loop {
            esp_println::println!("Starting BLE Advertising...");
            
            // FIX: Explicitly encode the advertisement payload
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
                    scan_data: &[], // No extra scan response data
                },
            ).await.unwrap();
            
            // FIX: Accept connection and attach the server to it
            let conn = advertiser.accept().await.unwrap().with_attribute_server(&server).unwrap();
            esp_println::println!("Smartphone Connected!");

            // FIX: Poll the `conn` for events, not the `server`
            loop {
                match conn.next().await {
                    GattConnectionEvent::Disconnected { .. } => {
                        esp_println::println!("Smartphone Disconnected!");
                        break; // Break the inner loop to restart advertising
                    }
                    GattConnectionEvent::Gatt { event } => match event {
                        GattEvent::Write(write_event) => {
                            // Check if the client wrote to our duty_cycle handle
                            if write_event.handle() == server.pwm_service.duty_cycle.handle {
                                
                                // FIX: Unwrap the Result returned by server.get()
                                let new_duty = server.get(&server.pwm_service.duty_cycle).unwrap_or(10);
                                esp_println::println!("App set duty cycle to: {}%", new_duty);
                                
                                // Prevent out-of-bounds errors
                                let safe_duty = new_duty.min(100);
                                
                                channel0.start_duty_fade(0, safe_duty as u8, 500).unwrap();
                            }
                        }
                        _ => {} // Ignore Read events and other types
                    },
                    _ => {}
                }
            }
        }
    };

    join(ble_runner_task, app_task).await;
    loop {}
}