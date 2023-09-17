//! This library provides abstractions for use in PC programs that
//! interact with devices over USB-serial. It's generally used with
//! an egui GUI. It prevents code-duplication.
//! See the separate module (anyleaf_usb) for code we share
//! between PC and firmware.

use std::{
    fs::{self, File},
    io::{self, Write, Read},
    time::{Duration, Instant},
};

use eframe::{IconData, egui::{self, Color32}};

use image;

use serialport::{self, SerialPort, SerialPortType};

// use winit::Window::Icon;

use anyleaf_usb::{self, MessageType, DEVICE_CODE_PC, MSG_START, PAYLOAD_START_I};

const FC_SERIAL_NUMBER: &str = "AN";
// const SLCAN_PRODUCT_NAME: &str = "ArduPilot SLCAN";
const SLCAN_PRODUCT_KEYWORD: &str = "slcan";

const BAUD: u32 = 115_200;
const BAUD_AIRPORT: u32 = 9_600;

const TIMEOUT_MILIS: u64 = 10;

pub const DISCONNECTED_TIMEOUT_MS: u64 = 1_000;

pub type Port = Box<dyn SerialPort>;

#[derive(Clone, Copy, PartialEq)]
pub enum ConnectionStatus {
    NotConnected,
    Connected,
}

impl Default for ConnectionStatus {
    fn default() -> Self {
        Self::NotConnected
    }
}

impl ConnectionStatus {
    pub fn as_str(&self) -> &str {
        match self {
            Self::NotConnected => "Not connected",
            Self::Connected => "Connected",
        }
    }

    pub fn as_color(&self) -> Color32 {
        match self {
            Self::NotConnected => Color32::YELLOW,
            Self::Connected => Color32::LIGHT_GREEN,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ConnectionType {
    Usb,
    Can,
}

impl Default for ConnectionType {
    fn default() -> Self {
        Self::Usb
    }
}

/// This mirrors that in the Python driver
#[derive(Default)]
pub struct SerialInterface {
    /// This `Box<dyn Trait>` is a awk, but part of the `serial_port`. This is for cross-platform
    /// compatibity, since Windows and Linux use different types; the `serial_port` docs are build
    /// for Linux, and don't show the Windows type. (ie `TTYPort vs COMPort`)
    pub serial_port: Option<Port>,
    pub connection_type: ConnectionType,
}

// todo: look into cleaning this, and `get_port()` below up.

impl SerialInterface {
    /// Create a new interface; either USB or CAN, depending on which we find first.
    pub fn connect() -> Self {
        let mut connection_type = ConnectionType::Usb;

        let ports = serialport::available_ports();
        if ports.is_err() {
            return Self::default();
        }

        for port_info in &ports.unwrap() {
            if let SerialPortType::UsbPort(info) = &port_info.port_type {
                let mut correct_port = false;

                let mut baud = BAUD;

                // Indicates a USB connection.
                if let Some(sn) = &info.serial_number {
                    if sn == FC_SERIAL_NUMBER {
                        correct_port = true;
                    }

                    // Indicates a (Serial) CAN connection.
                } else if let Some(product_name) = &info.product {
                    if product_name.to_lowercase().contains(SLCAN_PRODUCT_KEYWORD) {
                        connection_type = ConnectionType::Can;
                        correct_port = true;
                    }
                }

                // todo: Temp for ELRS Airport
                if let Some(man) = &info.manufacturer {
                    if man.to_lowercase().contains("wch") {
                        println!("Connected via Airport");
                        baud = BAUD_AIRPORT;
                        correct_port = true;
                    }
                }

                if !correct_port {
                    continue;
                }

                match serialport::new(&port_info.port_name, baud)
                    .timeout(Duration::from_millis(TIMEOUT_MILIS))
                    .open()
                {
                    Ok(port) => {
                        return Self {
                            serial_port: Some(port),
                            connection_type,
                        };
                    }

                    Err(serialport::Error { kind, description }) => {
                        match kind {
                            // serialport::ErrorKind::Io(io_kind) => {
                            //     println!("IO error openin the port");
                            // }
                            serialport::ErrorKind::NoDevice => {
                                // todo: Probably still getting this, but it seems to not
                                // todo be a dealbreaker. Eventually deal with it.
                                // println!("No device: {:?}", description);
                            }
                            _ => {
                                println!("Error opening the port: {:?} - {:?}", kind, description);
                            }
                        }
                    }
                }
                break;
            }
        }

        Self::default()
    }
}

/// Use this state as a field of application-specific state.
pub struct StateCommon {
    pub connection_status: ConnectionStatus,
    pub interface: SerialInterface,
    pub last_query: Instant,
    /// Used for determining if we're still connected, and getting updates from the FC.
    pub last_response: Instant,
}

impl Default for StateCommon {
    fn default() -> Self {
        Self {
            connection_status: Default::default(),
            interface: Default::default(),
            last_query: Instant::now(),
            last_response: Instant::now(),
        }
    }
}

impl StateCommon {
    /// Get the serial port; handles unwrapping.
    pub fn get_port(&mut self) -> Result<&mut Port, io::Error> {
        // todo: DRY with port. Trouble passing it as a param due to box<dyn
        match self.interface.serial_port.as_mut() {
            Some(p) => Ok(p),
            None => Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "No device connected",
            )),
        }
    }
}

/// Send a payload-less command, ie the only useful data being message-type.
/// Does not handle responses.
pub fn send_cmd<T: MessageType>(msg_type: T, port: &mut Port) -> Result<(), io::Error> {
    send_payload::<T, 4>(msg_type, &[], port)
}

/// Send a payload, using our format of standard start byte, message type byte,
/// payload, then CRC.
/// `N` is the entire message size. (Can't have it be payload size
/// due to restrictions)
pub fn send_payload<T: MessageType, const N: usize>(
    msg_type: T,
    payload: &[u8],
    port: &mut Port,
) -> Result<(), io::Error> {
    // N is the total packet size.
    let payload_size = msg_type.payload_size();

    // start byte, device type byte, message type byte, payload, CRC.
    let mut tx_buf = [0; N];

    tx_buf[0] = MSG_START;
    tx_buf[1] = DEVICE_CODE_PC;
    tx_buf[2] = msg_type.val();

    tx_buf[PAYLOAD_START_I..(payload_size + PAYLOAD_START_I)]
        .copy_from_slice(&payload[..payload_size]);

    tx_buf[payload_size + PAYLOAD_START_I] = anyleaf_usb::calc_crc(
        &anyleaf_usb::CRC_LUT,
        &tx_buf[..payload_size + PAYLOAD_START_I],
        (payload_size + PAYLOAD_START_I) as u8,
    );

    port.write_all(&tx_buf)?;

    Ok(())
}

// fn load_icon(path: &Path) -> Result<Icon, ImageError> {
//     let (icon_rgba, icon_width, icon_height) = {
//         let image = image::open(path)?.into_rgba8();
//         let (width, height) = image.dimensions();
//         let rgba = image.into_raw();
//         (rgba, width, height)
//     };
//     Ok(Icon::from_rgba(icon_rgba, icon_width, icon_height).expect("Failed to open icon"))
// }

fn load_icon(path: &str) -> eframe::IconData {
    let (icon_rgba, icon_width, icon_height) = {
        let mut f = File::open(&path).expect("No icon file found.");
        let metadata = fs::metadata(&path).expect("unable to read metadata");
        let mut buffer = vec![0; metadata.len() as usize];
        f.read(&mut buffer).expect("buffer overflow");

        // let image = image::load_from_memory(icon)
        let image = image::load_from_memory(&buffer)
            .expect("Failed to open icon path")
            .into_rgba8();

        let (width, height) = image.dimensions();
        let rgba = image.into_raw();
        (rgba, width, height)
    };

    IconData {
        rgba: icon_rgba,
        width: icon_width,
        height: icon_height,
    }
}

pub fn run<T: eframe::App + 'static>(
    state: T,
    window_title: &str,
    window_width: f32,
    window_height: f32,
    icon: Option<&str>,
) -> Result<(), eframe::Error> {
    let icon_data = match icon {
        Some(i) => Some(load_icon(i)),
        None => None,
    };

    let mut options = eframe::NativeOptions {
        initial_window_size: Some(egui::vec2(window_width, window_height)),
        // icon: load_icon(Path::new("../resources/icon.png")),
        icon_data,
        follow_system_theme: false,
        ..Default::default()
    };

    eframe::run_native(window_title, options, Box::new(|_cc| Box::new(state)))
}
