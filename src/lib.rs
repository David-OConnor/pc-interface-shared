//! This library provides abstractions for use in PC programs that
//! interact with devices over USB-serial. It's generally used with 
//! an egui GUI. It prevents code-duplication.
//! See the separate module (anyleaf_usb) for code we share
//! between PC and firmware.

use std::{
    convert::TryInto,
    io::{self, Read},
    thread,
    time::{self, Duration, Instant},
};

use eframe::egui::{self, Color32};

use serialport::{self, SerialPort, SerialPortType};

use anyleaf_usb::{self, MessageType, MSG_START};

const FC_SERIAL_NUMBER: &str = "AN";

const BAUD: u32 = 115_200;

const TIMEOUT_MILIS: u64 = 10;

const DISCONNECTED_TIMEOUT_MS: u64 = 1_000;

/// Convert bytes to a float
pub fn bytes_to_float(bytes: &[u8]) -> f32 {
    let bytes: [u8; 4] = bytes.try_into().unwrap();
    f32::from_bits(u32::from_be_bytes(bytes))
}

// todo: Connection status in this lib seems like a good idea,
// todo: But how should we deal with when it can connect to multiple
// todo device types?

pub enum ConnectionStatus {
    NotConnected,
    Connected,
    // ConnectedRx,
}

impl Default for ConnectionStatus {
    fn default() -> Self {
        Self::NotConnected
    }
}

impl ConnectionStatus {
    fn as_str(&self) -> &str {
        match self {
            Self::NotConnected => "Not connected",
            Self::Connected => "Connected",
            // Self::ConnectedRx => "Connected to Rx",
        }
    }

    fn as_color(&self) -> Color32 {
        match self {
            Self::NotConnected => Color32::YELLOW,
            Self::Connected => Color32::LIGHT_GREEN,
        }
    }
}

/// This mirrors that in the Python driver
pub struct SerialInterface {
    /// This `Box<dyn Trait>` is a awk, but part of the `serial_port`. This is for cross-platform
    /// compatibity, since Windows and Linux use different types; the `serial_port` docs are build
    /// for Linux, and don't show the Windows type. (ie `TTYPort vs COMPort`)
    pub serial_port: Option<Box<dyn SerialPort>>,
}

/// C+P from preflight
impl SerialInterface {
    pub fn new() -> Self {
        if let Ok(ports) = serialport::available_ports() {
            for port_info in &ports {
                if let SerialPortType::UsbPort(info) = &port_info.port_type {
                    if let Some(sn) = &info.serial_number {
                        if sn == FC_SERIAL_NUMBER {
                            match serialport::new(&port_info.port_name, BAUD)
                                .timeout(Duration::from_millis(TIMEOUT_MILIS))
                                .open()
                            {
                                // }
                                Ok(port) => {
                                    // println!("(No access error)");
                                    return Self {
                                        serial_port: Some(port),
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
                                            println!(
                                                "Error opening the port: {:?} - {:?}",
                                                kind, description
                                            );
                                        }
                                    }
                                }
                            }
                            // .expect("Failed to open serial port");
                        }
                    }
                }
            }
        }

        Self { serial_port: None }
    }
}

impl Default for SerialInterface {
    fn default() -> Self {
        Self::new()
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
    /// Send a payload-less command, ie the only useful data being message-type.
    /// Does not handle responses.
    pub fn send_cmd<T: MessageType>(&mut self, msg_type: T) -> Result<(), io::Error> {
         let port = match self.interface.serial_port.as_mut() {
            Some(p) => p,
            None => {
                self.connection_status = ConnectionStatus::NotConnected;

                return Err(io::Error::new(
                    io::ErrorKind::NotConnected,
                    "Device is not connected.",
                ));
            }
        };
        
        send_payload::<T, 3>(msg_type, &[], port)
    }
}

/// Send a payload, using our format of standard start byte, message type byte,
/// payload, then CRC.
pub fn send_payload<T: MessageType, const N: usize>(
    msg_type: T,
    payload: &[u8],
    port: &mut Box<dyn SerialPort>,
) -> Result<(), io::Error> {
    // N is the total packet size.
    let payload_size = msg_type.payload_size();

    let mut tx_buf = [0; N];

    tx_buf[0] = MSG_START;
    tx_buf[1] = msg_type.val();

    tx_buf[2..(payload_size + 2)].copy_from_slice(&payload);

    tx_buf[payload_size + 2] = anyleaf_usb::calc_crc(
        &anyleaf_usb::CRC_LUT,
        &tx_buf[..payload_size + 2],
        payload_size as u8 + 2,
    );

    port.write_all(&tx_buf)?;

    Ok(())
}


pub fn run<T: eframe::App + 'static>(state: T, window_title: &str, window_width: f32, window_height: f32) -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        initial_window_size: Some(egui::vec2(window_width, window_height)),
        ..Default::default()
    };

    eframe::run_native(
        window_title,
        options,
        Box::new(|_cc| Box::new(state)),
    )
}