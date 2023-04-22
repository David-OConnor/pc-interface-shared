//! This library provides abstractions for use in PC programs that
//! interact with devices over USB-serial. It's generally used with 
//! an egui GUI. It prevents code-duplication.
//! See the separate module (anyleaf_usb) for code we share
//! between PC and firmware.

use std::{
    io::{self},
    time::{Duration, Instant},
};

use eframe::egui::{self, Color32};

use serialport::{self, SerialPort, SerialPortType};

use anyleaf_usb::{self, MessageType, MSG_START, PAYLOAD_START_I};

const FC_SERIAL_NUMBER: &str = "AN";

const BAUD: u32 = 115_200;

const TIMEOUT_MILIS: u64 = 10;

pub const DISCONNECTED_TIMEOUT_MS: u64 = 1_000;

pub type Port = Box<dyn SerialPort>;

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
            // Self::ConnectedRx => "Connected to Rx",
        }
    }

    pub fn as_color(&self) -> Color32 {
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
    pub serial_port: Option<Port>,
}

// todo: look into cleaning this, and `get_port()` below up.

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
                                Ok(port) => {
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
    pub fn get_port(&mut self) -> Result<&mut Port, io::Error> {
        // todo: If it does'nt work after getting back from Southern Strike, try
        // todo this:
        // self.interface = SerialInterface::new();

        // todo: DRY with port. Trouble passing it as a param due to box<dyn
        match self.interface.serial_port.as_mut() {
            Some(p) => {
                self.connection_status = ConnectionStatus::Connected;
                self.last_response = Instant::now();

                Ok(p)
            }
            None => {
                self.connection_status = ConnectionStatus::NotConnected;

                Err(io::Error::new(
                    io::ErrorKind::NotConnected,
                    "Sensor interface is not connected.",
                ))
            }
        }
    }

    /// Send a payload-less command, ie the only useful data being message-type.
    /// Does not handle responses.
    pub fn send_cmd<T: MessageType>(&mut self, msg_type: T) -> Result<(), io::Error> {
        let port = self.get_port()?;
        send_payload::<T, 4>(0, msg_type, &[], port)
    }
}

/// Send a payload, using our format of standard start byte, message type byte,
/// payload, then CRC.
/// `N` is the entire message size. (Can't have it be payload size
/// due to restrictions)
pub fn send_payload<T: MessageType, const N: usize>(
    device_code: u8,
    msg_type: T,
    payload: &[u8],
    port: &mut Port,
) -> Result<(), io::Error> {
    // N is the total packet size.
    let payload_size = msg_type.payload_size();

    // start byte, device type byte, message type byte, payload, CRC.
    let mut tx_buf = [0; N];

    tx_buf[0] = MSG_START;
    tx_buf[1] = device_code;
    tx_buf[2] = msg_type.val();

    tx_buf[PAYLOAD_START_I..(payload_size + PAYLOAD_START_I)].copy_from_slice(&payload);

    tx_buf[payload_size + PAYLOAD_START_I] = anyleaf_usb::calc_crc(
        &anyleaf_usb::CRC_LUT,
        &tx_buf[..payload_size + PAYLOAD_START_I],
        payload_size as u8 + PAYLOAD_START_I as u8,
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