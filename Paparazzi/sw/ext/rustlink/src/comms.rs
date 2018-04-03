extern crate serial;

use std::net::{SocketAddr, UdpSocket};

use std::io::ErrorKind as IOErrorKind;
use std::io::Error as IOError;
use std::io::prelude::*;

use std::ffi::OsString;

use std::time::Duration;

use std::error::Error;

use std::sync::Arc;

use serial::prelude::*;

use pprzlink::parser::{PprzProtocolVersion,PprzMsgClassID};


/// Structure holding configuration details
pub struct LinkConfig {
	/// [ms]
	pub ping_period: u64,
	/// [ms]
	pub status_period: u64,
	pub baudrate: usize,
	pub port: OsString,
	/// if true, communicate over UDP
	pub udp: bool,
	/// communication received here
	pub udp_port: u16,
	/// uplink messages sent to here
	pub udp_uplink_port: u16,
	/// Version of Pprzlink this link is using
	pub pprzlink_version: PprzProtocolVersion,
	/// Ivy bus address
	pub ivy_bus: String,
	/// PAPARAZZ_SRC path
	pub pprz_root: String,
	/// Remote IP address (can be broadcast if the broadcast flag is specified)
	pub remote_addr: String,
	/// Sender ID
	pub sender_id: String,
	/// Rx message class to match against
	pub rx_msg_class: PprzMsgClassID,
	/// Name of the program for debugging purposes
	pub link_name: String,
	/// Allow UDP broadcast
	pub udp_broadcast: bool,
	/// AC_ID
	pub ac_id: u8,
	/// Encryption keys: my private Q_A
	pub q_a: Vec<u8>,
	/// Encryption keys: my public P_A
	pub p_a: Vec<u8>,
	/// Encryption keys: their public P_B
	pub p_b: Vec<u8>,
	/// Enabled Galois Embedded Crypto
	pub gec_enabled: bool,
}


/// Configure port to given settings
///
/// We can specify the device name and the baudrate
/// The default timeout is zero (non-blocking read/write)
fn configure_port(
    mut port: serial::SystemPort,
    baud: usize,
) -> Result<serial::SystemPort, Box<Error>> {
    let baud = serial::core::BaudRate::from_speed(baud);
    let settings = serial::core::PortSettings {
        baud_rate: baud,
        char_size: serial::Bits8,
        parity: serial::ParityNone,
        stop_bits: serial::Stop1,
        flow_control: serial::FlowNone,
    };

    port.configure(&settings)?;
    // sleeping for on millisecond seems to be a good compromise
    port.set_timeout(Duration::from_millis(1))?;

    Ok(port)
}

pub enum LinkCommType {
	Serial,
	Udp,
}

pub struct LinkComm {
	com_type: LinkCommType,
	port: Option<serial::SystemPort>,
	socket: Option<UdpSocket>,
	udp_uplink_port: u16,
	remote_addr: SocketAddr,
}

impl LinkComm {
	pub fn new(config: Arc<LinkConfig>) -> Result<LinkComm,Box<Error>> {
		if config.udp {
			// use sockets
			let mut com = LinkComm {
				com_type: LinkCommType::Udp,
				port: None,
				socket: None,
				udp_uplink_port: config.udp_uplink_port,
				remote_addr: (config.remote_addr.clone() + ":" + &config.udp_uplink_port.to_string()).parse()?,
			};
			// let the OS decide to which interface to bind/connect, specify only the port
			let socket = UdpSocket::bind(SocketAddr::from(([0, 0, 0, 0], config.udp_port as u16)))?;
			socket.set_broadcast(config.udp_broadcast)?;
			// set read timeout of 10^6 ns (1ms)
			socket.set_read_timeout(Some(Duration::new(0,1_000_000))).expect("set_read_timeout call failed");
			// set write timeout of 10^6 ns (1ms)
			socket.set_write_timeout(Some(Duration::new(0,1_000_000))).expect("set_write_timeout call failed");
		    com.socket = Some(socket);
		    return Ok(com)
			
		} else {
			// use serial port
			let mut com = LinkComm {
				com_type: LinkCommType::Serial,
				port: None,
				socket: None,
				udp_uplink_port: config.udp_uplink_port,
				remote_addr: (config.remote_addr.clone() + ":" + &config.udp_uplink_port.to_string()).parse()?,
			};
			let port = serial::open(&config.port)?;
		    let port = match configure_port(port, config.baudrate) {
		        Ok(port) => port,
		        Err(e) => return Err(e),
		    };
		    com.port = Some(port);
		    return Ok(com)
		}
		
		
	}
	
	/// write data from buffer to the device
	pub fn com_write(&mut self, buf: &[u8]) -> Result<usize, IOError> {
		match self.com_type {
			LinkCommType::Serial => {
				match self.port {
					Some(ref mut port) => {
						port.write(buf)
					}
					None => {
						Err(IOError::new(IOErrorKind::Other, "Port not initialized"))
					}
				}
			},
			LinkCommType::Udp => {
				match self.socket {
					Some(ref mut socket) => {
						socket.send_to(buf, self.remote_addr)
					}
					None => {
						Err(IOError::new(IOErrorKind::Other, "Tx socket not initialized"))
					}
				}
			}
		}
	}
	
	/// read data from the device into the buffer
	pub fn com_read(&mut self, buf: &mut [u8]) -> Result<usize, IOError> {
		match self.com_type {
			LinkCommType::Serial => {
				match self.port {
					Some(ref mut port) => {
						port.read(buf)
					}
					None => {
						Err(IOError::new(IOErrorKind::Other, "Port not initialized"))
					}
				}
			},
			LinkCommType::Udp => {
				match self.socket {
					Some(ref mut socket) => {
						let (number_of_bytes, src_addr) = socket.recv_from(buf)?;
						if src_addr.port() == self.udp_uplink_port {
							Ok(number_of_bytes)
						} else {
							Ok(0)
						}
					}
					None => {
						Err(IOError::new(IOErrorKind::Other, "Tx socket not initialized"))
					}
				}
			}
		}	
	}
}
