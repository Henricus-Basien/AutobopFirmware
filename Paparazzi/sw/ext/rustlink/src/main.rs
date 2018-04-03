extern crate serial;
extern crate ivyrust;
extern crate pprzlink;
extern crate rand;
extern crate libc;
extern crate regex;
extern crate rusthacl;


mod comms;
mod configs;
mod ivy;

use comms::*;
use configs::*;
use ivy::*;

use std::{thread, time};

use std::error::Error;

use std::sync::Mutex;
use std::sync::Arc;

use std::collections::VecDeque;

use pprzlink::parser::{PprzDictionary, PprzMessage, PprzMsgClassID};
use pprzlink::transport::PprzTransport;
use pprzlink::secure_transport::{SecurePprzTransport, StsParty};

use time::*;


#[allow(dead_code)]
fn print_array(b: &[u8]) {
    print!("data=[");
    for i in b {
        print!("0x{:x},", i);
    }
    println!("];");

}

fn thread_main_secure(
    config: Arc<LinkConfig>,
    dictionary: Arc<PprzDictionary>,
    msg_queue: Arc<Mutex<VecDeque<PprzMessage>>>,
) -> Result<(), Box<Error>> {
    println!("Starting encrypted datalink...");
    let mut port = match LinkComm::new(Arc::clone(&config)) {
        Ok(p) => p,
        Err(e) => panic!("{}: Comm initialization failed: {}", config.link_name, e),
    };

    // initialize variables in LINK_REPORT
    let mut status_report_timer = Instant::now();
    let status_report_period = Duration::from_millis(config.status_period);
    let mut report_msg = dictionary
        .find_msg_by_name(String::from("LINK_REPORT").as_ref())
        .expect("LINK_REPORT message not found");
    let mut status_report = RustlinkStatusReport::new();

    // instantiate a ping time and callback
    let mut ping_cb = LinkIvyPing::new(0.1);
    let ping_msg_id = dictionary
        .find_msg_by_name("PING")
        .expect("Ping message not found")
        .id;

    // use crypto
    let mut trans = SecurePprzTransport::new(StsParty::Initiator, config.ac_id);
    trans.set_sender(0);
    trans.my_private_key.set(&config.q_a, &config.p_a).unwrap();
    trans.their_public_key.set(&config.p_b).unwrap();
    trans.dictionary = Some(dictionary.clone());
    trans.set_msg_class(PprzMsgClassID::Telemetry);

    // initialize an emty buffer
    let mut buf = [0; 255]; // still have to manually allocate an array

    // get debug time
    let debug_time = RustlinkTime::new();

    loop {
        // process messages from the queue and encrypt them before sending
        let mut lock = msg_queue.lock();
        if let Ok(ref mut msg_queue) = lock {
            while !msg_queue.is_empty() {
                // get a message from the front of the queue
                let new_msg = msg_queue.pop_front().unwrap();

                // check for ping
                if new_msg.id == ping_msg_id {
                    // this message is a PING message, update the ping time
                    ping_cb.reset();
                }

				//println!("Sts stage: {:?}", trans.get_stage());

                // attempt to construct a message
                let msg = match trans.construct_pprz_msg(&new_msg.to_bytes()) {
                    Some(msg) => msg,
                    None => vec![],
                };

				//println!("Sending message: {:?}", msg);
                let len = port.com_write(msg.as_slice())?;
                status_report.tx_bytes += len;
                status_report.tx_msgs += 1;
                if len != msg.len() {
                    println!(
                        "{}: {} Written {} bytes, but the message was {} bytes",
                        config.link_name,
                        debug_time.elapsed(),
                        len,
                        msg.len()
                    );
                }
            }
        }

        // read data (1ms timeout right now)
        let len = match port.com_read(&mut buf[..]) {
            Ok(len) => len,
            Err(_) => continue,
        };
        status_report.rx_bytes += len;
        //println!("just got {} bytes",len);
        //println!("here is my original buffer: {:?}",&buf[0..len]);

        for byte in &buf[0..len] {
            if let Some(msg) = trans.parse_byte(*byte) {
                status_report.rx_msgs += 1;
				//println!("received new message!, total = {}",status_report.rx_msgs);

                let name = dictionary
                    .get_msg_name(
                        config.rx_msg_class,
                        PprzMessage::get_msg_id_from_buf(&msg, dictionary.protocol),
                    )
                    .expect("thread main: message name not found");
                let mut new_msg = dictionary.find_msg_by_name(&name).expect(
                    "thread main: no message found",
                );
                
                //println!("Message original: {}",new_msg.to_string().unwrap());
                // update message fields with real values
                new_msg.update(&msg);

                // check for PONG
                if new_msg.name == "PONG" {
                    // update time
                    ping_cb.update();
                }
                //println!("ivy message: {}",new_msg.to_string().unwrap());
                ivyrust::ivy_send_msg(new_msg.to_string().unwrap());
            }
        } // end for idx in 0..len

        // update status & send status message if needed
        // only if the period is non-zero
        if config.status_period != 0 {
            if status_report_timer.elapsed() >= status_report_period {
                report_msg = link_update_status(
                    report_msg,
                    &mut status_report,
                    status_report_period,
                    ping_cb.ping_time_ema,
                );
                status_report_timer = Instant::now();
            }
        }
    } // end-loop
}

/// Main serial thread
///
/// Manages reading/writing to/from the serial port `port_name`
/// at given `baudrate`, and requires a dictionary of messages.
///
/// Every SYNC_PERIOD ms it sends SYNC/CHANNEL message and any pending
/// messages stored in the message queue.
///
/// Otherwise reads from serial port, and if receives a new message it sends
/// it over IVY bus.
///
/// If new message from IVY bus is to be send, it saves it to the message queue (FIFO).
fn thread_main(
    config: Arc<LinkConfig>,
    dictionary: Arc<PprzDictionary>,
    msg_queue: Arc<Mutex<VecDeque<PprzMessage>>>,
) -> Result<(), Box<Error>> {
    println!("Starting regular datalink...");

    let mut port = match LinkComm::new(Arc::clone(&config)) {
        Ok(p) => p,
        Err(e) => panic!("{}: Comm initialization failed: {}", config.link_name, e),
    };

    // initialize variables in LINK_REPORT
    let mut status_report_timer = Instant::now();
    let status_report_period = Duration::from_millis(config.status_period);
    let mut report_msg = dictionary
        .find_msg_by_name(String::from("LINK_REPORT").as_ref())
        .expect("LINK_REPORT message not found");
    let mut status_report = RustlinkStatusReport::new();

    // instantiate a ping time and callback
    let mut ping_cb = LinkIvyPing::new(0.1);
    let ping_msg_id = dictionary
        .find_msg_by_name("PING")
        .expect("Ping message not found")
        .id;

    // initialize an emty buffer
    let mut buf = [0; 255]; // still have to manually allocate an array

    // use regular transport
    let mut rx = PprzTransport::new();

    // get debug time
    let debug_time = RustlinkTime::new();

    loop {
        // process messages from the queue and encrypt them before sending
        let mut lock = msg_queue.lock();
        if let Ok(ref mut msg_queue) = lock {
            while !msg_queue.is_empty() {
                // get a message from the front of the queue
                let new_msg = msg_queue.pop_front().unwrap();

                // check for ping
                if new_msg.id == ping_msg_id {
                    // this message is a PING message, update the ping time
                    ping_cb.reset();
                }

                // get a transort
                let mut tx = PprzTransport::new();
                let mut buf = new_msg.to_bytes();

                // construct a message from the transport
                tx.construct_pprz_msg(&buf);

                let len = port.com_write(tx.buf.as_mut_slice())?;
                status_report.tx_bytes += len;
                status_report.tx_msgs += 1;
                if len != tx.buf.len() {
                    println!(
                        "{}: {} Written {} bytes, but the message was {} bytes",
                        config.link_name,
                        debug_time.elapsed(),
                        len,
                        tx.buf.len()
                    );
                }
            }
        }

        // read data (1ms timeout right now)
        let len = match port.com_read(&mut buf[..]) {
            Ok(len) => len,
            Err(_) => continue,
        };
        status_report.rx_bytes += len;

        status_report.rx_msgs +=
            process_incoming_messages(&buf[0..len], &dictionary, &config, &mut rx, &mut ping_cb);

        // update status & send status message if needed
        // only if the period is non-zero
        if config.status_period != 0 {
            if status_report_timer.elapsed() >= status_report_period {
                report_msg = link_update_status(
                    report_msg,
                    &mut status_report,
                    status_report_period,
                    ping_cb.ping_time_ema,
                );
                status_report_timer = Instant::now();
            }
        }
    } // end-loop
}


/// process new data, return number of new messages
fn process_incoming_messages(
    buf: &[u8],
    dictionary: &Arc<PprzDictionary>,
    config: &Arc<LinkConfig>,
    rx: &mut PprzTransport,
    ping_cb: &mut LinkIvyPing,
) -> usize {
    let mut new_msgs = 0;
    // parse received data and optionally send a message
    for byte in buf {
        if rx.parse_byte(*byte) {
            new_msgs += 1;
            let name = dictionary
                .get_msg_name(
                    config.rx_msg_class,
                    PprzMessage::get_msg_id_from_buf(&rx.buf, dictionary.protocol),
                )
                .expect("thread main: message name not found");
            let mut msg = dictionary.find_msg_by_name(&name).expect(
                "thread main: no message found",
            );

            // update message fields with real values
            msg.update(&rx.buf);

            // check for PONG
            if msg.name == "PONG" {
                // update time
                ping_cb.update();
            }
            ivyrust::ivy_send_msg(msg.to_string().unwrap());
        } // end parse byte
    } // end for idx in 0..len
    new_msgs
}

/// Send PING message
///
/// Push PING at the beginning of the queue
/// and then sleep for `period` ms
/// The ping time is updated once the message is sent from the main thread
fn thread_ping(
    config: Arc<LinkConfig>,
    dictionary: Arc<PprzDictionary>,
    msg_queue: Arc<Mutex<VecDeque<PprzMessage>>>,
) {

    let ping_msg = dictionary.find_msg_by_name("PING").expect(
        "Ping message not found",
    );
    loop {
        // the sender ID is set to zero by default (GCS)
        // add at the beginning of the message gueue
        {
            let mut msg_lock = msg_queue.lock();
            if let Ok(ref mut msg_queue) = msg_lock {
                // push to front
                msg_queue.push_front(ping_msg.clone());
            }
        }
        thread::sleep(time::Duration::from_millis(config.ping_period));
    }
}


/// Main IVY loop
///
/// This thread only launchees `IvyMainLoop()` and loops forever
/// Uses the optional argument specifying a non-default bus address
fn thread_ivy_main(ivy_bus: String) -> Result<(), Box<Error>> {
    ivyrust::ivy_init(String::from("Link"), String::from("Ready"));
    if !ivy_bus.is_empty() {
        ivyrust::ivy_start(Some(ivy_bus));
    } else {
        ivyrust::ivy_start(None);
    }
    ivyrust::ivy_main_loop()
}


fn start_ivy_main_loop(config: Arc<LinkConfig>) {
    // spin the main IVY loop
    let _ = thread::spawn(move || if let Err(e) = thread_ivy_main(
        config.ivy_bus.clone(),
    )
    {
        println!("Error starting ivy thread: {}", e);
    } else {
        println!("Ivy thread finished");
    });
}

fn main() {
    // Initialize
    let config = link_init_and_configure();
    let dictionary = link_build_dictionary(Arc::clone(&config));
    let msg_queue = link_build_msg_queue();

    // spin ivy main loop
    let conf = Arc::clone(&config);
    start_ivy_main_loop(conf);

    // spin the serial loop
    let conf = Arc::clone(&config);
    let dict = Arc::clone(&dictionary);
    let queue = Arc::clone(&msg_queue);
    let t = match config.gec_enabled {
        true => {
            // use secured datalink
            thread::spawn(move || if let Err(e) = thread_main_secure(
                conf,
                dict,
                queue,
            )
            {
                println!("Error starting main thread: {}", e);
            } else {
                println!("Main thread finished");
            })
        }
        false => {
            // use regular datalink
            thread::spawn(move || if let Err(e) = thread_main(conf, dict, queue) {
                println!("Error starting main thread: {}", e);
            } else {
                println!("Main thread finished");
            })
        }
    };

    // bind global ivy callback
    let dict = Arc::clone(&dictionary);
    let queue = Arc::clone(&msg_queue);
    let mut ivy_cb = LinkIvySubscriber::new(dict, queue, &config.sender_id);
    ivy_cb.ivy_bind_to_sender(
        LinkIvySubscriber::ivy_callback,
        String::from("^") + &config.sender_id + " (.*)",
    );

    // ping periodic (only if the ping period is non-zero
    if config.ping_period != 0 {
        let conf = Arc::clone(&config);
        let dict = Arc::clone(&dictionary);
        let queue = Arc::clone(&msg_queue);
        let _ = thread::spawn(move || thread_ping(conf, dict, queue));
    }
    // close
    t.join().expect("Error waiting for serial thread to finish");

    // end program
    ivyrust::ivy_stop();
}
