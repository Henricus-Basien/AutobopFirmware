use libc::{c_void, c_char};

use std::ffi::{CString, CStr};

use std::mem;

use std::sync::Mutex;

use std::sync::Arc;

use std::collections::VecDeque;

use ivyrust::*;

use configs::RustlinkTime;

use pprzlink::parser::{PprzMessage,PprzDictionary};




/// Structure holding data about PING time
pub struct LinkIvyPing {
	/// Time instant of sending PING message
    pub ping_instant: Mutex<RustlinkTime>,
    /// Resulting ping time [s]
    pub ping_time: f64,
    /// Filter value for EMA ping value <0,1>
    pub alpha: f64,
    /// Ping time smoothed with exponential moving
    /// average with the value of alpha [s]
    pub ping_time_ema: f64,
}


impl LinkIvyPing {
	pub fn new(alpha: f64) -> LinkIvyPing {
		LinkIvyPing {
			ping_instant: Mutex::new(RustlinkTime::new()),
			ping_time: 0.0,
			ping_time_ema: 0.0,
			alpha: alpha,
		}
	}
	
	/// Reset the underlying timer
	pub fn reset(&mut self) {
		let mut lock = self.ping_instant.lock();
	    if let Ok(ref mut ping_instant) = lock {
		    ping_instant.reset();	
	    }
	}
    
    /// Callback processing ping
    /// updates the current PING time, and resets the timer
    #[allow(dead_code)]
    pub fn callback_ping(&mut self, _: Vec<String>) {
    	let mut lock = self.ping_instant.lock();
	    if let Ok(ref mut ping_instant) = lock {
			self.ping_time = ping_instant.elapsed();
			self.ping_time_ema = self.alpha * self.ping_time + 
								 (1.0 - self.alpha) * self.ping_time;
			ping_instant.reset();
	    }
    }
    
    /// Equivalent to calling "callback_ping" but from an external source
    /// (i.e. not an Ivy bus)
    pub fn update(&mut self) {
    	let mut lock = self.ping_instant.lock();
	    if let Ok(ref mut ping_instant) = lock {
			self.ping_time = ping_instant.elapsed();
			self.ping_time_ema = self.alpha * self.ping_time + 
								 (1.0 - self.alpha) * self.ping_time_ema;
			ping_instant.reset();
	    }
    }
    
	/// Bind ivy message to a simple callback with given regexpr
	#[allow(dead_code)]
    pub fn ivy_bind_ping<F>(&mut self, cb: F, regexpr: String)
    where
        F: Fn(&mut LinkIvyPing, Vec<String>),
    {
        let regexpr = CString::new(regexpr).unwrap();
        {
	        let boxed_cb: Box<(Box<Fn(&mut LinkIvyPing, Vec<String>)>, &mut LinkIvyPing)> =
	            Box::new((Box::new(cb), self));
	        unsafe {
			        Some(IvyBindMsg(
			            apply_closure_ping,
			            Box::into_raw(boxed_cb) as *const c_void,
			            regexpr.as_ptr(),
			        ))
	        };
        }
    }
}


/// Structure for global ivy callback
pub struct LinkIvySubscriber {
	/// dictionary holding paparazzi messages
	dictionary: Arc<PprzDictionary>,
	/// message queue for paparazzi messages from Ivy bus
    msg_queue: Arc<Mutex<VecDeque<PprzMessage>>>,
    /// sender ID part of the regexpr (can be empty)
    pub sender_id: String,
}

impl LinkIvySubscriber {
	pub fn new(dictionary: Arc<PprzDictionary>, msg_queue: Arc<Mutex<VecDeque<PprzMessage>>>, sender_id: &String) -> LinkIvySubscriber {
		LinkIvySubscriber {
			dictionary: dictionary,
			msg_queue: msg_queue,
			sender_id: sender_id.clone(),
		}
	}
	
	
	/// Callback processing new message
    pub fn ivy_callback(&mut self, data: Vec<String>) {
    	let mut lock = self.msg_queue.lock();
    	if let Ok(ref mut msg_queue) = lock {
			let mut values: Vec<&str> = data[0].split(&[' ', ','][..]).collect();
			values.insert(0,&self.sender_id); // the parser expects a sender field

			match self.dictionary.find_msg_by_name(values[1]) {
				Some(mut msg) => {
					msg.update_from_string(&values);
					// append at the end (no priority)
					msg_queue.push_back(msg);
				}
				None => {
					println!("LinkIvySubscriber: Message not found: {}",&data[0]);
				}
			}
	    }
    }
    
    
    /// Bind to all messages from given sender
    pub fn ivy_bind_to_sender<F>(&mut self, cb: F, regexpr: String)
    where
        F: Fn(&mut LinkIvySubscriber, Vec<String>),
    {
        let regexpr = CString::new(regexpr).unwrap();
        {
	        let boxed_cb: Box<(Box<Fn(&mut LinkIvySubscriber, Vec<String>)>, &mut LinkIvySubscriber)> =
	            Box::new((Box::new(cb), self));
	        unsafe {
			        Some(IvyBindMsg(
			            apply_closure_sender_callback,
			            Box::into_raw(boxed_cb) as *const c_void,
			            regexpr.as_ptr(),
			        ))
	        };
        }
    }
}


#[allow(dead_code)]
extern "C" fn apply_closure_ping(_app: IvyClientPtr,
                            user_data: *mut c_void,
                            argc: i32,
                            argv: *const *const c_char) {
    let mut v: Vec<String> = vec![];
    for i in 0..argc as isize {
        unsafe {
            let ptr = argv.offset(i);
            v.push(String::from(CStr::from_ptr(*ptr).to_str().unwrap()));
        }
    }
    
    let payload: &mut (Box<Fn(&mut LinkIvyPing, Vec<String>) -> ()>, &mut LinkIvyPing) =
        unsafe { mem::transmute(user_data) };
    
    payload.0(&mut payload.1, v);
}

extern "C" fn apply_closure_sender_callback(_app: IvyClientPtr,
                            user_data: *mut c_void,
                            argc: i32,
                            argv: *const *const c_char) {
    let mut v: Vec<String> = vec![];
    for i in 0..argc as isize {
        unsafe {
            let ptr = argv.offset(i);
            v.push(String::from(CStr::from_ptr(*ptr).to_str().unwrap()));
        }
    }
    
    let payload: &mut (Box<Fn(&mut LinkIvySubscriber, Vec<String>) -> ()>, &mut LinkIvySubscriber) =
        unsafe { mem::transmute(user_data) };
    
    payload.0(&mut payload.1, v);
}