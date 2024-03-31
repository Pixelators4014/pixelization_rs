use std::io;
use std::net::{SocketAddr, UdpSocket};

fn main() -> io::Result<()> {
    // Bind the UDP socket to a local port
    let socket = UdpSocket::bind("10.40.14.11:5800")?;
    println!("UDP server listening on 10.40.14.11:7878");

    let mut buf = [0; 1024]; // Buffer to store incoming packets

    loop {
        // Wait for a packet to arrive and capture the sender's address
        let (amt, src) = socket.recv_from(&mut buf)?;

        println!("Received {} bytes from {}", amt, src);

        // Create a blank 25-byte packet
        let blank_packet = [0u8; 25];

        // Send the blank packet back to the sender
        match socket.send_to(&blank_packet, &src) {
            Ok(sent) => println!("Sent {} bytes back to {}", sent, src),
            Err(e) => eprintln!("Couldn't send a response: {}", e),
        }
    }
}
