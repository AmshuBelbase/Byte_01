#!/usr/bin/env python3
"""
Socket-based Sender for Motor Controller
Sends numpy arrays to receiver via TCP socket
"""

import numpy as np
import socket
import pickle
import time


def send_array_to_motor(array, host='127.0.0.1', port=50000, timeout=2.0):
    """
    Send numpy array to motor controller via socket
    
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        
        # Serialize and send array
        data = pickle.dumps(array)
        sock.sendall(data)
        sock.close()
        
        return True
        
    except ConnectionRefusedError:
        print("❌ Connection refused - Is receiver running?")
        return False
    except socket.timeout:
        print("❌ Connection timeout")
        return False
    except Exception as e:
        print(f"❌ Send error: {e}")
        return False


def test_connection(host='127.0.0.1', port=50000):
    """Test if receiver is available"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.connect((host, port))
        sock.close()
        return True
    except:
        return False


if __name__ == '__main__':
    HOST = '127.0.0.1'
    PORT = 50000
    
    print("="*70)
    print("Motor Controller Socket Sender")
    print("="*70)
    
    # Test connection
    print(f"Testing connection to {HOST}:{PORT}...", end=" ")
    if test_connection(HOST, PORT):
        print("✅ Connected!")
    else:
        print("❌ Failed!")
        print("\n⚠️  Make sure motor_controller_receiver.py is running first!")
        print("   Start it in another terminal and try again.")
        
        retry = input("\nRetry connection? (y/n): ").strip().lower()
        if retry != 'y':
            exit(1)
        
        if not test_connection(HOST, PORT):
            print("❌ Still cannot connect. Exiting.")
            exit(1)
        print("✅ Connected on retry!")
    
    # Predefined destinations
    destinations = {
        '1': np.array([10.0, 40.0, 60.0]),
        '2': np.array([30.0, 50.0, 0.0]),
        '3': np.array([0.0, 0.0, 0.0]),
    }
    
    print("\n📋 Available commands:")
    print("  1 - Move to [10°, 40°, 60°]")
    print("  2 - Move to [30°, 50°, 0°]")
    print("  3 - Move to [0°, 0°, 0°] (home)")
    print("  q - Quit")
    print()
    
    while True:
        cmd = input("Cmd (1/2/3/q): ").strip()
        
        if cmd.lower() == 'q':
            print("Exiting sender...")
            break
            
        if cmd in destinations:
            array = destinations[cmd]
            
            if send_array_to_motor(array, HOST, PORT):
                print(f"✅ Sent: {array.tolist()}°")
            else:
                print("⚠️  Send failed - receiver may have disconnected")
                
        else:
            print("❌ Invalid command. Use 1/2/3/q")
    
    print("Sender closed.")
