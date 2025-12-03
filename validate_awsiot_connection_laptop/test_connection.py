#!/usr/bin/env python3
"""
Test AWS IoT Connection with Certificates
This verifies your certificates and AWS IoT setup are correct
"""

import ssl
import socket
import json
from datetime import datetime

# AWS IoT Configuration
AWS_IOT_ENDPOINT = "a3lkzcadhi1yzr-ats.iot.eu-west-1.amazonaws.com"
AWS_IOT_PORT = 8883
THING_NAME = "aviva-fov-tablet-1"

# Certificate files (update paths if needed)
CA_CERT = "ca.crt"
DEVICE_CERT = "device.crt"
DEVICE_KEY = "device.key"

def test_tls_connection():
    """Test TLS connection to AWS IoT"""
    print("="*60)
    print("AWS IoT Certificate Test")
    print("="*60)
    print(f"Endpoint: {AWS_IOT_ENDPOINT}:{AWS_IOT_PORT}")
    print(f"Thing Name: {THING_NAME}")
    print(f"CA Cert: {CA_CERT}")
    print(f"Device Cert: {DEVICE_CERT}")
    print(f"Device Key: {DEVICE_KEY}")
    print("="*60 + "\n")
    
    try:
        # Create SSL context with certificates
        print("[1/5] Creating SSL context...")
        context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
        context.load_cert_chain(certfile=DEVICE_CERT, keyfile=DEVICE_KEY)
        context.load_verify_locations(cafile=CA_CERT)
        context.verify_mode = ssl.CERT_REQUIRED
        context.check_hostname = True
        print("✅ SSL context created\n")
        
        # Create socket
        print("[2/5] Creating socket...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)
        print("✅ Socket created\n")
        
        # Connect to AWS IoT
        print(f"[3/5] Connecting to {AWS_IOT_ENDPOINT}:{AWS_IOT_PORT}...")
        sock.connect((AWS_IOT_ENDPOINT, AWS_IOT_PORT))
        print("✅ TCP connection established\n")
        
        # Wrap socket with SSL
        print("[4/5] Starting TLS handshake...")
        ssl_sock = context.wrap_socket(sock, server_hostname=AWS_IOT_ENDPOINT)
        print("✅ TLS handshake successful!\n")
        
        # Get certificate info
        print("[5/5] Retrieving certificate info...")
        cert = ssl_sock.getpeercert()
        print("✅ Certificate retrieved\n")
        
        print("="*60)
        print("TLS CONNECTION SUCCESS!")
        print("="*60)
        
        print("\n📋 Server Certificate Info:")
        print(f"  Subject: {dict(x[0] for x in cert['subject'])}")
        print(f"  Issuer: {dict(x[0] for x in cert['issuer'])}")
        print(f"  Version: {cert['version']}")
        print(f"  Not Before: {cert['notBefore']}")
        print(f"  Not After: {cert['notAfter']}")
        
        print("\n✅ Your certificates are VALID and working!")
        print("✅ AWS IoT endpoint is reachable")
        print("✅ TLS v1.2 connection successful")
        
        # Close connection
        ssl_sock.close()
        sock.close()
        
        print("\n" + "="*60)
        print("CONCLUSION")
        print("="*60)
        print("✅ Certificates work fine from your computer!")
        print("❌ The issue is likely with the C5 firmware:")
        print("   - Certificates might not be correctly flashed to C5")
        print("   - C5 AT firmware might have MQTT disabled")
        print("   - Certificate format issue in C5 flash")
        
        return True
        
    except ssl.SSLError as e:
        print(f"\n❌ SSL Error: {e}")
        print("\nPossible issues:")
        print("  - Certificate/key files are incorrect")
        print("  - Certificate expired or not yet valid")
        print("  - CA certificate doesn't match server cert")
        return False
        
    except socket.timeout:
        print(f"\n❌ Connection timeout")
        print("  - Check your internet connection")
        print("  - Firewall might be blocking port 8883")
        return False
        
    except FileNotFoundError as e:
        print(f"\n❌ Certificate file not found: {e}")
        print("\nMake sure these files are in the current directory:")
        print(f"  - {CA_CERT}")
        print(f"  - {DEVICE_CERT}")
        print(f"  - {DEVICE_KEY}")
        return False
        
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return False

if __name__ == "__main__":
    test_tls_connection()