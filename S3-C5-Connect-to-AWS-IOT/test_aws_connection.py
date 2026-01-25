#!/usr/bin/env python3
"""
Test AWS IoT connection using the same certificates as the C5 chip.
This helps verify if the issue is with certificates or the C5 firmware.
"""

import ssl
import socket
import sys

# AWS IoT endpoint and port
AWS_IOT_ENDPOINT = "a3lkzcadhi1yzr-ats.iot.ap-southeast-2.amazonaws.com"
MQTT_PORT = 8883

# Certificate paths - using the Sydney certs that should be flashed to C5
CA_CERT = "src/example/sydney/ca.crt"
DEVICE_CERT = "src/example/sydney/device.crt"
DEVICE_KEY = "src/example/sydney/device.key"

def test_tcp_connection():
    """Test basic TCP connection to AWS IoT endpoint."""
    print(f"\n[1] Testing TCP connection to {AWS_IOT_ENDPOINT}:{MQTT_PORT}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(30)
        sock.connect((AWS_IOT_ENDPOINT, MQTT_PORT))
        print(f"    TCP connection successful!")
        sock.close()
        return True
    except Exception as e:
        print(f"    TCP connection failed: {e}")
        return False

def test_tls_connection():
    """Test TLS connection with mutual authentication."""
    print(f"\n[2] Testing TLS connection with client certificates...")

    # Read and clean the CA cert (remove any artifacts)
    try:
        with open(CA_CERT, 'r') as f:
            ca_content = f.read()
        # Remove the )EOF"; artifact if present
        if ')EOF"' in ca_content:
            ca_content = ca_content.split('-----END CERTIFICATE-----')[0] + '-----END CERTIFICATE-----\n'
            # Write cleaned version
            with open(CA_CERT + '.clean', 'w') as f:
                f.write(ca_content)
            ca_cert_path = CA_CERT + '.clean'
            print(f"    Note: Cleaned CA cert (removed artifact)")
        else:
            ca_cert_path = CA_CERT
    except FileNotFoundError:
        print(f"    ERROR: CA cert not found at {CA_CERT}")
        return False

    try:
        # Create SSL context
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.verify_mode = ssl.CERT_REQUIRED
        context.check_hostname = True

        # Load certificates
        print(f"    Loading CA cert: {ca_cert_path}")
        context.load_verify_locations(ca_cert_path)

        print(f"    Loading device cert: {DEVICE_CERT}")
        print(f"    Loading device key: {DEVICE_KEY}")
        context.load_cert_chain(certfile=DEVICE_CERT, keyfile=DEVICE_KEY)

        # Create socket and wrap with TLS
        print(f"    Connecting to {AWS_IOT_ENDPOINT}:{MQTT_PORT}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(60)  # 60 second timeout for TLS handshake

        tls_sock = context.wrap_socket(sock, server_hostname=AWS_IOT_ENDPOINT)
        tls_sock.connect((AWS_IOT_ENDPOINT, MQTT_PORT))

        print(f"\n    TLS connection successful!")
        print(f"    TLS version: {tls_sock.version()}")
        print(f"    Cipher: {tls_sock.cipher()}")

        # Get server certificate info
        cert = tls_sock.getpeercert()
        if cert:
            print(f"    Server CN: {dict(x[0] for x in cert['subject'])}")

        tls_sock.close()
        return True

    except ssl.SSLCertVerificationError as e:
        print(f"\n    SSL Certificate Verification Error: {e}")
        print("    This usually means the CA certificate is wrong or doesn't match the server.")
        return False
    except ssl.SSLError as e:
        print(f"\n    SSL Error: {e}")
        return False
    except FileNotFoundError as e:
        print(f"\n    File not found: {e}")
        return False
    except Exception as e:
        print(f"\n    Connection failed: {type(e).__name__}: {e}")
        return False

def test_mqtt_connection():
    """Test actual MQTT connection to AWS IoT."""
    print(f"\n[3] Testing MQTT connection...")

    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("    paho-mqtt not installed. Run: pip install paho-mqtt")
        return False

    # Read and clean the CA cert
    try:
        with open(CA_CERT, 'r') as f:
            ca_content = f.read()
        if ')EOF"' in ca_content:
            ca_content = ca_content.split('-----END CERTIFICATE-----')[0] + '-----END CERTIFICATE-----\n'
            with open(CA_CERT + '.clean', 'w') as f:
                f.write(ca_content)
            ca_cert_path = CA_CERT + '.clean'
        else:
            ca_cert_path = CA_CERT
    except FileNotFoundError:
        print(f"    ERROR: CA cert not found")
        return False

    connected = [False]
    error_msg = [None]

    def on_connect(client, userdata, flags, rc, properties=None):
        if rc == 0:
            print(f"    MQTT connected successfully!")
            connected[0] = True
        else:
            error_codes = {
                1: "Connection refused - incorrect protocol version",
                2: "Connection refused - invalid client identifier",
                3: "Connection refused - server unavailable",
                4: "Connection refused - bad username or password",
                5: "Connection refused - not authorized"
            }
            error_msg[0] = error_codes.get(rc, f"Unknown error code: {rc}")
            print(f"    MQTT connection failed: {error_msg[0]}")

    def on_disconnect(client, userdata, rc, properties=None):
        if rc != 0:
            print(f"    Unexpected disconnection: {rc}")

    try:
        # Create MQTT client (using MQTT v5 or v3.1.1)
        try:
            # Try MQTT v5 first (paho-mqtt 2.x)
            client = mqtt.Client(client_id="kia-tennis-fov-tablet-20", protocol=mqtt.MQTTv5)
        except (AttributeError, TypeError):
            # Fall back to older API (paho-mqtt 1.x)
            client = mqtt.Client(client_id="kia-tennis-fov-tablet-20")

        client.on_connect = on_connect
        client.on_disconnect = on_disconnect

        # Configure TLS
        client.tls_set(
            ca_certs=ca_cert_path,
            certfile=DEVICE_CERT,
            keyfile=DEVICE_KEY,
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS
        )

        print(f"    Connecting to {AWS_IOT_ENDPOINT}:{MQTT_PORT}...")
        print(f"    Client ID: kia-tennis-fov-tablet-20")

        client.connect(AWS_IOT_ENDPOINT, MQTT_PORT, keepalive=120)

        # Wait for connection
        client.loop_start()
        import time
        timeout = 30
        while not connected[0] and error_msg[0] is None and timeout > 0:
            time.sleep(1)
            timeout -= 1
            print(f"    Waiting... ({30-timeout}s)", end='\r')

        print()
        client.loop_stop()
        client.disconnect()

        return connected[0]

    except Exception as e:
        print(f"    MQTT connection error: {type(e).__name__}: {e}")
        return False

def print_cert_info():
    """Print certificate information."""
    print("\n[0] Certificate Information:")
    print("=" * 50)

    try:
        import subprocess

        # Check CA cert
        print(f"\n  CA Certificate ({CA_CERT}):")
        result = subprocess.run(
            ['openssl', 'x509', '-in', CA_CERT, '-noout', '-subject', '-issuer', '-dates'],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            print(f"  {result.stdout}")
        else:
            # Try with cleaned version
            with open(CA_CERT, 'r') as f:
                ca_content = f.read()
            if ')EOF"' in ca_content:
                print("    (Contains artifact, will be cleaned)")

        # Check device cert
        print(f"\n  Device Certificate ({DEVICE_CERT}):")
        result = subprocess.run(
            ['openssl', 'x509', '-in', DEVICE_CERT, '-noout', '-subject', '-issuer', '-dates'],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            print(f"  {result.stdout}")

    except FileNotFoundError:
        print("  openssl not found - skipping detailed cert info")
    except Exception as e:
        print(f"  Error reading certs: {e}")

def main():
    print("=" * 60)
    print("AWS IoT Connection Test")
    print("=" * 60)
    print(f"\nEndpoint: {AWS_IOT_ENDPOINT}")
    print(f"Port: {MQTT_PORT}")

    print_cert_info()

    # Test 1: TCP
    tcp_ok = test_tcp_connection()

    # Test 2: TLS
    if tcp_ok:
        tls_ok = test_tls_connection()
    else:
        print("\n[2] Skipping TLS test (TCP failed)")
        tls_ok = False

    # Test 3: MQTT
    if tls_ok:
        mqtt_ok = test_mqtt_connection()
    else:
        print("\n[3] Skipping MQTT test (TLS failed)")
        mqtt_ok = False

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"TCP Connection:  {'PASS' if tcp_ok else 'FAIL'}")
    print(f"TLS Connection:  {'PASS' if tls_ok else 'FAIL'}")
    print(f"MQTT Connection: {'PASS' if mqtt_ok else 'FAIL'}")

    if mqtt_ok:
        print("\nCertificates are valid! The issue is likely with the C5 chip configuration.")
        print("Check if the certificates are properly flashed to the C5.")
    elif tls_ok:
        print("\nTLS works but MQTT fails - check AWS IoT Thing policy and client ID.")
    elif tcp_ok:
        print("\nTCP works but TLS fails - certificate issue (wrong CA, expired, or mismatched).")
    else:
        print("\nCannot reach AWS IoT - check network/firewall.")

    return 0 if mqtt_ok else 1

if __name__ == "__main__":
    sys.exit(main())
