import socket

def get_ip_address():
    try:
        # Get the hostname of the machine
        hostname = socket.gethostname()
        # Get the IP address associated with the hostname
        ip_address = socket.gethostbyname(hostname)
    except Exception as e:
        print(f"Error obtaining IP address: {e}")
        ip_address = "Unable to determine IP"
    
    return ip_address

# Get and print the IP address
ip = get_ip_address()
print(f"My IP address is: {ip}")