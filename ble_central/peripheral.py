#!/usr/bin/env python3

from bluezero import peripheral

# Define UUIDs for your custom service and characteristic.
MY_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
MY_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"

# Define a read callback that returns a string (as a bytearray)
def read_callback():
    # This is the data the client will receive when reading the characteristic.
    return bytearray('Hello, BLE!', 'utf-8')

# Replace this with your Bluetooth adapter MAC address.
# For example, you could find it with `hciconfig` (it might look like 'B8:27:EB:XX:XX:XX')
adapter_address = 'B8:27:EB:2A:94:70'

# Create a Peripheral object with the local name you'd like to broadcast.
my_peripheral = peripheral.Peripheral(adapter_address=adapter_address,
                                        local_name='MyGATTServer')

# Add a primary service to your GATT server.
my_peripheral.add_service(srv_id=1, uuid=MY_SERVICE_UUID, primary=True)

# Add a characteristic to the service created above.
# In this example, the characteristic is set up as a read-only value.
my_peripheral.add_characteristic(srv_id=1,
                                 chr_id=1,
                                 uuid=MY_CHAR_UUID,
                                 value=[],  # Initial value is empty.
                                 notifying=False,
                                 flags=['read'],  # You can add 'write' if needed.
                                 read_callback=read_callback)

# Publish the GATT application – the service is now active and advertised.
print("Publishing GATT service...")
my_peripheral.publish()

