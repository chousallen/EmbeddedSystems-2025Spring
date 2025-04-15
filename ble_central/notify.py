import asyncio
from bleak import BleakScanner, BleakClient

# Define the target device name and the 128-bit service UUID
TARGET_DEVICE_NAME = "BlueNRG"  # Replace with your device's name
SERVICE_UUID_128_BIT = "00000000-0001-11e1-9ab4-0002a5d5c51b"  # Replace with your 128-bit service UUID

async def notification_handler(sender, data):
    print(f"Notification from {sender}: {data.hex()}")

async def main():
    # Step 1: Scan for the device by name.
    print(f"Scanning for device with name: {TARGET_DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(TARGET_DEVICE_NAME, timeout=10.0)

    if not device:
        print(f"Device with name '{TARGET_DEVICE_NAME}' not found.")
        return

    print(f"Found device: {device.name} [{device.address}]")

    # Connect to the device.
    async with BleakClient(device) as client:
        print(f"Connected to {device.name}")

        # Step 2: Retrieve and print all services.
        services = await client.get_services()
        print("Services:")
        for service in services:
            print(f"  [Service] {service.uuid}")
        
        # Step 3: Select the service with the specified 128-bit UUID.
        service = services.get_service(SERVICE_UUID_128_BIT)
        if not service:
            print(f"Service with UUID {SERVICE_UUID_128_BIT} not found.")
            return

        print(f"Found service: {service.uuid}")

        # Instead of directly writing to the CCCD, use start_notify for the desired characteristic.
        # Replace `characteristic_uuid` with the UUID of the characteristic you wish to monitor.
        characteristic_uuid = "00e00000-0001-11e1-ac36-0002a5d5c51b"  # Set this to your target characteristic UUID
        await client.start_notify(characteristic_uuid, notification_handler)
        print("Notifications enabled via start_notify.")

        # Keep the program running to receive notifications.
        await asyncio.sleep(30)

if __name__ == "__main__":
    asyncio.run(main())
