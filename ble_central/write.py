import asyncio
from bleak import BleakClient, BleakScanner

async def write_to_ble_device(device_name, service_uuid, characteristic_uuid, data):
    devices = await BleakScanner.discover()
    target_device = next((d for d in devices if d.name == device_name), None)

    if not target_device:
        print(f"Device with name '{device_name}' not found.")
        return

    async with BleakClient(target_device.address) as client:
        if not client.is_connected:
            print(f"Failed to connect to device '{device_name}'.")
            return

        try:
            await client.write_gatt_char(characteristic_uuid, data)
            print(f"Data written to characteristic {characteristic_uuid}.")
        except Exception as e:
            print(f"Failed to write to characteristic: {e}")

if __name__ == "__main__":
    device_name = "BlueNRG"
    service_uuid = "00000000-0002-11e1-9ab4-0002a5d5c51b"
    characteristic_uuid = "01020304-0506-0708-090a-0b0c0d0e0f00"
    data = input("Enter the data to write (in hex, e.g., '0x01'): ")

    # Convert hex string to bytes
    data_bytes = bytes.fromhex(data[2:]) if data.startswith("0x") else data.encode()

    asyncio.run(write_to_ble_device(device_name, service_uuid, characteristic_uuid, data_bytes))