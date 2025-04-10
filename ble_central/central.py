import asyncio
from bleak import BleakScanner, BleakClient

async def main():
    print("🔍 Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=5)

    for i, device in enumerate(devices):
        print(f"[{i}] {device.name} ({device.address})")

    if not devices:
        print("No BLE devices found.")
        return

    # Select the device with the name "eslab-pi"
    device_name = "Allen's A55"
    selected_device = next((device for device in devices if device.name == device_name), None)
    if not selected_device:
        print(f"\n❌ Device with name {device_name} not found.")
        return
    print(f"\n🔗 Connecting to {selected_device.name} ({selected_device.address})...")

    async with BleakClient(selected_device.address) as client:
        connected = await client.is_connected()
        print(f"✅ Connected: {connected}")

        # List services and characteristics
        print("\n📡 Services:")
        for service in client.services:
            print(f"{service.uuid}:")
            for char in service.characteristics:
                print(f"  └─ {char.uuid} (properties: {char.properties})")

        # Example: read a characteristic (change UUID to one from above)
        # data = await client.read_gatt_char(0x181c)
        # print("📥 Data:", data)

asyncio.run(main())
