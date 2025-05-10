import asyncio
from bleak import BleakClient, BleakScanner

# Nordic UART Service (NUS) UUIDs
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Notification from ESP32
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Write to ESP32

DEVICE_NAME = "ESP32_Vibro_new"

async def run():
    print(f"Scanning for BLE device named '{DEVICE_NAME}'...")
    devices = await BleakScanner.discover()
    target = None
    for d in devices:
        if d.name == DEVICE_NAME:
            target = d
            break

    if not target:
        print(f"Device '{DEVICE_NAME}' not found. Make sure it's advertising and in range.")
        return

    print(f"Found device {DEVICE_NAME} [{target.address}], connecting...")
    async with BleakClient(target.address) as client:
        if not client.is_connected:
            print("Failed to connect")
            return
        print("Connected! Subscribing to telemetry notifications...")

        def notification_handler(sender, data):
            # data is a bytearray
            text = data.decode('utf-8', errors='ignore').strip()
            print(f"> {text}")

        await client.start_notify(UART_TX_CHAR_UUID, notification_handler)

        print("Listening for telemetry. Press Ctrl+C to exit.")
        try:
            # Keep the script running to receive notifications
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("Disconnecting...")
        finally:
            await client.stop_notify(UART_TX_CHAR_UUID)
            print("Disconnected.")

if __name__ == "__main__":
    asyncio.run(run())

# Instructions:
# 1. pip install bleak
# 2. Save this script as esp32_ble_telemetry.py
# 3. Run: python esp32_ble_telemetry.py
