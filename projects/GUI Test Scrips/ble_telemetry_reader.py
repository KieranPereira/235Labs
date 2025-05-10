#!/usr/bin/env python3
"""
BLE Telemetry Reader for ESP32_Vibro_Test

This script connects to the ESP32 GATT server and subscribes to notifications
on the telemetry characteristic, printing each line of data as received.

Requires:
    pip install bleak
"""

import asyncio
from bleak import BleakScanner, BleakClient

# UUIDs must match those in your ESP32 sketch
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
TARGET_NAME = "ESP32_Vibro_Test"


def notification_handler(sender: int, data: bytearray):
    """
    Callback for handling incoming notifications.
    Decodes and prints the CSV telemetry line.
    """
    try:
        text = data.decode('utf-8').strip()
        print(f"[{sender}] {text}")
    except Exception as e:
        print(f"Error decoding data: {e}")


async def run():
    # Discover devices
    print("🔎 Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    target = None
    for d in devices:
        print(f" • {d.name} [{d.address}]")
        if d.name == TARGET_NAME or SERVICE_UUID.lower() in [u.lower() for u in d.metadata.get("uuids", [])]:
            target = d
            break

    if not target:
        print(f"❌ Could not find device named '{TARGET_NAME}' or advertising the service.")
        return

    print(f"✅ Found {target.name} at {target.address}")

    # Connect and subscribe to notifications
    async with BleakClient(target.address) as client:
        if not client.is_connected:
            print("❌ Failed to connect")
            return
        print("🔗 Connected. Subscribing to notifications...")

        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

        print("📡 Listening for telemetry. Press Ctrl+C to exit.")
        # Keep the program running to receive notifications
        while True:
            await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n👋 Exiting")
