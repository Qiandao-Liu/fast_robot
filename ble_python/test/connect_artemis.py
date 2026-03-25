import asyncio
from bleak import BleakScanner, BleakClient

NAME = "Artemis BLE"
TIMEOUT = 8.0

async def main():
    devices = await BleakScanner.discover(timeout=5)
    targets = [d for d in devices if (getattr(d, "name", None) or "") == NAME]
    if not targets:
        print("Not found:", NAME)
        return

    d = targets[0]
    print("Connecting to:", d.address, d.name)

    async with BleakClient(d.address, timeout=TIMEOUT) as client:
        print("Connected:", client.is_connected)

        svcs = None
        if hasattr(client, "get_services"):
            svcs = await client.get_services()
        else:
            svcs = client.services

        if svcs is None:
            print("No services object available")
            return

        for s in svcs:
            print("SERVICE", s.uuid)
            for c in s.characteristics:
                props = ",".join(getattr(c, "properties", []))
                print("  CHAR", c.uuid, props)

if __name__ == "__main__":
    asyncio.run(main())
