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

        # 兼容不同 bleak 版本：优先 get_services，否则用 client.services
        svcs = None
        if hasattr(client, "get_services"):
            svcs = await client.get_services()
        else:
            # 某些版本需要先访问一次 services 触发枚举
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
