import asyncio
async def x():
    while True:
        print("hi")
        await asyncio.sleep(0)
async def y():
    for i in range(10):
        asyncio.get_event_loop().create_task(z())
        await asyncio.sleep(1)
async def z():
    while True:
        print("bye")
        await asyncio.sleep(0)
print(asyncio.get_event_loop())

asyncio.get_event_loop().create_task(y())
asyncio.get_event_loop().create_task(x())

asyncio.get_event_loop().run_forever()
