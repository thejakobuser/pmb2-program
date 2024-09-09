import asyncio

async def my_coroutine():
    print("Starting coroutine...")
    await asyncio.sleep(2)  # Simulating a long-running operation
    print("Coroutine completed.")

async def main():
    print("Starting main function...")
    await my_coroutine()
    print("Main function completed.")

asyncio.run(main())