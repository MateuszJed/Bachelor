import asyncio
import sys
# sys.path.insert(0, "..")
import logging
from asyncua import Client, Node, ua

logging.basicConfig(level=logging.INFO)
_logger = logging.getLogger('asyncua')


async def main():
    url = 'opc.tcp://169.254.182.5:8000'
    async with Client(url=url) as client:
        # Client has a few methods to get proxy to UA nodes that should always be in address space such as Root or Objects
        while 1:
            tip_x = client.get_node("ns=5;i=1")
            tip_y = client.get_node("ns=6;i=1")
            tip_z = client.get_node("ns=7;i=1")

            print(await tip_x.read_value(), await tip_y.read_value(),await tip_z.read_value())

if __name__ == '__main__':
    asyncio.run(main())