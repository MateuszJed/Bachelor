import logging
import asyncio
import sys
sys.path.insert(0, "..")

from asyncua import ua, Server
from asyncua.common.methods import uamethod

@uamethod
def func(parent, value):
    return value * 2

async def main():
    _logger = logging.getLogger('asyncua')
    # setup our server
    server = Server()
    await server.init()
    server.set_endpoint('opc.tcp://169.254.182.5:8000')
    # server.set_endpoint('opc.tcp://169.254.182.5:8000')


    val = 1
    # populating our address space
    # server.nodes, contains links to very common nodes like objects and root
    actual_position = await server.nodes.objects.add_object(2, 'actual_position')
    actual_position_x = await actual_position.add_variable(2, 'actual_position_x', 0.000)
    actual_position_y = await actual_position.add_variable(3, 'actual_position_y', 0.000)
    actual_position_z = await actual_position.add_variable(4, 'actual_position_z', 0.000)

    angle_position = await server.nodes.objects.add_object(3, 'angle_position')
    angle_position_1 = await angle_position.add_variable(5, 'angle_position_1', 0.000)
    angle_position_2 = await angle_position.add_variable(6, 'angle_position_2', 0.000)
    angle_position_3 = await angle_position.add_variable(7, 'angle_position_3', 0.000)

    # Set MyVariable to be writable by clients
    await actual_position_x.set_writable()
    await actual_position_y.set_writable()
    await actual_position_z.set_writable()

    await angle_position_1.set_writable()
    await angle_position_2.set_writable()
    await angle_position_3.set_writable()

    await server.nodes.objects.add_method(ua.NodeId('ServerMethod', 2), ua.QualifiedName('ServerMethod', 2), func, [ua.VariantType.Float], [ua.VariantType.Float])
    _logger.info('Starting server!')
    async with server:
        while True:
            await asyncio.sleep(1)

            _logger.info('#################')
            _logger.info('Actual Position')
            _logger.info('Set value of %s to %.3f', actual_position_x, await actual_position_x.get_value())
            _logger.info('Set value of %s to %.3f', actual_position_y, await actual_position_y.get_value())
            _logger.info('Set value of %s to %.3f', actual_position_z, await actual_position_z.get_value())
            _logger.info('Angle')
            _logger.info('Set value of %s to %.3f', angle_position_1, await angle_position_1.get_value())
            _logger.info('Set value of %s to %.3f', angle_position_2, await angle_position_2.get_value())
            _logger.info('Set value of %s to %.3f', angle_position_3, await angle_position_3.get_value())
            _logger.info('#################\n')




if __name__ == '__main__':

    logging.basicConfig(level=logging.INFO)

    asyncio.run(main(), debug=False)