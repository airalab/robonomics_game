#!/usr/bin/env python

import ast

import rospy
from robonomics_game_warehouse.srv import Place, PlaceResponse
from robonomics_game_warehouse.srv import Order, OrderResponse
from robonomics_game_warehouse.srv import EmptyAll, EmptyAllResponse
from robonomics_game_warehouse.srv import FillAll, FillAllResponse
from robonomics_game_warehouse.srv import QuantityAvailable, QuantityAvailableResponse
from robonomics_game_warehouse.srv import Content, ContentResponse

from warehouse import Warehouse


def handle_place(request):
    position = wh.get_free_cell()
    if position is not None:
        wh.occupy_cell(position)
        offset = warehouse_offset
        return PlaceResponse(True, position[0] + offset[0], position[1] + offset[1])
    else:
        return PlaceResponse(False, 0, 0)


def handle_order(request):
    position = wh.get_occupied_cell()
    if position is not None:
        wh.free_cell(position)
        offset = warehouse_offset
        return OrderResponse(True, position[0] + offset[0], position[1] + offset[1])
    else:
        return OrderResponse(False, 0, 0)


def handle_fill_all(request):
    size = wh.get_size()
    for cell_pos in [(x, z) for x in range(1, size[0] + 1) for z in range(1, size[1] + 1)]:
        wh.occupy_cell([cell_pos[0] + warehouse_offset[0], cell_pos[1] + warehouse_offset[1]])
    return FillAllResponse()


def handle_empty_all(request):
    size = wh.get_size()
    for cell_pos in [(x, z) for x in range(1, size[0] + 1) for z in range(1, size[1] + 1)]:
        wh.free_cell([cell_pos[0] + warehouse_offset[0], cell_pos[1] + warehouse_offset[1]])
    return EmptyAllResponse()


def handle_quantity_available(request):
    return QuantityAvailableResponse(wh.get_quantity_available())


def handle_content(request):
    return ContentResponse(rospy.get_param('~content'))


def main():
    rospy.init_node('warehouse')

    if not rospy.has_param('~size'):
        raise rospy.ROSInitException('Parameter "size" must be set [length, height]')
    if not rospy.has_param('~offset'):
        raise rospy.ROSInitException('Parameter "offset" must be set [length, height]')
    if not rospy.has_param('~warehouse'):
        raise rospy.ROSInitException('Parameter "warehouse" muse be set "raws" or "goods"')
    if not rospy.has_param('~content'):
        raise rospy.ROSInitException('Items type must be specified in parameter "content"')

    warehouse_size = ast.literal_eval(rospy.get_param('~size'))
    global warehouse_offset
    warehouse_offset = ast.literal_eval(rospy.get_param('~offset'))

    global wh
    wh = Warehouse(warehouse_size)

    rospy.Service('~place', Place, handle_place)  # put item in cell
    rospy.Service('~order', Order, handle_order)  # get item from cell
    rospy.Service('~fill_all', FillAll, handle_fill_all)
    rospy.Service('~empty_all', EmptyAll, handle_empty_all)
    rospy.Service('~quantity_available', QuantityAvailable, handle_quantity_available)
    rospy.Service('~content', Content, handle_content)

    rospy.spin()


if __name__ == '__main__':
    main()
