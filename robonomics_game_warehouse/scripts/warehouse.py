#!/usr/bin/env python


class Warehouse:

    def __init__(self, warehouse_size):
        if not isinstance(warehouse_size, list):
            raise TypeError('Warehouse size type must be a list [length, height]')
        self.size = warehouse_size
        self.capacity = self.size[0] * self.size[1]
        self.state = [[None for _ in range(self.size[0] + 1)] for _ in range(self.size[1] + 1)]

    def get_size(self):
        return self.size

    def get_capacity(self):
        return self.capacity

    def get_quantity_available(self):
        return sum(_.count(True) for _ in self.state)

    def get_free_cell(self):
        col = [col for col in self.state if False in col]
        if (col):
            pos = [self.state.index(col[0]) + 1, col[0].index(False) + 1]
            return pos
        else:
            return None

    def get_occupied_cell(self):
        col = [col for col in self.state if True in col]
        if (col):
            pos = [self.state.index(col[0]) + 1, col[0].index(True) + 1]
            return pos
        else:
            return None

    def free_cell(self, pos):
        self.state[pos[0] - 1][pos[1] - 1] = False

    def occupy_cell(self, pos):
        self.state[pos[0] - 1][pos[1] - 1] = True

    def _trace(self):
        attrs = vars(self)
        print(', '.join("%s: %s" % item for item in attrs.items()))


if __name__ == '__main__':
    wh = Warehouse([2, 3])
    wh.occupy_cell([1, 1])
    wh._trace()
