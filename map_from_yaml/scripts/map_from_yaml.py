#!/usr/bin/env python3
import argparse
import math

import cv2
import numpy as np
import yaml

OBS_RADIUS = 1.0


def main():
    parser = argparse.ArgumentParser(description='Generate map.png from a yaml file')
    parser.add_argument('-i', type=str, required=True)
    parser.add_argument('-o', type=str, required=True)
    args = parser.parse_args()
    yaml.warnings({'YAMLLoadWarning': False})
    f = open(args.i, 'r', encoding='utf-8')
    cfg = yaml.load(f.read())
    f.close()
    map_size = 200, 200
    dimensions, obstacles = np.array(cfg['map']['dimensions']), np.array(cfg['map']['obstacles'])
    map_image = np.zeros(map_size, dtype=np.uint8)

    def c2d(a, i):
        return int(a / dimensions[i] * map_size[i])

    def d2c(a, i):
        return a / map_size[i] * dimensions[i]

    for ob in obstacles:
        ob[1] = dimensions[1] - ob[1]
        ob[0], ob[1] = ob[1], ob[0]
        x_range = max(0, c2d(ob[0] - OBS_RADIUS, 0)), min(map_size[0], c2d(ob[0] + OBS_RADIUS, 0) + 1)
        y_range = max(0, c2d(ob[1] - OBS_RADIUS, 0)), min(map_size[0], c2d(ob[1] + OBS_RADIUS, 0) + 1)
        for x in range(*x_range):
            for y in range(*y_range):
                xc, yc = d2c(x, 0), d2c(y, 1)
                if math.hypot(xc - ob[0], yc - ob[1]) < OBS_RADIUS:
                    map_image[x, y] = 255
    x_edge, y_edge = int(map_size[0] / 10), int(map_size[1] / 10)
    edge = np.zeros([map_size[0] + 2 * x_edge, map_size[1] + 2 * y_edge], dtype=np.uint8)
    edge.fill(255)
    edge[x_edge:map_size[0] + x_edge, y_edge:map_size[1] + y_edge] = map_image
    edge = cv2.resize(edge, (129, 129))
    print('Done')
    cv2.imwrite(args.o, edge)


if __name__ == '__main__':
    main()
