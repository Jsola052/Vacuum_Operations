path = [[0.100, 0.200, 0.100],
        [0.100, 0.400, 0.100],
        [0.150, 0.400, 0.100],
        [0.300, 0.400, 0.100],
        [0.300, 0.300, 0.100]]
for x in path:
    print(x)
    temp_vector = path[path.index(x) + 1] - x
    print ("temp_vector = " + temp_vector)