def collision_Check(path, blocks):
    #path = (n,3)
    #blocks = (n,9) last three for RGB
    
    for i in range(path.shape[0] - 1):
        start = path[i,:]
        end = path[i+1,:]
        increment = (end - start) / 5
        for j in range(5):
            point = start + j * increment
            for block in range(blocks.shape[0]):
                if in_Block(point, blocks[block]) == True:
                    return True    
    return False


def in_Block(point, block):
    if point[0] >= block[0] and point[0] <= block[3]:
        if point[1] >= block[1] and point[1] <= block[4]:
            if point[2] >= block[2] and point[2] <= block[5]:
                return True
    return False