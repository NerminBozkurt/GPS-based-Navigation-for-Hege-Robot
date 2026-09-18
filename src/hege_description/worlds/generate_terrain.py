import math

size = 257
filename = '/workspace/src/hege_description/worlds/field_heightmap.pgm'

with open(filename, 'wb') as f:
    f.write(f'P5\n{size} {size}\n255\n'.encode('ascii'))
    for y in range(size):
        for x in range(size):
            # Normalizacja od 0 do 2*PI (podstawowe fale)
            sx = (x / size) * 4.0 * math.pi
            sy = (y / size) * 4.0 * math.pi
            
            # Łagodne pagórki
            hills = math.sin(sx) * math.cos(sy)
            
            # Bruzdy poprzeczne (efekt zaoranego pola)
            furrows = math.sin(x / size * 50.0 * math.pi) * 0.3
            
            val = hills + furrows
            # Skalowanie od około -1.3 do 1.3 -> 0 do 255
            val = (val + 1.3) / 2.6
            val = max(0, min(255, int(val * 255)))
            
            f.write(bytes([val]))

print('Mapa wysokosci wygenerowana!')
