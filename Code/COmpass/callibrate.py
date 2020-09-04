minx = 0
maxx = 0
miny = 0
maxy = 0

for i in range(0,500):
    x_out = read_word_2c(3)
    y_out = read_word_2c(7)
    z_out = read_word_2c(5)
    
    
    if x_out < minx:
        minx=x_out
    
    if y_out < miny:
        miny=y_out
    
    if x_out > maxx:
        maxx=x_out
    
    if y_out > maxy:
        maxy=y_out
    
    #print x_out, y_out, (x_out * scale), (y_out * scale)
    time.sleep(0.1)

print "minx: ", minx
print "miny: ", miny
print "maxx: ", maxx
print "maxy: ", maxy
print "x offset: ", (maxx + minx) / 2
print "y offset: ", (maxy + miny) / 2