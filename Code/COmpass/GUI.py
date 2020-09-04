for i in range(0,500):
    x_out = read_word_2c(3)
    y_out = read_word_2c(7)
    z_out = read_word_2c(5)
    
    bearing  = math.atan2(y_out, x_out) 
    if (bearing < 0):
        bearing += 2 * math.pi
    
    print x_out, y_out, (x_out * scale), (y_out * scale)
    time.sleep(0.1)