x_start = -1200
x_end = 1200
y_start = -1200
y_end = 1200
discretize_loc = 100
x_bins = [e for e in range(x_start,x_end+1,discretize_loc)]
y_bins = [e for e in range(y_start,y_end+1,discretize_loc)]
# xy_bins = [-800,-705,-611,-517,-423,-329,-235,-141,-47,47,141,235,329,423,517,611,705,800]
t_start = 90
t_end = 270
discretize_trans = 30
t_bins = [e for e in range(t_start,t_end+1,discretize_trans)]
forward = "'forward'"
back = "'back'"
right = "'right'"
left = "'left'"
forwardleft = "'forwardleft'"
forwardright = "'forwardright'"
stop = "'stop'"
turnleft = "'turnleft'"
turnright = "'turnright'"

t_f = open("hard_policy.txt","w")
t_f.write("policy = dict()\n\n")
for theta in t_bins:
        for x in x_bins:
                for y in y_bins:
                        if theta>220:
                                t_f.write("policy["+"("+str(x)+","+str(y)+","+str(theta)+")] = " + turnleft+"\n")
                        elif theta<150:
                                t_f.write("policy["+"("+str(x)+","+str(y)+","+str(theta)+")] = " + turnright+"\n")
                        elif x>150 and y>150:
                                t_f.write("policy["+"("+str(x)+","+str(y)+","+str(theta)+")] = " + forwardleft+"\n")
                        elif x>150 and y<-150:
                                t_f.write("policy["+"("+str(x)+","+str(y)+","+str(theta)+")] = " + forwardright+"\n")
                        elif x<-150:
                                t_f.write("policy["+"("+str(x)+","+str(y)+","+str(theta)+")] = " + back+"\n")
                        elif y>150:
                                t_f.write("policy["+"("+str(x)+","+str(y)+","+str(theta)+")] = " + left+"\n")
                        elif y<-150:
                                t_f.write("policy["+"("+str(x)+","+str(y)+","+str(theta)+")] = " + right+"\n")
                        else:
                                t_f.write("policy["+"("+str(x)+","+str(y)+","+str(theta)+")] = " + stop+"\n")