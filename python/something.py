import numpy as np

count = 0
n = 20
for i1 in range(1, n):
    for i2 in range(i1+1, n):
        for i3 in range(i2+1, n):
            for i4 in range(i3+1, n):
            # for i5 in range(n, n):
                i = [i1, i2, i3, i4]
                if (i[0] % (i[0] - i[1])) == 0 and (i[1] % (i[0] - i[1])) == 0 and \
                    (i[1] % (i[1] - i[2])) == 0 and (i[2] % (i[1] - i[2])) == 0 and \
                    (i[2] % (i[2] - i[3])) == 0 and (i[3] % (i[2] - i[3])) == 0 and \
                    (i[0] % (i[0] - i[2])) == 0 and (i[2] % (i[0] - i[2])) == 0 and \
                    (i[0] % (i[0] - i[3])) == 0 and (i[3] % (i[0] - i[3])) == 0 and \
                    (i[1] % (i[1] - i[3])) == 0 and (i[3] % (i[1] - i[3])) == 0: # and \
                    # (i[0] % (i[0] - i[4])) == 0 and (i[0] % (i[0] - i[4])) == 0 and \
                    # (i[1] % (i[1] - i[4])) == 0 and (i[1] % (i[1] - i[4])) == 0 and \
                    # (i[2] % (i[2] - i[4])) == 0 and (i[2] % (i[2] - i[4])) == 0 and \
                    # (i[3] % (i[3] - i[4])) == 0 and (i[3] % (i[3] - i[4])) == 0:
                    print(i)
                    count += 1
                
                    

print(count)