def mrange( a, l):
        minA = a - l
        r = []
        for i in range(0,l*2+1):
            r.append(minA + i)
        return r
    
print(mrange(1,2))
print(mrange(-5,1))