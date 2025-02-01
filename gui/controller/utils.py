def isIterable(x):
    try:
        iter(x)
        return True
    except TypeError:
        return False