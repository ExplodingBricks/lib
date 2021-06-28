import inspect


def nicePrint(text=""):
    print(inspect.currentframe().f_back.f_code.co_names[0] + ": " + text)
