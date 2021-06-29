import inspect


def nicePrint(*argv):
    """Helper function to print with caller name. Accepts infinite args to be printed.
    """
    print(str(inspect.currentframe().f_back.f_code.co_name) + ":", *argv)
