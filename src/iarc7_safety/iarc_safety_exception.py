#!/usr/bin/env python

class IARCSafetyException(RuntimeError):
    pass

class IARCFatalSafetyException(IARCSafetyException):
    pass
