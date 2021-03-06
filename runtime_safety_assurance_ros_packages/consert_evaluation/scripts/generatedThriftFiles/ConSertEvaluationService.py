'''
MIT License

Copyright (c) 2019 DEIS Project

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
#
# Autogenerated by Thrift Compiler (0.11.0)
#
# DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
#
#  options string: py
#

from thrift.Thrift import TType, TMessageType, TFrozenDict, TException, TApplicationException
from thrift.protocol.TProtocol import TProtocolException
from thrift.TRecursive import fix_spec

import sys
import logging
from .ttypes import *
from thrift.Thrift import TProcessor
from thrift.transport import TTransport
all_structs = []


class Iface(object):
    def EvaluateConSerts(self, conSertEvaluationConfiguration):
        """
        Parameters:
         - conSertEvaluationConfiguration
        """
        pass


class Client(Iface):
    def __init__(self, iprot, oprot=None):
        self._iprot = self._oprot = iprot
        if oprot is not None:
            self._oprot = oprot
        self._seqid = 0

    def EvaluateConSerts(self, conSertEvaluationConfiguration):
        """
        Parameters:
         - conSertEvaluationConfiguration
        """
        self.send_EvaluateConSerts(conSertEvaluationConfiguration)
        return self.recv_EvaluateConSerts()

    def send_EvaluateConSerts(self, conSertEvaluationConfiguration):
        self._oprot.writeMessageBegin('EvaluateConSerts', TMessageType.CALL, self._seqid)
        args = EvaluateConSerts_args()
        args.conSertEvaluationConfiguration = conSertEvaluationConfiguration
        args.write(self._oprot)
        self._oprot.writeMessageEnd()
        self._oprot.trans.flush()

    def recv_EvaluateConSerts(self):
        iprot = self._iprot
        (fname, mtype, rseqid) = iprot.readMessageBegin()
        if mtype == TMessageType.EXCEPTION:
            x = TApplicationException()
            x.read(iprot)
            iprot.readMessageEnd()
            raise x
        result = EvaluateConSerts_result()
        result.read(iprot)
        iprot.readMessageEnd()
        if result.success is not None:
            return result.success
        if result.EpsilonScriptExecutionException is not None:
            raise result.EpsilonScriptExecutionException
        raise TApplicationException(TApplicationException.MISSING_RESULT, "EvaluateConSerts failed: unknown result")


class Processor(Iface, TProcessor):
    def __init__(self, handler):
        self._handler = handler
        self._processMap = {}
        self._processMap["EvaluateConSerts"] = Processor.process_EvaluateConSerts

    def process(self, iprot, oprot):
        (name, type, seqid) = iprot.readMessageBegin()
        if name not in self._processMap:
            iprot.skip(TType.STRUCT)
            iprot.readMessageEnd()
            x = TApplicationException(TApplicationException.UNKNOWN_METHOD, 'Unknown function %s' % (name))
            oprot.writeMessageBegin(name, TMessageType.EXCEPTION, seqid)
            x.write(oprot)
            oprot.writeMessageEnd()
            oprot.trans.flush()
            return
        else:
            self._processMap[name](self, seqid, iprot, oprot)
        return True

    def process_EvaluateConSerts(self, seqid, iprot, oprot):
        args = EvaluateConSerts_args()
        args.read(iprot)
        iprot.readMessageEnd()
        result = EvaluateConSerts_result()
        try:
            result.success = self._handler.EvaluateConSerts(args.conSertEvaluationConfiguration)
            msg_type = TMessageType.REPLY
        except TTransport.TTransportException:
            raise
        except TAbstractEpsilonScriptExecutionException as EpsilonScriptExecutionException:
            msg_type = TMessageType.REPLY
            result.EpsilonScriptExecutionException = EpsilonScriptExecutionException
        except TApplicationException as ex:
            logging.exception('TApplication exception in handler')
            msg_type = TMessageType.EXCEPTION
            result = ex
        except Exception:
            logging.exception('Unexpected exception in handler')
            msg_type = TMessageType.EXCEPTION
            result = TApplicationException(TApplicationException.INTERNAL_ERROR, 'Internal error')
        oprot.writeMessageBegin("EvaluateConSerts", msg_type, seqid)
        result.write(oprot)
        oprot.writeMessageEnd()
        oprot.trans.flush()

# HELPER FUNCTIONS AND STRUCTURES


class EvaluateConSerts_args(object):
    """
    Attributes:
     - conSertEvaluationConfiguration
    """


    def __init__(self, conSertEvaluationConfiguration=None,):
        self.conSertEvaluationConfiguration = conSertEvaluationConfiguration

    def read(self, iprot):
        if iprot._fast_decode is not None and isinstance(iprot.trans, TTransport.CReadableTransport) and self.thrift_spec is not None:
            iprot._fast_decode(self, iprot, [self.__class__, self.thrift_spec])
            return
        iprot.readStructBegin()
        while True:
            (fname, ftype, fid) = iprot.readFieldBegin()
            if ftype == TType.STOP:
                break
            if fid == 1:
                if ftype == TType.STRUCT:
                    self.conSertEvaluationConfiguration = TConSertEvaluationConfiguration()
                    self.conSertEvaluationConfiguration.read(iprot)
                else:
                    iprot.skip(ftype)
            else:
                iprot.skip(ftype)
            iprot.readFieldEnd()
        iprot.readStructEnd()

    def write(self, oprot):
        if oprot._fast_encode is not None and self.thrift_spec is not None:
            oprot.trans.write(oprot._fast_encode(self, [self.__class__, self.thrift_spec]))
            return
        oprot.writeStructBegin('EvaluateConSerts_args')
        if self.conSertEvaluationConfiguration is not None:
            oprot.writeFieldBegin('conSertEvaluationConfiguration', TType.STRUCT, 1)
            self.conSertEvaluationConfiguration.write(oprot)
            oprot.writeFieldEnd()
        oprot.writeFieldStop()
        oprot.writeStructEnd()

    def validate(self):
        return

    def __repr__(self):
        L = ['%s=%r' % (key, value)
             for key, value in self.__dict__.items()]
        return '%s(%s)' % (self.__class__.__name__, ', '.join(L))

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.__dict__ == other.__dict__

    def __ne__(self, other):
        return not (self == other)
all_structs.append(EvaluateConSerts_args)
EvaluateConSerts_args.thrift_spec = (
    None,  # 0
    (1, TType.STRUCT, 'conSertEvaluationConfiguration', [TConSertEvaluationConfiguration, None], None, ),  # 1
)


class EvaluateConSerts_result(object):
    """
    Attributes:
     - success
     - EpsilonScriptExecutionException
    """


    def __init__(self, success=None, EpsilonScriptExecutionException=None,):
        self.success = success
        self.EpsilonScriptExecutionException = EpsilonScriptExecutionException

    def read(self, iprot):
        if iprot._fast_decode is not None and isinstance(iprot.trans, TTransport.CReadableTransport) and self.thrift_spec is not None:
            iprot._fast_decode(self, iprot, [self.__class__, self.thrift_spec])
            return
        iprot.readStructBegin()
        while True:
            (fname, ftype, fid) = iprot.readFieldBegin()
            if ftype == TType.STOP:
                break
            if fid == 0:
                if ftype == TType.STRUCT:
                    self.success = TConSertEvaluationResult()
                    self.success.read(iprot)
                else:
                    iprot.skip(ftype)
            elif fid == 1:
                if ftype == TType.STRUCT:
                    self.EpsilonScriptExecutionException = TAbstractEpsilonScriptExecutionException()
                    self.EpsilonScriptExecutionException.read(iprot)
                else:
                    iprot.skip(ftype)
            else:
                iprot.skip(ftype)
            iprot.readFieldEnd()
        iprot.readStructEnd()

    def write(self, oprot):
        if oprot._fast_encode is not None and self.thrift_spec is not None:
            oprot.trans.write(oprot._fast_encode(self, [self.__class__, self.thrift_spec]))
            return
        oprot.writeStructBegin('EvaluateConSerts_result')
        if self.success is not None:
            oprot.writeFieldBegin('success', TType.STRUCT, 0)
            self.success.write(oprot)
            oprot.writeFieldEnd()
        if self.EpsilonScriptExecutionException is not None:
            oprot.writeFieldBegin('EpsilonScriptExecutionException', TType.STRUCT, 1)
            self.EpsilonScriptExecutionException.write(oprot)
            oprot.writeFieldEnd()
        oprot.writeFieldStop()
        oprot.writeStructEnd()

    def validate(self):
        return

    def __repr__(self):
        L = ['%s=%r' % (key, value)
             for key, value in self.__dict__.items()]
        return '%s(%s)' % (self.__class__.__name__, ', '.join(L))

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.__dict__ == other.__dict__

    def __ne__(self, other):
        return not (self == other)
all_structs.append(EvaluateConSerts_result)
EvaluateConSerts_result.thrift_spec = (
    (0, TType.STRUCT, 'success', [TConSertEvaluationResult, None], None, ),  # 0
    (1, TType.STRUCT, 'EpsilonScriptExecutionException', [TAbstractEpsilonScriptExecutionException, None], None, ),  # 1
)
fix_spec(all_structs)
del all_structs

