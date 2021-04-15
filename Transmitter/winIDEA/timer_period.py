# Initialization script
# Created by winIDEA
import isystem.connect as ic
cmgr = ic.ConnectionMgr()
cmgr.connectMRU('')
debug = ic.CDebugFacade(cmgr)

try:
  debug.modify(ic.IConnectDebug.fMonitor, '@"TIM2/PSC"', '8')
  debug.modify(ic.IConnectDebug.fMonitor, '@"TIM2/ARR"', '8547')
  debug.modify(ic.IConnectDebug.fMonitor, '@"TIM21/PSC"', '9999')
  debug.modify(ic.IConnectDebug.fMonitor, '@"TIM21/ARR"', '6400')
except Exception as ex:
  print (ex)