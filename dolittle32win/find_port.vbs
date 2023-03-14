'Option Explicit
Dim selftpath
selfpath = GetSelfDirectory()

dim com_port
com_port = CheckConnectStuduino()
If (com_port = "Err") Then

    WScript.Quit(1)
End If

WScript.StdOut.Write (com_port)
WScript.Quit(0)

'------------------------------------------------------------
'------------------------------------------------------------
'------------------------------------------------------------


Function CompStuduino(strText)
	Dim objRE
	Set objRE = CreateObject("VBScript.RegExp")
	With objRE

	    .Pattern = "(COM\d*\s)- Prolific -\sUSB\\VID_067B&PID_2303.+"
	    .IgnoreCase = True
	    .Global = False

	    CompStuduino = .Test(strText)
	'     If CompStuduino = True Then
	'        MsgBox "ありました"
	'     Else
	'        MsgBox "ありませんでした"
	'     End If
	End With
	Set objRE = Nothing
End Function

Function CheckConnectStuduino()
    Dim WshShell, outExec, StdOut
    Dim strCmd
    Dim retStr
    retStr = Null
    
    strCmd = selfpath+"\studuino\hardware\tools\listComPorts.exe"

    Set WshShell = CreateObject("WScript.Shell")
    Set outExec = WshShell.Exec(strCmd)
    Do While outExec.Status = 0
         WScript.Sleep 100
    Loop
    Set StdOut = outExec.StdOut
    
    
    Dim count
    count = 0
    
    Do While Not StdOut.AtEndOfStream
        Dim outStr
        outStr = StdOut.ReadLine()
        If CompStuduino(outStr) Then
            Dim aryStr
            aryStr = Split(outStr)
            retStr = "" + aryStr(0)
            count = count + 1
        End If
    Loop
    
    If count = 1 Then
       CheckConnectStuduino = retStr
    ElseIf count > 1 Then
        CheckConnectStuduino = "Err"
    Else
        CheckConnectStuduino = "Err"
    End If
     
    'Dim  StdErr
    'Dim  szStr
    'Set StdErr = outExec.StdErr
    'szStr = "STDERR" & vbCrLf
    'Do While Not StdErr.AtEndOfStream
    '   szStr = szStr & StdErr.ReadLine() & vbCrLf
    'Loop
    Set WshShell = Nothing
End Function


Function GetSelfDirectory()

	'pathLen = Len(wscript.scriptfullname) - Len(wscript.scriptname)
	'GetSelfDirectory = Left(wscript.scriptfullname, pathLen)
         Dim aryStr
         aryStr = Split(ModulePath,"\")

	'上記はexeファイル変換後に使えないため変更
	dim name
	name = ModulePath
	pathLen = Len(ModulePath) - Len(aryStr(Ubound(aryStr)))
	GetSelfDirectory = Left(ModulePath, pathLen)
End Function


Function ModulePath()
  dim T,fHandle
  ModulePath=WScript.ScriptFullName
  T=lcase(ModulePath)
  T=left(T,len(T)-4)
  if right(T,4)<>".tmp" then exit function
  on error resume next
  set fHandle=CreateObject("Scripting.FileSystemObject").OpenTextFile(T)
  T=fHandle.ReadLine
  fHandle.close
  if err.number=0 then ModulePath=T
End Function